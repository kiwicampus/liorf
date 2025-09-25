#include "utility.h"
#include "liorf/cloud_info.h"
#include "liorf/save_map.h"
#include "dumpGraph.h"
#include "liorf/FactorGraphLoader.h"
#include <iomanip>
#include <gtsam/base/serialization.h>
// <!-- liorf_yjz_lucky_boy -->
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <limits>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <liorf/refine_map.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/base/serialization.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "Scancontext.h"

using namespace gtsam;

BOOST_CLASS_EXPORT_GUID(gtsam::GPSFactor, "gtsam::GPSFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<Pose3>, "gtsam::BetweenFactor<Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<Pose3>, "gtsam::PriorFactor<Pose3>");


using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

class RollingBuffer {
private:
    std::deque<double> buffer;
    double sum = 0.0;
    const size_t maxSize = 10;

public:
    void addSample(double sample) {
        if (buffer.size() == maxSize) {
            // Remove the oldest sample and subtract its value from the sum
            sum -= buffer.front();
            buffer.pop_front();
        }
        // Add the new sample and update the sum
        buffer.push_back(sample);
        sum += sample;
    }

    double getAvg() const {
        if(buffer.empty())
        {
            return 0.0;
        }
        return sum/buffer.size();
    }
};

struct LoopClosureResult
{
    bool success;
    gtsam::Pose3 pose;
    pair<int, int> indexes;
    gtsam::SharedNoiseModel noise;
};

class mapOptimization : public ParamServer
{

public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Publisher pubSLAMInfo;
    ros::Publisher pubGpsOdom;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;
    ros::Subscriber subInitialPose;

    ros::ServiceServer srvSaveMap;
    ros::ServiceServer srvUseGps;
    ros::ServiceServer srvRefineMap;
    bool useGps = true;
    int addedGpsFactors = 0;

    std::deque<nav_msgs::Odometry> gpsQueue;
    bool first_gps_added = false;
    liorf::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    double laserCloudRawTime;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;
    double prevtimeLaserInfoCur;
    RollingBuffer speeds;
    sensor_msgs::PointCloud2 prevRosCloud;
    Eigen::Affine3f lastIncre;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    // vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    GeographicLib::LocalCartesian gps_trans_;

    // scancontext loop closure
    SCManager scManager;

    // data saver
    std::fstream pgSaveStream; // pg: pose-graph 
    std::fstream pgTimeSaveStream; // pg: pose-graph 
    std::vector<std::string> edges_str;
    std::vector<std::string> vertices_str;

    // Complete factor graph saver for offline reconstruction
    NonlinearFactorGraph completeFactorGraph;  // Store all factors
    Values completeInitialEstimate;            // Store all initial estimates
    std::fstream factorSaveStream;             // Stream to save factors
    std::vector<std::string> all_factors_str;  // String representation of all factors

    // graph dump saver
    std::vector<double> keyframeStamps;

    std::string saveSCDDirectory;
    std::string saveNodePCDDirectory;
    
    // Session loading
    std::unique_ptr<FactorGraphLoader> sessionLoader_;
    bool sessionLoaded_;
    std::string sessionBasePath_;

    tf::TransformListener tfListener;
    Eigen::Affine3f lidarToBaseLink;
    bool hasTransform = false;
    
    // Relocalization state after session loading
    bool waitingForInitialPose = false;
    bool initialPoseReceived = false;
    gtsam::Pose3 receivedInitialPose;
    int closestSessionPoseIndex = -1;

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/trajectory", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("liorf/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("liorf/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("liorf/mapping/path", 1);

        subCloud = nh.subscribe<liorf::cloud_info>("liorf/deskew/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS   = nh.subscribe<sensor_msgs::NavSatFix> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subInitialPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &mapOptimization::initialPoseHandler, this, ros::TransportHints().tcpNoDelay());

        srvSaveMap  = nh.advertiseService("liorf/save_map", &mapOptimization::saveMapService, this);
        srvUseGps = nh.advertiseService("liorf/use_gps", &mapOptimization::setUseGps, this);
        srvRefineMap = nh.advertiseService("liorf/refine_map", &mapOptimization::refineMapService, this);

        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/liorf/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("liorf/mapping/cloud_registered_raw", 1);

        pubSLAMInfo           = nh.advertise<liorf::cloud_info>("liorf/mapping/slam_info", 1);
        pubGpsOdom            = nh.advertise<nav_msgs::Odometry> ("liorf/mapping/gps_odom", 1);

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
        downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        allocateMemory();

        // giseop
        // create directory and remove old files;
        // savePCDDirectory = std::getenv("HOME") + savePCDDirectory; // rather use global path 
        int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        unused = system((std::string("mkdir ") + savePCDDirectory).c_str());

        saveSCDDirectory = savePCDDirectory + "SCDs/"; // SCD: scan context descriptor 
        unused = system((std::string("exec rm -r ") + saveSCDDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveSCDDirectory).c_str());

        saveNodePCDDirectory = savePCDDirectory + "Scans/";
        unused = system((std::string("exec rm -r ") + saveNodePCDDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveNodePCDDirectory).c_str());

        pgSaveStream = std::fstream(savePCDDirectory + "singlesession_posegraph.g2o", std::fstream::out);
        pgTimeSaveStream = std::fstream(savePCDDirectory + "times.txt", std::fstream::out); pgTimeSaveStream.precision(dbl::max_digits10);
        
        // Check if session loading is requested
        sessionBasePath_ = loadSessionPath;
        if (!sessionBasePath_.empty()) {
        // if (false) {}
            ROS_INFO("Loading session from: %s", sessionBasePath_.c_str());
            sessionLoader_ = std::make_unique<FactorGraphLoader>();
            if (sessionLoader_->loadSession(sessionBasePath_)) {
                sessionLoaded_ = true;
                
                // Override GPS datum if session has it
                if (sessionLoader_->hasGPSDatum()) {
                    ROS_INFO("Overriding GPS datum with session data: lat=%.6f, lon=%.6f, alt=%.2f", 
                             sessionLoader_->getGPSLatitude(), 
                             sessionLoader_->getGPSLongitude(), 
                             sessionLoader_->getGPSAltitude());
                    gps_trans_.Reset(sessionLoader_->getGPSLatitude(), 
                                   sessionLoader_->getGPSLongitude(), 
                                   sessionLoader_->getGPSAltitude());
                }
                
                loadSessionData();
            } else {
                ROS_ERROR("Failed to load session from: %s", sessionBasePath_.c_str());
                sessionLoaded_ = false;
            }
        } else {
            sessionLoaded_ = false;
            ROS_INFO("No session loading requested, starting SLAM from scratch");
        }
    }

    bool setUseGps(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        useGps = req.data;
        res.success = true;
        return true;
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());


        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void writeVertex(const int _node_idx, const gtsam::Pose3& _initPose)
    {
        gtsam::Point3 t = _initPose.translation();
        gtsam::Rot3 R = _initPose.rotation();

        std::string curVertexInfo {
            "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
            + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
            + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
            + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

        // pgVertexSaveStream << curVertexInfo << std::endl;
        vertices_str.emplace_back(curVertexInfo);
    }
    
    void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose)
    {
        gtsam::Point3 t = _relPose.translation();
        gtsam::Rot3 R = _relPose.rotation();

        std::string curEdgeInfo {
            "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
            + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
            + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
            + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

        // pgEdgeSaveStream << curEdgeInfo << std::endl;
        edges_str.emplace_back(curEdgeInfo);
    }


    void adjustForRotation()
    {
        if (hasTransform)
        {
        pcl::transformPointCloud(*laserCloudSurfLast, *laserCloudSurfLast, lidarToBaseLink);
                
        }
        else
        {
            ROS_WARN("Could not transform base_link to livox_link:");
            return;
        }
    }

    void laserCloudInfoHandler(const liorf::cloud_infoConstPtr& msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        if (!hasTransform) {
            try {
                tf::StampedTransform lidar_transform;
                tfListener.waitForTransform(lidarFrame, "base_link", ros::Time(0), ros::Duration(0.1));
                tfListener.lookupTransform(lidarFrame, "base_link", ros::Time(0), lidar_transform);
                tf::Vector3 trans = lidar_transform.getOrigin();
                tf::Quaternion quat = lidar_transform.getRotation();
                
                lidarToBaseLink = Eigen::Affine3f::Identity();
                lidarToBaseLink.translate(Eigen::Vector3f(trans.x(), trans.y(), trans.z()));
                Eigen::Quaternionf eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());
                lidarToBaseLink.rotate(eigen_quat);
                lidarToBaseLink = lidarToBaseLink.inverse();
                hasTransform = true;
                ROS_INFO("Got transform from %s to base_link", lidarFrame.c_str());
            } catch (tf::TransformException ex) {
                ROS_WARN_THROTTLE(1.0, "Failed to get transform from %s to base_link: %s", lidarFrame.c_str(), ex.what());
                return;
            }
        }

        if(!gpsQueue.empty())
        {
            ROS_INFO_THROTTLE(30.0, "GPS messages are %f ahead of Lidar and your current setting is %f. Take this into account in case an adjustment is necessary", gpsQueue.front().header.stamp.toSec() - timeLaserInfoCur, mappingGpsCloudTimeOffset);
        }
        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudSurfLast);
        laserCloudRawTime = cloudInfo.header.stamp.toSec(); // giseop save node time

        // TODO
        // ......
        // remapping
        // ......
        // END

        std::lock_guard<std::mutex> lock(mtx);

        // Handle relocalization after session loading
        if (waitingForInitialPose) {
            if (initialPoseReceived) {
                ROS_INFO("Processing relocalization with received initial pose");
                if (performRelocalization()) {
                    waitingForInitialPose = false;
                    initialPoseReceived = false;
                    ROS_INFO("Relocalization successful, resuming normal mapping");
                } else {
                    ROS_WARN("Relocalization failed, still waiting for initial pose");
                    return;
                }
            } else {
                ROS_INFO_THROTTLE(5.0, "Waiting for initial pose from /initialpose or GPS...");
                return;
            }
        }

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            adjustForRotation();

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }
    }

    void initialPoseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& poseMsg)
    {
        if (!waitingForInitialPose) {
            return; // Not in relocalization mode
        }
        
        ROS_INFO("Received initial pose from /initialpose topic");
        
        // Convert ROS pose to GTSAM pose
        const auto& pos = poseMsg->pose.pose.position;
        const auto& quat = poseMsg->pose.pose.orientation;
        
        gtsam::Rot3 rotation(quat.w, quat.x, quat.y, quat.z);
        gtsam::Point3 translation(pos.x, pos.y, pos.z);
        receivedInitialPose = gtsam::Pose3(rotation, translation);
        
        initialPoseReceived = true;
        ROS_INFO("Initial pose set to: [%.3f, %.3f, %.3f]", pos.x, pos.y, pos.z);
    }

    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
    {
        if (gpsMsg->status.status != 0 && gpsMsg->status.status != 2)
            return;

        Eigen::Vector3d trans_local_;
        static bool first_gps = false;
        if (!first_gps) {
            first_gps = true;
            if(mappingGpsDatumLatitude != 0.0 || mappingGpsDatumLongitude != 0)
            {
                gps_trans_.Reset(mappingGpsDatumLatitude, mappingGpsDatumLongitude, mappingGpsDatumAltitude);
                std::cout << "First pose saved from Datum: latitude " << mappingGpsDatumLatitude << ", longitude: " << mappingGpsDatumLongitude << std::endl;
            }
            else
            {
                std::cout << "First pose saved from GPS: latitude " << gpsMsg->latitude << ", longitude: " << gpsMsg->longitude << std::endl;
                gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
            }
        }

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        // Check if we're waiting for initial pose and can use GPS
        if (waitingForInitialPose && !initialPoseReceived) {
            ROS_INFO("Using GPS for initial pose relocalization");
            gtsam::Rot3 rotation = gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0); // GPS doesn't provide orientation
            gtsam::Point3 translation(trans_local_[0], trans_local_[1], trans_local_[2]);
            receivedInitialPose = gtsam::Pose3(rotation, translation);
            initialPoseReceived = true;
            ROS_INFO("Initial pose set from GPS: [%.3f, %.3f, %.3f]", trans_local_[0], trans_local_[1], trans_local_[2]);
        }

        nav_msgs::Odometry gps_odom;
        gps_odom.header = gpsMsg->header;
        gps_odom.header.frame_id = odometryFrame;
        gps_odom.pose.pose.position.x = trans_local_[0];
        gps_odom.pose.pose.position.y = trans_local_[1];
        gps_odom.pose.pose.position.z = trans_local_[2];
        gps_odom.pose.covariance[0] = gpsMsg->position_covariance[0];
        gps_odom.pose.covariance[7] = gpsMsg->position_covariance[4];
        gps_odom.pose.covariance[14] = gpsMsg->position_covariance[8];
        gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        // pubGpsOdom.publish(gps_odom);
        gpsQueue.push_back(gps_odom);
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    bool saveMapService(liorf::save_mapRequest& req, liorf::save_mapResponse& res)
    {
        if (savePCD == false)
            return false;

        // save pose graph (runs when programe is closing)
        cout << "****************************************************" << endl; 
        cout << "Saving the posegraph ..." << endl; // giseop

        for(auto& _line: vertices_str)
            pgSaveStream << _line << std::endl;
        for(auto& _line: edges_str)
            pgSaveStream << _line << std::endl;

        pgSaveStream.close();
        // pgVertexSaveStream.close();
        // pgEdgeSaveStream.close();

        const std::string kitti_format_pg_filename {savePCDDirectory + "optimized_poses.txt"};
        saveOptimizedVerticesKITTIformat(isamCurrentEstimate, kitti_format_pg_filename);

        // save map 
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        // save key frame transformations
        pcl::io::savePCDFileBinary(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        // extract global point cloud map        
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        // down-sample and save surf cloud
        // downSizeFilterSurf.setInputCloud(globalSurfCloud);
        // downSizeFilterSurf.filter(*globalSurfCloudDS);
        // pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);
        // down-sample and save global point cloud map
        *globalMapCloud += *globalSurfCloud;
        pcl::io::savePCDFileBinary(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        dump(savePCDDirectory + "graph/", *isam, isamCurrentEstimate,  keyframeStamps,surfCloudKeyFrames);
        // Save YAML factor graph
        dumpYAML(savePCDDirectory + "graph/", *isam, isamCurrentEstimate,  keyframeStamps,surfCloudKeyFrames, &gps_trans_);
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed" << endl;

      return true;
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            // ROS_INFO("Visualizing global map with %zu keyframes and path with %zu poses", cloudKeyPoses6D->size(), globalPath.poses.size());
            publishGlobalMap();
            publishFrames();
        }

        if (savePCD == false)
            return;

        // save pose graph (runs when programe is closing) - No longer used, we use the service for that purpose
        // cout << "****************************************************" << endl; 
        // cout << "Saving the posegraph ..." << endl; // giseop

        // for(auto& _line: vertices_str)
        //     pgSaveStream << _line << std::endl;
        // for(auto& _line: edges_str)
        //     pgSaveStream << _line << std::endl;

        // pgSaveStream.close();
        // // pgVertexSaveStream.close();
        // // pgEdgeSaveStream.close();

        // const std::string kitti_format_pg_filename {savePCDDirectory + "optimized_poses.txt"};
        // saveOptimizedVerticesKITTIformat(isamCurrentEstimate, kitti_format_pg_filename);

        // // save map 
        // cout << "****************************************************" << endl;
        // cout << "Saving map to pcd files ..." << endl;
        // // save key frame transformations
        // pcl::io::savePCDFileASCII(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        // pcl::io::savePCDFileASCII(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        // // extract global point cloud map        
        // pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        // for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
        //     *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
        //     cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        // }
        // // down-sample and save surf cloud
        // downSizeFilterSurf.setInputCloud(globalSurfCloud);
        // downSizeFilterSurf.filter(*globalSurfCloudDS);
        // pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);
        // // down-sample and save global point cloud map
        // *globalMapCloud += *globalSurfCloud;
        // pcl::io::savePCDFileASCII(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        // dump(savePCDDirectory + "graph/", *isam, isamCurrentEstimate,  keyframeStamps,surfCloudKeyFrames);
        // // Save YAML factor graph
        // dumpYAML(savePCDDirectory + "graph/", *isam, isamCurrentEstimate,  keyframeStamps,surfCloudKeyFrames, &gps_trans_);
        // cout << "****************************************************" << endl;
        // cout << "Saving map to pcd files completed" << endl;
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (common_lib_->pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
        pcl::io::savePCDFileBinary(savePCDDirectory + "/lio_sam_map.pcd", *globalMapKeyFramesDS);
    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performRSLoopClosure();
            // performSCLoopClosure(); // commented because this created false loops in corridors, since all of them look the same
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performRSLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // LoopClosureResult closure_result;
        // tryLoopClosure(loopKeyCur, loopKeyPre, historyKeyframeFitnessScore, closure_result);

        // extract cloud
        std::cout << "trying to close loop between pose " << loopKeyCur << " and " << loopKeyPre << std::endl;
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, -1);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        std::cout << "icp for loop closing has converged: " <<  icp.hasConverged() << " with fitness score " << icp.getFitnessScore() << std::endl;

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    void tryLoopClosure(int loopKeyCur, int loopKeyPre, float icp_convergence_threshold, LoopClosureResult& result)
    {
        std::cout << "trying to close loop between pose " << loopKeyCur << " and " << loopKeyPre << std::endl;
        result.success = false;
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, -1);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, -1);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        std::cout << "icp for loop closing has converged: " <<  icp.hasConverged() << " with fitness score " << icp.getFitnessScore() << std::endl;

        if (icp.hasConverged() == false || icp.getFitnessScore() > icp_convergence_threshold)
        {
            return;
        }

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        result.pose = poseFrom.between(poseTo);
        result.indexes = make_pair(loopKeyCur, loopKeyPre);
        result.noise = constraintNoise;
        result.success = true;
    }

    // copy from sc-lio-sam
    void performSCLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        // first: nn index, second: yaw diff 
        auto detectResult = scManager.detectLoopClosureID(); 
        int loopKeyCur    = copy_cloudKeyPoses3D->size() - 1;;
        int loopKeyPre    = detectResult.first;
        float yawDiffRad  = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
        if( loopKeyPre == -1)
            return;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return;

        // std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            int base_key = 0;
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, base_key); // giseop 

            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // giseop 
        pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        // giseop, robust kernel for a SC loop
        float robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise; 
        robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
        ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID, int id_to_search = -1)
    {
        int loopKeyCur;
        float query_cloud_time;
        if(id_to_search < 0)
        {
            loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
            query_cloud_time = timeLaserInfoCur;
        }
        else
        {
            loopKeyCur = id_to_search; 
            query_cloud_time = copy_cloudKeyPoses6D->points[id_to_search].time;
        }
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
        {
            ROS_INFO("loopKeyCur %i is already in loopIndexContainer", loopKeyCur);
            return false;
        }

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->points[loopKeyCur], historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - query_cloud_time) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int& loop_index)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;

            int select_loop_index = (loop_index != -1) ? loop_index : key + i;
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[select_loop_index]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty() || sessionLoaded_)
        {
            if(!sessionLoaded_){
                transformTobeMapped[0] = cloudInfo.imuRollInit;
                transformTobeMapped[1] = cloudInfo.imuPitchInit;
                transformTobeMapped[2] = cloudInfo.imuYawInit;
            }
            sessionLoaded_ = false;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                if (transIncre.translation().norm() > 5.0) {
                    // If so, set transIncre to the previous transIncre
                    std::cout << "this sample is bad because delta is " << transIncre.translation().norm()  << " m/. Setting covariance to the last increment" << std::endl; 
                    transIncre = lastIncre;
                }
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;
                lastIncre = transIncre;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true && imuType)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (common_lib_->pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp;
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding surf key frames (or map)
        downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        // #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[2]);
        float crx = cos(transformTobeMapped[2]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].x;
            pointOri.y = laserCloudOri->points[i].y;
            pointOri.z = laserCloudOri->points[i].z;
            // lidar -> camera
            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;

            float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                      + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

            float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                      + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                      + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

            float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                      + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                      + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
              
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arx;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            return;
        }

        if (laserCloudSurfLastDSNum > 30)
        {
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true && imuType)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        // if(!first_gps_added)
        // {
        //     std::cout << "Skipping first odom because gps has not been received\n";
        //     return;
        // }
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));

            writeVertex(0, trans2gtsamPose(transformTobeMapped));

        }else{
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtsam::Pose3 relPose = poseFrom.between(poseTo);
            double position_noise = 1e-6;
            bool degenerate = false;
            if(prevtimeLaserInfoCur == 0)
            {
                prevtimeLaserInfoCur = timeLaserInfoCur;
            }
            else
            {
                double speed = relPose.translation().norm() / (timeLaserInfoCur - prevtimeLaserInfoCur);
                if (speed > speeds.getAvg() + 0.2 && speed > 1.5)
                {
                    position_noise = 1e-1;
                    std::cout << "this sample is bad because speed is " << speed << " m/s and avg is " << speeds.getAvg() << ". Setting covariance to " << position_noise << std::endl; 
                    degenerate = true;
                    poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.at(cloudKeyPoses6D->points.size() -2));
                    poseTo   = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
                    relPose = poseFrom.between(poseTo);
                    relPose = gtsam::Pose3(gtsam::Rot3(), relPose.translation());
                }
                else
                { 
                speeds.addSample(speed);
                }
            }
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << position_noise, position_noise, position_noise, 1e-4, 1e-4, 1e-4).finished());

            prevtimeLaserInfoCur = timeLaserInfoCur;
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);

            writeVertex(cloudKeyPoses3D->size(), poseTo);
            writeEdge({cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size()}, relPose); // giseop
        }
    }

    void addGPSFactor()
    {
        if (!useGps) // Check the parameter
            return;

        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 1.0 && first_gps_added)
            {
                return;
            }
        }

        // pose covariance small, no need to correct
        // if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        //     return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() - mappingGpsCloudTimeOffset < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() - mappingGpsCloudTimeOffset > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                {
                    continue;
                }

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                // if (!useGpsElevation || addedGpsFactors > 25)
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                {
                    continue;
                }

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                float adding_threshold = addedGpsFactors < mappingGpsSwitchThreshold ? mappingGpsIntervalFirstPoses : mappingGpsIntervalGeneral;
                if (common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < adding_threshold)
                {
                    if(first_gps_added)
                    {
                        continue;
                    }
                }
                else
                {
                    lastGPSPoint = curGPSPoint;
                }

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, mappingGpsFactorSigma), max(noise_y, mappingGpsFactorSigma), max(noise_z, 5.0f*mappingGpsFactorSigma);
                // Vector3 << max(noise_x, 0.02f), max(noise_y, 0.02f), max(noise_z, 0.02f);
                // Vector3 << noise_x, noise_y, noise_z;
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);
                addedGpsFactors++;
                pubGpsOdom.publish(thisGPS);
                if(!first_gps_added)
                {
                    std::cout << "adding firts gps\n";
                    first_gps_added = true;
                }

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            // gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            auto noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
            writeEdge({indexFrom, indexTo}, poseBetween); // giseop
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
        {
            return;
        }
        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        // Keep the complete factor graph for offline reconstruction
        gtSAMgraph.resize(0);  // Commented out to preserve complete graph
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        keyframeStamps.push_back(timeLaserInfoCur);

        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 


        // FOR NOW WE ARE NOT SAVING SCDS AND CLOUDS
        // if( sc_input_type == SCInputType::SINGLE_SCAN_FULL )
        // {
        //     pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
        //     pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);

        //     scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        // }  
        // else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT)
        // { 
        //     scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
        // }
        // else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT)
        // { 
        //     pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
        //     loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, historyKeyframeSearchNum, -1);
        //     scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud); 
        // }

        //  // save sc data
        // const auto& curr_scd = scManager.getConstRefRecentSCD();
        // std::string curr_scd_node_idx = padZeros(scManager.polarcontexts_.size() - 1);

        // saveSCD(saveSCDDirectory + curr_scd_node_idx + ".scd", curr_scd);


        // // save keyframe cloud as file giseop
        // bool saveRawCloud { true };
        // pcl::PointCloud<PointType>::Ptr thisKeyFrameCloud(new pcl::PointCloud<PointType>());
        // *thisKeyFrameCloud += *thisSurfKeyFrame;
        // try
        // {
        //     pcl::io::savePCDFileBinary(saveNodePCDDirectory + curr_scd_node_idx + ".pcd", *thisKeyFrameCloud);
        // }
        // catch(const std::exception& e)
        // {
        //     std::cerr << e.what() << '\n';
        // }
        
        // pgTimeSaveStream << laserCloudRawTime << std::endl;

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true && imuType)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            std::cout << "cloudKeyPoses3D is empty" << std::endl;
            return;
        }
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            pcl::transformPointCloud(*cloudOut, *cloudOut, lidarToBaseLink);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0)
        {
            if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            {
                liorf::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }
    
    void loadSessionData() {
        if (!sessionLoaded_ || !sessionLoader_) {
            return;
        }
        
        ROS_INFO("Loading session data into mapping system...");
        
        // Copy ISAM object
        delete isam;
        isam = new ISAM2(*sessionLoader_->getISAM());
        ROS_INFO("ISAM copied with %zu factors", isam->getFactorsUnsafe().size());
        
        // Get the optimized estimate from the session
        // Note: Do NOT overwrite initialEstimate with session data since ISAM already contains all session variables
        // initialEstimate should remain empty and only contain NEW variables that will be added
        isamCurrentEstimate = sessionLoader_->getOptimizedEstimate();
        
        // isamCurrentEstimate.print("isamCurrentEstimate");

        // Reset transform and set waiting state for relocalization
        transformTobeMapped[0] = 0.0;
        transformTobeMapped[1] = 0.0;
        transformTobeMapped[2] = 0.0; 
        transformTobeMapped[3] = 0.0;
        transformTobeMapped[4] = 0.0;
        transformTobeMapped[5] = 0.0;
        
        // Set relocalization state
        waitingForInitialPose = true;
        initialPoseReceived = false;
        closestSessionPoseIndex = -1;
        ROS_INFO("Session loaded. Waiting for initial pose from /initialpose topic or GPS...");
        
        // Copy keyframe poses
        const auto& keyframe_data = sessionLoader_->getKeyframeData();
        cloudKeyPoses3D->clear();
        cloudKeyPoses6D->clear();
        
        for (const auto& keyframe_pair : keyframe_data) {
            int id = keyframe_pair.first;
            const auto& keyframe = keyframe_pair.second;
            const auto& pose = keyframe->pose;
            // Add to 3D poses
            PointType pose3d;
            pose3d.x = pose.translation().x();
            pose3d.y = pose.translation().y();
            pose3d.z = pose.translation().z();
            pose3d.intensity = id;
            cloudKeyPoses3D->push_back(pose3d);
            
            // Add to 6D poses
            PointTypePose pose6d;
            pose6d.x = pose.translation().x();
            pose6d.y = pose.translation().y();
            pose6d.z = pose.translation().z();
            pose6d.roll = pose.rotation().roll();
            pose6d.pitch = pose.rotation().pitch();
            pose6d.yaw = pose.rotation().yaw();
            pose6d.intensity = id;
            pose6d.time = keyframe->timestamp; // Use timestamp from keyframe data
            cloudKeyPoses6D->push_back(pose6d);
        }
        
        // Copy timestamps
        keyframeStamps.clear();
        for (const auto& keyframe_pair : keyframe_data) {
            keyframeStamps.push_back(keyframe_pair.second->timestamp);
        }
        
        // Copy point clouds using the new structure
        surfCloudKeyFrames.clear();
        
        // Use the already declared keyframe_data variable
        for (const auto& keyframe_pair : keyframe_data) {
            const auto& keyframe = keyframe_pair.second;
            
            // Create a copy of the cloud for this keyframe
            pcl::PointCloud<PointType>::Ptr keyframeCloud(new pcl::PointCloud<PointType>(*keyframe->cloud));
            surfCloudKeyFrames.push_back(keyframeCloud);
        }
        
        ROS_INFO("Session loaded with %zu keyframes", keyframe_data.size());
        
        // Set GPS datum if available
        if (sessionLoader_->hasGPSDatum()) {
            gps_trans_.Reset(sessionLoader_->getGPSLatitude(), 
                           sessionLoader_->getGPSLongitude(), 
                           sessionLoader_->getGPSAltitude());
            ROS_INFO("GPS datum set from session: %.6f, %.6f, %.2f", 
                    sessionLoader_->getGPSLatitude(), 
                    sessionLoader_->getGPSLongitude(), 
                    sessionLoader_->getGPSAltitude());
        }
        
        // Extract loop closures from BetweenFactors that don't join consecutive poses
        extractLoopClosuresFromSession();
        extractAndPublishGPSFactors();
        
        // Reconstruct global path for visualization
        globalPath.poses.clear();
        ROS_INFO("Updating path with %zu keyframes", cloudKeyPoses6D->size());
        for (const auto& pose6d : cloudKeyPoses6D->points) {
            updatePath(pose6d);
        }
        
        // Update copy arrays for loop closure visualization
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        
        ROS_INFO("Session data loaded: %zu keyframes, %zu factors", 
                cloudKeyPoses3D->size(), gtSAMgraph.size());
    }
    
    void extractLoopClosuresFromSession() {
        if (!sessionLoaded_ || !sessionLoader_) {
            return;
        }
        
        ROS_INFO("Extracting loop closures from cached session data...");
        
        // Clear existing loop closure data
        loopIndexContainer.clear();
        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        
        // Get cached loop closure data from FactorGraphLoader
        const auto& loop_indices = sessionLoader_->getLoopClosureIndices();
        const auto& loop_poses = sessionLoader_->getLoopClosurePoses();
        
        int loop_count = 0;
        for (size_t i = 0; i < loop_indices.size(); ++i) {
            int idx1 = loop_indices[i].first;
            int idx2 = loop_indices[i].second;
            const auto& loop_pose = loop_poses[i];
            
            ROS_INFO("Found loop closure between poses %d and %d", idx1, idx2);
            
            // Add to loop closure containers
            loopIndexContainer[idx1] = idx2;
            loopIndexQueue.push_back(std::make_pair(idx1, idx2));
            loopPoseQueue.push_back(loop_pose);
            
            // Create default noise model for visualization
            gtsam::Vector6 default_noise;
            default_noise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; // 0.1 rad, 0.1 m
            gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Variances(default_noise);
            loopNoiseQueue.push_back(noise_model);
            
            loop_count++;
        }
        
        ROS_INFO("Extracted %d loop closures from session", loop_count);
    }
    
    void extractAndPublishGPSFactors() {
        if (!sessionLoaded_ || !sessionLoader_) {
            return;
        }
        
        ROS_INFO("Extracting GPS factors from cached session data...");
        
        // Get cached GPS factor data from FactorGraphLoader
        const auto& gps_indices = sessionLoader_->getGPSFactorIndices();
        int gps_count = 0;
        
        for (const auto& gps_data : gps_indices) {
            int idx = gps_data.first;
            const auto& gps_point = gps_data.second;
            
            ROS_INFO("Found GPS factor for pose %d at (%.2f, %.2f, %.2f)", idx, gps_point.x(), gps_point.y(), gps_point.z());
            
            // Create and publish GPS odometry message
            nav_msgs::Odometry gps_odom;
            gps_odom.header.stamp = ros::Time::now();
            gps_odom.header.frame_id = odometryFrame;
            gps_odom.child_frame_id = "base_link";
            gps_odom.pose.pose.position.x = gps_point.x();
            gps_odom.pose.pose.position.y = gps_point.y();
            gps_odom.pose.pose.position.z = gps_point.z();
            gps_odom.pose.pose.orientation.w = 1.0; // No rotation info in GPS
            
            // Default covariance for visualization
            gps_odom.pose.covariance[0] = 1.0;   // x
            gps_odom.pose.covariance[7] = 1.0;   // y
            gps_odom.pose.covariance[14] = 5.0;  // z
            
            // Publish to GPS odometry topic
            pubGpsOdom.publish(gps_odom);
            usleep(100000);
            gps_count++;
        }
        
        ROS_INFO("Published %d GPS factors to odometry topic", gps_count);
    }
    
    bool performRelocalization()
    {
        if (cloudKeyPoses3D->empty()) {
            ROS_ERROR("No session poses loaded for relocalization");
            return false;
        }
        
        ROS_INFO("Starting relocalization process...");
        
        // Step 1: Find closest pose in the session to the received initial pose
        closestSessionPoseIndex = findClosestSessionPose(receivedInitialPose);
        if (closestSessionPoseIndex == -1) {
            ROS_ERROR("Failed to find closest session pose");
            return false;
        }
        
        ROS_INFO("Closest session pose found at index %d", closestSessionPoseIndex);
        
        // Step 2: Refine the pose using ICP against the map around the closest pose
        gtsam::Pose3 refinedPose;
        if (!refineInitialPoseWithICP(receivedInitialPose, closestSessionPoseIndex, refinedPose)) {
            ROS_ERROR("ICP refinement failed");
            return false;
        }
        
        ROS_INFO("Pose refined using ICP");
        
        // Step 3: Set transformTobeMapped to the refined pose
        transformTobeMapped[0] = refinedPose.rotation().roll();
        transformTobeMapped[1] = refinedPose.rotation().pitch();
        transformTobeMapped[2] = refinedPose.rotation().yaw();
        transformTobeMapped[3] = refinedPose.translation().x();
        transformTobeMapped[4] = refinedPose.translation().y();
        transformTobeMapped[5] = refinedPose.translation().z();
        
        ROS_INFO("Relocalization complete. New pose: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2],
                transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        
        return true;
    }
    
    int findClosestSessionPose(const gtsam::Pose3& targetPose)
    {
        if (cloudKeyPoses3D->empty()) {
            return -1;
        }
        
        double minDistance = std::numeric_limits<double>::max();
        int closestIndex = -1;
        
        gtsam::Point3 targetTranslation = targetPose.translation();
        
        for (size_t i = 0; i < cloudKeyPoses3D->size(); ++i) {
            const auto& pose = cloudKeyPoses3D->points[i];
            double distance = sqrt(pow(pose.x - targetTranslation.x(), 2) +
                                 pow(pose.y - targetTranslation.y(), 2) +
                                 pow(pose.z - targetTranslation.z(), 2));
            
            if (distance < minDistance) {
                minDistance = distance;
                closestIndex = i;
            }
        }
        
        ROS_INFO("Closest session pose at index %d with distance %.3f meters", closestIndex, minDistance);
        return closestIndex;
    }
    
    bool refineInitialPoseWithICP(const gtsam::Pose3& initialGuess, int mapCenterIndex, gtsam::Pose3& refinedPose)
    {
        ROS_INFO("Refining initial pose using ICP around index %d and pose %f %f %f %f %f %f", mapCenterIndex, initialGuess.translation().x(), initialGuess.translation().y(), initialGuess.translation().z(), initialGuess.rotation().roll(), initialGuess.rotation().pitch(), initialGuess.rotation().yaw());
        
        // Reuse existing loopFindNearKeyframes function to build local map around the target pose
        pcl::PointCloud<PointType>::Ptr localMap(new pcl::PointCloud<PointType>());
        loopFindNearKeyframes(localMap, mapCenterIndex, historyKeyframeSearchNum, -1);
        
        if (localMap->size() < 300) { // Same threshold as loop closure
            ROS_ERROR("Local map too small for ICP: %zu points", localMap->size());
            return false;
        }

        publishCloud(pubIcpKeyFrames, localMap, ros::Time::now(), odometryFrame);
        
        // Get current scan - reuse existing downsampled scan if available
        pcl::PointCloud<PointType>::Ptr currentScan;
        if (laserCloudSurfLastDS && laserCloudSurfLastDS->size() > 0) {
            currentScan = laserCloudSurfLastDS; // Use already downsampled scan
        } else if (laserCloudSurfLast && laserCloudSurfLast->size() > 0) {
            currentScan = laserCloudSurfLast; // Use raw scan
        } else {
            ROS_ERROR("No current scan available for ICP");
            return false;
        }
        
        if (currentScan->size() < 100) {
            ROS_ERROR("Current scan too small for ICP: %zu points", currentScan->size());
            return false;
        }

        // Try different yaw rotations (45-degree increments) to handle GPS heading uncertainty
        // Use OpenMP to parallelize all rotation attempts
        const int numRotations = 8;
        int rotationOrder[] = {0, 180, 90, 270, 45, 225, 135, 315};
        
        // Arrays to store results from parallel execution
        double fitnessScores[numRotations];
        bool converged[numRotations];
        Eigen::Matrix4f finalTransforms[numRotations];
        
        ROS_INFO("Running parallel ICP with %d rotation attempts", numRotations);
        
        // Parallel execution of all rotation attempts
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < numRotations; i++) {
            int rotDegrees = rotationOrder[i];
            
            // Create rotation around Z axis (yaw)
            double rotRadians = rotDegrees * M_PI / 180.0;
            gtsam::Rot3 additionalRotation = gtsam::Rot3::RzRyRx(0, 0, rotRadians);
            gtsam::Pose3 rotatedGuess(additionalRotation * initialGuess.rotation(), initialGuess.translation());
            
            // Transform the current scan with this rotation
            pcl::PointCloud<PointType>::Ptr transformedScan(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*currentScan, *transformedScan, rotatedGuess.matrix().cast<float>());
            
            // Setup ICP with optimized settings for speed
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setInputSource(transformedScan);
            icp.setInputTarget(localMap);
            icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
            icp.setMaximumIterations(50); // Reduced from 100
            icp.setTransformationEpsilon(1e-4); // Relaxed from 1e-6
            icp.setEuclideanFitnessEpsilon(1e-4); // Relaxed from 1e-6
            icp.setRANSACIterations(0); // Disable RANSAC for speed
            icp.setUseReciprocalCorrespondences(true); // Enable for better convergence
            
            // Perform ICP
            pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
            icp.align(*aligned);
            
            // Store results
            fitnessScores[i] = icp.getFitnessScore();
            converged[i] = icp.hasConverged();
            
            if (converged[i]) {
                // Calculate final transformation
                Eigen::Matrix4f icpCorrection = icp.getFinalTransformation();
                Eigen::Matrix4f initialTransform = rotatedGuess.matrix().cast<float>();
                finalTransforms[i] = icpCorrection * initialTransform;
            }
        }
        
        // After parallel execution, find the best result
        double bestFitnessScore = std::numeric_limits<double>::max();
        Eigen::Matrix4f bestFinalTransform;
        bool anyConverged = false;
        int bestRotationDegrees = 0;
        int bestIndex = -1;
        
        for (int i = 0; i < numRotations; i++) {
            int rotDegrees = rotationOrder[i];
            ROS_INFO("Rotation %d degrees: converged=%d, fitness=%.4f", rotDegrees, converged[i], fitnessScores[i]);
            
            if (converged[i] && fitnessScores[i] < 0.7 && fitnessScores[i] < bestFitnessScore) {
                bestFitnessScore = fitnessScores[i];
                bestRotationDegrees = rotDegrees;
                bestIndex = i;
                anyConverged = true;
                bestFinalTransform = finalTransforms[i];
            }
        }
        
        if (anyConverged) {
            ROS_INFO("Best result: %d degrees with fitness score: %.4f", bestRotationDegrees, bestFitnessScore);
            
            // Publish the best result for visualization (recreate the transformation for publishing)
            double rotRadians = bestRotationDegrees * M_PI / 180.0;
            gtsam::Rot3 additionalRotation = gtsam::Rot3::RzRyRx(0, 0, rotRadians);
            gtsam::Pose3 rotatedGuess(additionalRotation * initialGuess.rotation(), initialGuess.translation());
            
            pcl::PointCloud<PointType>::Ptr bestTransformedScan(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*currentScan, *bestTransformedScan, rotatedGuess.matrix().cast<float>());
            publishCloud(pubRecentKeyFrame, bestTransformedScan, ros::Time::now(), odometryFrame);
        }
        
        if (!anyConverged) {
            ROS_ERROR("ICP failed to converge at any rotation. Best fitness was: %.4f", bestFitnessScore);
            return false;
        }
        
        ROS_INFO("Parallel ICP completed: best rotation %d degrees with fitness score: %.4f", bestRotationDegrees, bestFitnessScore);
        
        // Use the best result
        Eigen::Matrix4f finalTransform = bestFinalTransform;
        
        // Convert back to GTSAM pose
        gtsam::Pose3 gtsamTransform(finalTransform.cast<double>());
        refinedPose = gtsamTransform;
        
        return true;
    }

    // Service callback for refining the map
    bool refineMapService(liorf::refine_map::Request &req, liorf::refine_map::Response &res)
    {
        ROS_INFO("=== REFINE MAP SERVICE CALLED ===");
        ROS_INFO("Target pose: %d, Window size: %d, ICP threshold: %f", req.target_pose_index, req.window_size, req.icp_convergence_threshold);
        
        // Copy current poses at the beginning for thread safety
        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();
        
        ROS_INFO("Current map size: %zu poses", copy_cloudKeyPoses3D->size());
        
        if(req.target_pose_index > copy_cloudKeyPoses3D->size() - 1)
        {
            ROS_WARN("Requested loop closure from a pose that is not on the map: pose: %i, map size: %i", req.target_pose_index, (int)copy_cloudKeyPoses3D->size());
            res.success = false;
            res.message = "Target pose index out of range";
            return true;
        }
        
        int min_pose = std::max(0, req.target_pose_index - req.window_size);
        int max_pose = std::min((int)copy_cloudKeyPoses3D->size() - 1, req.target_pose_index + req.window_size);
        if(req.window_size == 0)
        {
            min_pose = req.target_pose_index;
            max_pose = req.target_pose_index+1;
        }
        ROS_INFO("Searching poses from %d to %d", min_pose, max_pose);
        
        bool any_loop_closed = false;
        int attempts = 0;
        int successful_detections = 0;
        
        for(int pose_idx=min_pose; pose_idx<max_pose; pose_idx++)
        {
            attempts++;
            int loopKeyCur;
            int loopKeyPre;
            
            ROS_INFO("Attempting loop closure detection for pose %d", pose_idx);
            
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre, pose_idx) == false)
            {
                ROS_INFO("No poses close to pose %i were found", pose_idx);
                continue;
            }
            
            successful_detections++;
            ROS_INFO("Found potential loop closure: current=%d, previous=%d", loopKeyCur, loopKeyPre);
                
            LoopClosureResult closure_result;
            tryLoopClosure(loopKeyCur, loopKeyPre, req.icp_convergence_threshold, closure_result);
            
            if(closure_result.success)
            {
                ROS_INFO("ICP SUCCESS! Adding loop closure between %d and %d", loopKeyCur, loopKeyPre);
                
                mtx.lock();
                loopIndexQueue.push_back(closure_result.indexes);
                loopPoseQueue.push_back(closure_result.pose);
                loopNoiseQueue.push_back(closure_result.noise);
                mtx.unlock();
                
                any_loop_closed = true;
                loopIndexContainer[loopKeyCur] = loopKeyPre;
                
                ROS_INFO("Loop closure found between pose %i and %i. translation: %f %f %f, rotation: %f %f %f", 
                        loopKeyCur, loopKeyPre, 
                        closure_result.pose.translation().x(), closure_result.pose.translation().y(), closure_result.pose.translation().z(), 
                        closure_result.pose.rotation().roll(), closure_result.pose.rotation().pitch(), closure_result.pose.rotation().yaw());
            }
            else
            {
                ROS_WARN("ICP FAILED between pose %i and %i, Continuing with next pair", loopKeyCur, loopKeyPre);
            }            
        }
        
        ROS_INFO("Loop closure attempts: %d, successful detections: %d, successful ICP: %s", 
                attempts, successful_detections, any_loop_closed ? "YES" : "NO");
        
        if(any_loop_closed)
        {
            ROS_INFO("=== UPDATING ISAM WITH LOOP CLOSURES ===");
            
            // Create a clean graph for just the loop closure factors
            NonlinearFactorGraph loopGraph;
            Values loopEstimate; // Should be empty for loop closures
            
            ROS_INFO("Number of loop closures to add: %zu", loopIndexQueue.size());
            
            // Add only the loop closure factors to a clean graph
            for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
            {
                int indexFrom = loopIndexQueue[i].first;
                int indexTo = loopIndexQueue[i].second;
                gtsam::Pose3 poseBetween = loopPoseQueue[i];
                auto noiseBetween = loopNoiseQueue[i];
                loopGraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
                ROS_INFO("Added loop factor: %d -> %d", indexFrom, indexTo);
            }
            
            // Clear the loop queues
            loopIndexQueue.clear();
            loopPoseQueue.clear();
            loopNoiseQueue.clear();
            
            ROS_INFO("Created clean loop graph with %zu factors", loopGraph.size());
            
            // Update ISAM with only the loop closure factors
            isam->update(loopGraph, loopEstimate);
            isam->update();
            
            ROS_INFO("ISAM updated with loop factors");
            
            // Set loop closure flag to trigger additional updates and pose correction
            aLoopIsClosed = true;
            
            // Additional ISAM updates for loop closure convergence
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();

            ROS_INFO("Additional ISAM updates completed");
            
            // Calculate the updated estimates from ISAM (CRITICAL!)
            isamCurrentEstimate = isam->calculateEstimate();
            ROS_INFO("Updated isamCurrentEstimate with %zu poses", isamCurrentEstimate.size());
            
            // Correct poses based on optimized estimates
            correctPoses();
            
            ROS_INFO("Pose correction completed");
            
            // Reset the loop closure flag
            aLoopIsClosed = false;
        }
        else
        {
            ROS_WARN("No loop closures were successfully added!");
        }

        // Implementation completed
        ROS_INFO("Refine map service called with target_pose_index: %d, window_size: %d, icp_convergence_threshold: %f", req.target_pose_index, req.window_size, req.icp_convergence_threshold);
        res.success = any_loop_closed;
        res.message = any_loop_closed ? "Map refinement completed successfully" : "No loop closures found";
        return true;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "liorf");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
