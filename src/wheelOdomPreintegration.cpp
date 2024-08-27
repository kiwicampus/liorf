#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion()
    {
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("liorf/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("liorf/imu/path", 1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // static tf
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class WheelOdometryPreintegration : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subWheelOdometry;
    ros::Publisher pubOdometry;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;

    gtsam::Pose3 initialPose_;
    bool initialPoseSet = false;

    // Wheel to lidar: 0.0, 0.0, 0.6 (x, y, z) + 0., 0.12467473, 0., 0.99219767 (q.x, q.y, q.z, q.w)
    gtsam::Pose3 wheel2Lidar = gtsam::Pose3(gtsam::Rot3::Quaternion(0.99219767, 0.0, 0.12467473, 0.0), gtsam::Point3(0.16, 0.0, 0.6));
    // Lidar to Wheel:[ 0.14844238  0.         -0.58134745] (x,y,z) + [ 0.         -0.12467473  0.          0.99219767] (q.x, q.y, q.z, q.w)
    gtsam::Pose3 lidar2Wheel = gtsam::Pose3(gtsam::Rot3::Quaternion(0.99219767, 0.0, -0.12467473, 0.0), gtsam::Point3(-0.007, 0.000, -0.621));

    WheelOdometryPreintegration()
    {
        ros::NodeHandle nh;

        subWheelOdometry = nh.subscribe<nav_msgs::Odometry>(
            "/wheel_odometry/global_odometry", 10,
            &WheelOdometryPreintegration::wheelOdometryHandler, this);

        pubOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 10);
    }

    void wheelOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double newX = odomMsg->pose.pose.position.x;
        double newY = odomMsg->pose.pose.position.y;

        tf::Quaternion quat(
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z,
            odomMsg->pose.pose.orientation.w);
        double roll, pitch, newTheta;
        tf::Matrix3x3(quat).getRPY(roll, pitch, newTheta);

        gtsam::Pose3 newPose(gtsam::Rot3::Rz(newTheta), gtsam::Point3(newX, newY, prevPose_.z()));

        if (!initialPoseSet) {
            initialPose_ = newPose;
            initialPoseSet = true;
        }

        gtsam::Pose3 relativePose = initialPose_.inverse().compose(newPose);

        prevPose_ = relativePose;
        prevVel_ = gtsam::Vector3(odomMsg->twist.twist.linear.x, odomMsg->twist.twist.linear.y, 0);
        prevState_ = gtsam::NavState(relativePose, prevVel_);

        publishTransformedOdometry(odomMsg->header.stamp);
    }

    void publishTransformedOdometry(const ros::Time& timestamp)
    {
        gtsam::Pose3 pose = prevState_.pose();
        gtsam::Pose3 lidarPose = pose.compose(wheel2Lidar);

        gtsam::Vector3 velocity = prevState_.velocity();
        nav_msgs::Odometry odometry;
        odometry.header.stamp = timestamp;
        odometry.header.frame_id = mapFrame;
        odometry.child_frame_id = "odom_imu";

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        auto quat = lidarPose.rotation().toQuaternion();
        odometry.pose.pose.orientation.x = quat.x();
        odometry.pose.pose.orientation.y = quat.y();
        odometry.pose.pose.orientation.z = quat.z();
        odometry.pose.pose.orientation.w = quat.w();
        
        // Uncomment if velocity is needed
        // odometry.twist.twist.linear.x = velocity.x();
        // odometry.twist.twist.linear.y = velocity.y();
        // odometry.twist.twist.linear.z = velocity.z();

        pubOdometry.publish(odometry);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odometry_preintegration");

    WheelOdometryPreintegration woPreintegration;
    TransformFusion TF;

    ROS_INFO("\033[1;32m----> Wheel Odometry Preintegration Started.\033[0m");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
