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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfMap2Odom;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfOdom2BaseLink;
    tf2::Stamped<tf2::Transform> lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;

    TransformFusion(const rclcpp::NodeOptions & options) : ParamServer("liorf_transformFusion", options)
    {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        tfMap2Odom = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tfOdom2BaseLink = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        if (lidarFrame != baselinkFrame)
        {
            try
            {
                tf2::fromMsg(tfBuffer->lookupTransform(
                    lidarFrame, baselinkFrame, rclcpp::Time(0)), lidar2Baselink);
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR(get_logger(), "%s", ex.what());
            }
        }

        subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>("liorf_mapping/mapping/odometry", QosPolicy(history_policy, reliability_policy),
                    std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1));

        subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(odomTopic+"_incremental", QosPolicy(history_policy, reliability_policy),
                    std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1));

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, QosPolicy(history_policy, reliability_policy));
        pubImuPath = create_publisher<nav_msgs::msg::Path>("liorf_mapping/imu/path", QosPolicy(history_policy, reliability_policy));
    }

    Eigen::Affine3f odom2affine(nav_msgs::msg::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = ROS_TIME(odomMsg->header.stamp);
    }

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        // static tf
        tf2::Quaternion quat_tf;
        rclcpp::Time t(static_cast<uint32_t>(lidarOdomTime * 1e9));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(t);

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (ROS_TIME(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
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
        nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        quat_tf.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometry.pose.pose.orientation = quat_msg;
        pubImuOdometry->publish(laserOdometry);

        // publish tf
        tf2::Transform tCur(tf2::Quaternion(laserOdometry.pose.pose.orientation.x, laserOdometry.pose.pose.orientation.y, laserOdometry.pose.pose.orientation.z, laserOdometry.pose.pose.orientation.w),
                                tf2::Vector3(laserOdometry.pose.pose.position.x, laserOdometry.pose.pose.position.y, laserOdometry.pose.pose.position.z));
        if (lidarFrame != baselinkFrame)
            tCur *= lidar2Baselink;

        tf2::Stamped<tf2::Transform> temp_odom_to_base(tCur, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_base_link;
        tf2::convert(temp_odom_to_base, trans_odom_to_base_link);
        trans_odom_to_base_link.child_frame_id = baselinkFrame;
        tfOdom2BaseLink->sendTransform(trans_odom_to_base_link);

        // publish IMU path
        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double imuTime = ROS_TIME(imuOdomQueue.back().header.stamp);
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() && ROS_TIME(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath->publish(imuPath);
            }
        }
    }
};

class WheelOdometryPreintegration : public ParamServer
{
public:
    std::mutex mtx;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subWheelOdometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;

    // First wheel odometry pose received; subsequent poses are expressed relative to it so
    // the mapping pipeline sees odometry starting near the origin, regardless of the wheel
    // odometry source's own absolute frame.
    gtsam::Pose3 initialPose_;
    bool initialPoseSet = false;

    // Wheel to lidar: 0.0, 0.0, 0.6 (x, y, z) + 0., 0.12467473, 0., 0.99219767 (q.x, q.y, q.z, q.w)
    gtsam::Pose3 wheel2Lidar = gtsam::Pose3(gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
    // Lidar to Wheel:[ 0.14844238  0.         -0.58134745] (x,y,z) + [ 0.         -0.12467473  0.          0.99219767] (q.x, q.y, q.z, q.w)
    gtsam::Pose3 lidar2Wheel = gtsam::Pose3(gtsam::Rot3::Quaternion(0.99219767, 0.0, -0.12467473, 0.0), gtsam::Point3(-0.007, 0.000, -0.621));

    WheelOdometryPreintegration(const rclcpp::NodeOptions& options) :
            ParamServer("wheel_odometry_preintegration", options)
    {
        subWheelOdometry = create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odometry/global_odometry", 10,
            std::bind(&WheelOdometryPreintegration::wheelOdometryHandler, this, std::placeholders::_1));

        pubOdometry = create_publisher<nav_msgs::msg::Odometry>(
            odomTopic+"_incremental", 10);
    }

    void wheelOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double newX = odomMsg->pose.pose.position.x;
        double newY = odomMsg->pose.pose.position.y;

        tf2::Quaternion quat(
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z,
            odomMsg->pose.pose.orientation.w);
        double roll, pitch, newTheta;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, newTheta);

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

    void publishTransformedOdometry(const rclcpp::Time& timestamp)
    {
        gtsam::Pose3 pose = prevState_.pose();
        gtsam::Pose3 lidarPose = pose.compose(wheel2Lidar);

        nav_msgs::msg::Odometry odometry;
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

        pubOdometry->publish(odometry);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;

    auto WoP = std::make_shared<WheelOdometryPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);
    e.add_node(WoP);
    e.add_node(TF);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Wheel Odometry Preintegration Started.\033[0m");

    e.spin();

    rclcpp::shutdown();
    return 0;
}
