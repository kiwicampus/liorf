#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "liorf/FactorGraphLoader.h"

class ROSFactorGraphVisualizer {
private:
    ros::NodeHandle nh_;
    ros::Publisher pubPriorPath_;
    ros::Publisher pubBetweenPath_;
    ros::Publisher pubGPSPath_;
    ros::Publisher pubOptimizedPath_;
    ros::Publisher pubConcatenatedCloud_;
    
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    
    // The library loader
    std::unique_ptr<FactorGraphLoader> loader_;
    
    // Path messages for different factor types
    nav_msgs::Path priorPath_;
    nav_msgs::Path betweenPath_;
    nav_msgs::Path gpsPath_;
    nav_msgs::Path optimizedPath_;
    
public:
    ROSFactorGraphVisualizer() {
        // Initialize publishers
        pubPriorPath_ = nh_.advertise<nav_msgs::Path>("liorf/prior_factors", 1);
        pubBetweenPath_ = nh_.advertise<nav_msgs::Path>("liorf/between_factors", 1);
        pubGPSPath_ = nh_.advertise<nav_msgs::Path>("liorf/gps_factors", 1);
        pubOptimizedPath_ = nh_.advertise<nav_msgs::Path>("liorf/optimized_trajectory", 1);
        pubConcatenatedCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("liorf/concatenated_cloud", 1);
        
        // Initialize path messages
        priorPath_.header.frame_id = "map";
        betweenPath_.header.frame_id = "map";
        gpsPath_.header.frame_id = "map";
        optimizedPath_.header.frame_id = "map";
        
        // Initialize loader
        loader_ = std::make_unique<FactorGraphLoader>();
    }
    
    bool loadSession(const std::string& base_path) {
        if (!loader_->loadSession(base_path)) {
            ROS_ERROR("Failed to load session from: %s", base_path.c_str());
            return false;
        }
        
        // Build visualization paths from loaded data
        buildVisualizationPaths();
        
        return true;
    }
    
    void buildVisualizationPaths() {
        // Clear existing paths
        priorPath_.poses.clear();
        betweenPath_.poses.clear();
        gpsPath_.poses.clear();
        optimizedPath_.poses.clear();
        
        // Build paths from loaded data
        const auto& keyframe_data = loader_->getKeyframeData();
        const auto& factor_graph = loader_->getFactorGraph();
        
        // Add all keyframe poses to optimized path
        for (const auto& keyframe_pair : keyframe_data) {
            const auto& keyframe = keyframe_pair.second;
            addPoseToPath(optimizedPath_, keyframe->pose);
        }
        
        // Parse individual factors to build visualization paths
        for (size_t i = 0; i < factor_graph.size(); ++i) {
            const auto& factor = factor_graph[i];
            
            // Check factor type and extract poses
            if (factor->keys().size() > 0) {
                int key = factor->keys()[0];
                
                if (keyframe_data.find(key) != keyframe_data.end()) {
                    const auto& keyframe = keyframe_data.at(key);
                    const auto& pose = keyframe->pose;
        
                    // Add to appropriate path based on factor type
                    if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(factor.get())) {
                        addPoseToPath(priorPath_, pose);
                    } else if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factor.get())) {
                        addPoseToPath(betweenPath_, pose);
                    } else if (dynamic_cast<const gtsam::GPSFactor*>(factor.get())) {
                        addPoseToPath(gpsPath_, pose);
                }
                }
            }
        }
        
        ROS_INFO("Built visualization paths - Prior: %zu, Between: %zu, GPS: %zu, Optimized: %zu", 
                 priorPath_.poses.size(), betweenPath_.poses.size(), 
                 gpsPath_.poses.size(), optimizedPath_.poses.size());
    }
    
    void addPoseToPath(nav_msgs::Path& path, const gtsam::Pose3& pose, 
                   const std::string& frame_id = "map", double timestamp = 0.0) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.header.stamp = timestamp > 0.0 ? ros::Time(timestamp) : ros::Time::now();
        
        // Set position
        pose_stamped.pose.position.x = pose.translation().x();
        pose_stamped.pose.position.y = pose.translation().y();
        pose_stamped.pose.position.z = pose.translation().z();
        
        // Set orientation
        auto quat = pose.rotation().toQuaternion();
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        pose_stamped.pose.orientation.w = quat.w();
        
        path.poses.push_back(pose_stamped);
    }
    
    void publishPaths() {
        ros::Time now = ros::Time::now();
        
        // Update timestamps
        priorPath_.header.stamp = now;
        betweenPath_.header.stamp = now;
        gpsPath_.header.stamp = now;
        optimizedPath_.header.stamp = now;
        
        // Publish all paths
        pubPriorPath_.publish(priorPath_);
        pubBetweenPath_.publish(betweenPath_);
        pubGPSPath_.publish(gpsPath_);
        pubOptimizedPath_.publish(optimizedPath_);
        
        // Publish concatenated cloud (generated on demand with filtering)
        std::cout << "Generating concatenated cloud" << std::endl;
        auto cloud = loader_->generateConcatenatedCloud(0.3); // Use 0.3m leaf size for filtering
        std::cout << "Cloud size: " << cloud->size() << std::endl;
        if (cloud && cloud->size() > 0) {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.frame_id = "map";
            cloud_msg.header.stamp = now;
            pubConcatenatedCloud_.publish(cloud_msg);
        }
        
        ROS_INFO("Published paths - Prior: %zu, Between: %zu, GPS: %zu, Optimized: %zu", 
                 priorPath_.poses.size(), betweenPath_.poses.size(), 
                 gpsPath_.poses.size(), optimizedPath_.poses.size());
    }
    
    void run() {
        ros::Rate rate(1); // 1 Hz
        
        while (ros::ok()) {
            publishPaths();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_liorf_session");
    
    if (argc != 2) {
        ROS_ERROR("Usage: %s <base_path>", argv[0]);
        return 1;
    }
    
    std::string base_path = argv[1];
    
    ROSFactorGraphVisualizer visualizer;
    
    if (!visualizer.loadSession(base_path)) {
        ROS_ERROR("Failed to load session from: %s", base_path.c_str());
        return 1;
    }
    
    ROS_INFO("Session loaded successfully. Publishing visualization data...");
    
    // Run the main loop
    visualizer.run();
    
    return 0;
}
