#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "FactorGraphLoader.h"

// Debug/visualization tool: loads a previously-saved mapping session (factor_graph.yaml +
// per-keyframe clouds) and republishes it for inspection in RViz. Not part of the mapping
// critical path.
class ROSFactorGraphVisualizer : public rclcpp::Node
{
public:
    ROSFactorGraphVisualizer() : Node("load_liorf_session")
    {
        pubPriorPath_ = create_publisher<nav_msgs::msg::Path>("liorf_mapping/prior_factors", 1);
        pubBetweenPath_ = create_publisher<nav_msgs::msg::Path>("liorf_mapping/between_factors", 1);
        pubGPSPath_ = create_publisher<nav_msgs::msg::Path>("liorf_mapping/gps_factors", 1);
        pubOptimizedPath_ = create_publisher<nav_msgs::msg::Path>("liorf_mapping/optimized_trajectory", 1);
        pubConcatenatedCloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("liorf_mapping/concatenated_cloud", 1);

        priorPath_.header.frame_id = "map";
        betweenPath_.header.frame_id = "map";
        gpsPath_.header.frame_id = "map";
        optimizedPath_.header.frame_id = "map";

        loader_ = std::make_unique<FactorGraphLoader>();
    }

    bool loadSession(const std::string& base_path)
    {
        if (!loader_->loadSession(base_path)) {
            RCLCPP_ERROR(get_logger(), "Failed to load session from: %s", base_path.c_str());
            return false;
        }

        buildVisualizationPaths();
        return true;
    }

    void start()
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ROSFactorGraphVisualizer::publishPaths, this));
    }

private:
    void buildVisualizationPaths()
    {
        priorPath_.poses.clear();
        betweenPath_.poses.clear();
        gpsPath_.poses.clear();
        optimizedPath_.poses.clear();

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

            if (factor->keys().size() > 0) {
                int key = factor->keys()[0];

                if (keyframe_data.find(key) != keyframe_data.end()) {
                    const auto& keyframe = keyframe_data.at(key);
                    const auto& pose = keyframe->pose;

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

        RCLCPP_INFO(get_logger(), "Built visualization paths - Prior: %zu, Between: %zu, GPS: %zu, Optimized: %zu",
                 priorPath_.poses.size(), betweenPath_.poses.size(),
                 gpsPath_.poses.size(), optimizedPath_.poses.size());
    }

    void addPoseToPath(nav_msgs::msg::Path& path, const gtsam::Pose3& pose,
                   const std::string& frame_id = "map", double timestamp = 0.0)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.header.stamp = timestamp > 0.0 ? rclcpp::Time(static_cast<int64_t>(timestamp * 1e9)) : this->now();

        pose_stamped.pose.position.x = pose.translation().x();
        pose_stamped.pose.position.y = pose.translation().y();
        pose_stamped.pose.position.z = pose.translation().z();

        auto quat = pose.rotation().toQuaternion();
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        pose_stamped.pose.orientation.w = quat.w();

        path.poses.push_back(pose_stamped);
    }

    void publishPaths()
    {
        rclcpp::Time now = this->now();

        priorPath_.header.stamp = now;
        betweenPath_.header.stamp = now;
        gpsPath_.header.stamp = now;
        optimizedPath_.header.stamp = now;

        pubPriorPath_->publish(priorPath_);
        pubBetweenPath_->publish(betweenPath_);
        pubGPSPath_->publish(gpsPath_);
        pubOptimizedPath_->publish(optimizedPath_);

        // Publish concatenated cloud (generated on demand with filtering)
        RCLCPP_INFO(get_logger(), "Generating concatenated cloud");
        auto cloud = loader_->generateConcatenatedCloud(0.5); // 0.5m leaf size for filtering
        RCLCPP_INFO(get_logger(), "Cloud size: %zu", cloud->size());
        if (cloud && cloud->size() > 0) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.frame_id = "map";
            cloud_msg.header.stamp = now;
            pubConcatenatedCloud_->publish(cloud_msg);
        }

        RCLCPP_INFO(get_logger(), "Published paths - Prior: %zu, Between: %zu, GPS: %zu, Optimized: %zu",
                 priorPath_.poses.size(), betweenPath_.poses.size(),
                 gpsPath_.poses.size(), optimizedPath_.poses.size());
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPriorPath_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubBetweenPath_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGPSPath_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOptimizedPath_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubConcatenatedCloud_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<FactorGraphLoader> loader_;

    nav_msgs::msg::Path priorPath_;
    nav_msgs::msg::Path betweenPath_;
    nav_msgs::msg::Path gpsPath_;
    nav_msgs::msg::Path optimizedPath_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("load_liorf_session"), "Usage: %s <base_path>", argv[0]);
        rclcpp::shutdown();
        return 1;
    }

    std::string base_path = argv[1];

    auto visualizer = std::make_shared<ROSFactorGraphVisualizer>();

    if (!visualizer->loadSession(base_path)) {
        RCLCPP_ERROR(visualizer->get_logger(), "Failed to load session from: %s", base_path.c_str());
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(visualizer->get_logger(), "Session loaded successfully. Publishing visualization data...");

    visualizer->start();
    rclcpp::spin(visualizer);

    rclcpp::shutdown();
    return 0;
}
