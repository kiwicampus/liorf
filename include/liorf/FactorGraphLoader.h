#ifndef FACTOR_GRAPH_LOADER_H
#define FACTOR_GRAPH_LOADER_H

#include <string>
#include <vector>
#include <map>
#include <memory>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// GTSAM includes
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>

// YAML includes
#include <yaml-cpp/yaml.h>

// Define point types
using PointType = pcl::PointXYZI;

// Structure to hold keyframe data
struct KeyframeData {
    int id;
    gtsam::Pose3 pose;
    double timestamp;
    pcl::PointCloud<PointType>::Ptr cloud;
    
    KeyframeData(int id_, const gtsam::Pose3& pose_, double timestamp_)
        : id(id_), pose(pose_), timestamp(timestamp_), cloud(new pcl::PointCloud<PointType>()) {}
};

class FactorGraphLoader {
private:
    std::string base_path_;
    
    // GTSAM objects
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph factor_graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values optimized_estimate_;
    
    // Data storage - now using the struct
    std::map<int, std::shared_ptr<KeyframeData>> keyframe_data_;
    
    // GPS datum
    bool has_gps_datum_;
    double gps_latitude_;
    double gps_longitude_;
    double gps_altitude_;
    
    // Cached loop closures and GPS factors for visualization
    std::vector<std::pair<int, int>> loop_closure_indices_;
    std::vector<gtsam::Pose3> loop_closure_poses_;
    std::vector<std::pair<int, gtsam::Point3>> gps_factor_indices_;
    
    // Loading state
    bool is_loaded_;
    bool is_optimized_;

public:
    FactorGraphLoader();
    ~FactorGraphLoader();
    
    // Main loading function
    bool loadSession(const std::string& base_path);
    
    // Data access functions
    const gtsam::NonlinearFactorGraph& getFactorGraph() const { return factor_graph_; }
    const gtsam::Values& getInitialEstimate() const { return initial_estimate_; }
    const gtsam::Values& getOptimizedEstimate() const { return optimized_estimate_; }
    const gtsam::ISAM2* getISAM() const { return isam_.get(); }
    
    // Keyframe data access - returns reference to avoid copying
    const std::map<int, std::shared_ptr<KeyframeData>>& getKeyframeData() const { return keyframe_data_; }
    
    // Generate concatenated cloud on demand (no storage waste)
    pcl::PointCloud<PointType>::Ptr generateConcatenatedCloud(double leaf_size = 0.3) const;
    
    // GPS datum access
    bool hasGPSDatum() const { return has_gps_datum_; }
    double getGPSLatitude() const { return gps_latitude_; }
    double getGPSLongitude() const { return gps_longitude_; }
    double getGPSAltitude() const { return gps_altitude_; }
    
    // Loop closure and GPS factor access for visualization
    const std::vector<std::pair<int, int>>& getLoopClosureIndices() const { return loop_closure_indices_; }
    const std::vector<gtsam::Pose3>& getLoopClosurePoses() const { return loop_closure_poses_; }
    const std::vector<std::pair<int, gtsam::Point3>>& getGPSFactorIndices() const { return gps_factor_indices_; }
    
    // State queries
    bool isLoaded() const { return is_loaded_; }
    bool isOptimized() const { return is_optimized_; }
    
    // Optimization
    bool optimizeGraph();
    
    // Utility functions
    size_t getNumKeyframes() const { return keyframe_data_.size(); }
    size_t getNumFactors() const { return factor_graph_.size(); }

    // Get all keyframe IDs
    std::vector<int> getKeyframeIDs() const;

    // Get optimized pose for a given keyframe
    bool getOptimizedPose(int id, gtsam::Pose3& pose) const;

    // Get cloud for a given keyframe
    pcl::PointCloud<PointType>::Ptr getKeyframeCloud(int id) const;

private:
    // Internal loading functions
    bool loadYAML(const std::string& yaml_path);
    void loadVertices(const YAML::Node& vertices_node);
    void loadFactors(const YAML::Node& factors_node);
    bool loadPointClouds();
    
    // Factor loading helpers
    void loadPriorFactor(const YAML::Node& factor);
    void loadBetweenFactor(const YAML::Node& factor);
    void loadGPSFactor(const YAML::Node& factor);
    
    // Helper functions
    std::string getYAMLPath() const;
    std::string getCloudDirectory(int keyframe_id) const;
};

#endif // FACTOR_GRAPH_LOADER_H 