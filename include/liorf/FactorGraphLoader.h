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

class FactorGraphLoader {
private:
    std::string base_path_;
    
    // GTSAM objects
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph factor_graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values optimized_estimate_;
    
    // Data storage
    std::map<int, gtsam::Pose3> keyframe_poses_;
    std::vector<double> keyframe_stamps_;
    pcl::PointCloud<PointType>::Ptr concatenated_cloud_;
    
    // GPS datum
    bool has_gps_datum_;
    double gps_latitude_;
    double gps_longitude_;
    double gps_altitude_;
    
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
    const std::map<int, gtsam::Pose3>& getKeyframePoses() const { return keyframe_poses_; }
    const std::vector<double>& getKeyframeStamps() const { return keyframe_stamps_; }
    const pcl::PointCloud<PointType>::Ptr& getConcatenatedCloud() const { return concatenated_cloud_; }
    
    // GPS datum access
    bool hasGPSDatum() const { return has_gps_datum_; }
    double getGPSLatitude() const { return gps_latitude_; }
    double getGPSLongitude() const { return gps_longitude_; }
    double getGPSAltitude() const { return gps_altitude_; }
    
    // State queries
    bool isLoaded() const { return is_loaded_; }
    bool isOptimized() const { return is_optimized_; }
    
    // Optimization
    bool optimizeGraph();
    
    // Utility functions
    size_t getNumKeyframes() const { return keyframe_poses_.size(); }
    size_t getNumFactors() const { return factor_graph_.size(); }

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