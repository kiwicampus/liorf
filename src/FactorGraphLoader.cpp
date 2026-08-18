#include "FactorGraphLoader.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <boost/format.hpp>
#include <gtsam/inference/Symbol.h>

// includes for segmentation output 
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/format.hpp> 


#include <pcl/common/transforms.h> // Necesario para pcl::transformPointCloud
using namespace gtsam;
using symbol_shorthand::X;

FactorGraphLoader::FactorGraphLoader() 
    : base_path_("")
    , has_gps_datum_(false)
    , gps_latitude_(0.0)
    , gps_longitude_(0.0)
    , gps_altitude_(0.0)
    , is_loaded_(false)
    , is_optimized_(false) {
    
    // Initialize ISAM2
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = std::make_unique<ISAM2>(parameters);
    
    // Initialize visualization cache vectors
    loop_closure_indices_.clear();
    loop_closure_poses_.clear();
    gps_factor_indices_.clear();
}

FactorGraphLoader::~FactorGraphLoader() = default;

bool FactorGraphLoader::loadSession(const std::string& base_path, bool optimize) {
    base_path_ = base_path;
    
    // Reset state
    is_loaded_ = false;
    is_optimized_ = false;
    factor_graph_.resize(0);
    initial_estimate_.clear();
    optimized_estimate_.clear();
    keyframe_data_.clear();
    
    // Clear visualization cache
    loop_closure_indices_.clear();
    loop_closure_poses_.clear();
    gps_factor_indices_.clear();
    
    // Load YAML file
    std::string yaml_path = getYAMLPath();
    if (!loadYAML(yaml_path)) {
        std::cerr << "Failed to load YAML file: " << yaml_path << std::endl;
        return false;
    }
    
    // Load point clouds
    if (!loadPointClouds()) {
    std::cerr << "Failed to load point clouds" << std::endl;
    return false;
    }
    
    
    is_loaded_ = true;
    std::cout << "Session loaded successfully from: " << base_path << std::endl;
    std::cout << "Keyframes: " << getNumKeyframes() << ", Factors: " << getNumFactors() << std::endl;

    if (optimize) {
        std::cout << "Optimizing factor graph..." << std::endl;
        optimizeGraph();
    }

    return true;
}

bool FactorGraphLoader::loadYAML(const std::string& yaml_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        
        if (!config["metadata"] || !config["vertices"] || !config["factors"]) {
            std::cerr << "Invalid YAML format: missing required sections" << std::endl;
            return false;
        }
        
        std::cout << "Loading factor graph from: " << yaml_path << std::endl;
        std::cout << "Total keyframes: " << config["metadata"]["total_keyframes"].as<int>() << std::endl;
        std::cout << "Total factors: " << config["metadata"]["total_factors"].as<int>() << std::endl;
        
        // Load GPS datum if available
        if (config["metadata"]["gps_datum"]) {
            has_gps_datum_ = true;
            gps_latitude_ = config["metadata"]["gps_datum"]["latitude"].as<double>();
            gps_longitude_ = config["metadata"]["gps_datum"]["longitude"].as<double>();
            gps_altitude_ = config["metadata"]["gps_datum"]["altitude"].as<double>();
            std::cout << "Loaded GPS datum: " << gps_latitude_ << ", " << gps_longitude_ << ", " << gps_altitude_ << std::endl;
        }
        
        // Load vertices (keyframe poses)
        loadVertices(config["vertices"]);
        
        // Load factors
        loadFactors(config["factors"]);
        
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading YAML: " << e.what() << std::endl;
        return false;
    }
}

void FactorGraphLoader::loadVertices(const YAML::Node& vertices_node) {
    keyframe_data_.clear();
    
    for (const auto& vertex : vertices_node) {
        int id = vertex["id"].as<int>();
        
        // Extract translation
        double x = vertex["translation"]["x"].as<double>();
        double y = vertex["translation"]["y"].as<double>();
        double z = vertex["translation"]["z"].as<double>();
        
        // Extract rotation (quaternion)
        double qx = vertex["rotation"]["x"].as<double>();
        double qy = vertex["rotation"]["y"].as<double>();
        double qz = vertex["rotation"]["z"].as<double>();
        double qw = vertex["rotation"]["w"].as<double>();
        
        // Create GTSAM pose
        gtsam::Rot3 rotation(qw, qx, qy, qz);
        gtsam::Point3 translation(x, y, z);
        gtsam::Pose3 pose(rotation, translation);
        
        // Create keyframe data structure
        double timestamp = 0.0;
        if (vertex["timestamp"]) {
            timestamp = vertex["timestamp"].as<double>();
        }
        
        auto keyframe_data = std::make_shared<KeyframeData>(id, pose, timestamp);
        keyframe_data_[id] = keyframe_data;
        
        // Add to initial estimate
        initial_estimate_.insert(id, pose);
    }
    
    std::cout << "Loaded " << keyframe_data_.size() << " vertices" << std::endl;
}

void FactorGraphLoader::loadFactors(const YAML::Node& factors_node) {
    factor_graph_.resize(0);
    
    for (const auto& factor : factors_node) {
        std::string type = factor["type"].as<std::string>();
        
        if (type == "PriorFactor") {
            loadPriorFactor(factor);
        } else if (type == "BetweenFactor") {
            loadBetweenFactor(factor);
        } else if (type == "GPSFactor") {
            loadGPSFactor(factor);
        }
    }
    
    std::cout << "Loaded " << factor_graph_.size() << " factors" << std::endl;
}

void FactorGraphLoader::loadPriorFactor(const YAML::Node& factor) {
    int key = factor["key"].as<int>();
    
    // Get pose from initial estimate
    if (initial_estimate_.exists(key)) {
        gtsam::Pose3 pose = initial_estimate_.at<gtsam::Pose3>(key);
        
        // Create noise model from YAML data
        gtsam::Vector6 prior_sigmas;
        if (factor["noise_model"] && factor["noise_model"]["sigmas"]) {
            auto sigmas = factor["noise_model"]["sigmas"];
            if (sigmas.size() == 6) {
                for (size_t i = 0; i < 6; ++i) {
                    prior_sigmas(i) = sigmas[i].as<double>();
                }
            } else {
                std::cout << "Invalid sigma count for PriorFactor, using default values" << std::endl;
                prior_sigmas << 0.1, 0.1, M_PI, 10000, 10000, 10000;
            }
        } else {
            std::cout << "No noise model found for PriorFactor, using default values" << std::endl;
            prior_sigmas << 0.1, 0.1, M_PI, 10000, 10000, 10000;
        }
        
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = 
            gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
        
        // Add factor
        factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(key, pose, prior_noise));
    }
}

void FactorGraphLoader::loadBetweenFactor(const YAML::Node& factor) {
    int key1 = factor["key1"].as<int>();
    int key2 = factor["key2"].as<int>();
    
    // Extract measured relative pose
    auto measured = factor["measured_pose"];
    double x = measured["translation"]["x"].as<double>();
    double y = measured["translation"]["y"].as<double>();
    double z = measured["translation"]["z"].as<double>();
    
    // Extract rotation (quaternion)
    double qx = measured["rotation"]["x"].as<double>();
    double qy = measured["rotation"]["y"].as<double>();
    double qz = measured["rotation"]["z"].as<double>();
    double qw = measured["rotation"]["w"].as<double>();
    
    gtsam::Rot3 rotation(qw, qx, qy, qz);
    gtsam::Point3 translation(x, y, z);
    gtsam::Pose3 relative_pose(rotation, translation);
    
    // Check if this is a loop closure (non-consecutive poses)
    if (abs(key1 - key2) > 1) {
        std::cout << "Found loop closure between poses " << key1 << " and " << key2 << std::endl;
        
        // Cache loop closure for visualization
        loop_closure_indices_.push_back(std::make_pair(key1, key2));
        loop_closure_poses_.push_back(relative_pose);
    }
    
    // Create noise model from YAML data
    gtsam::Vector6 between_sigmas;
    if (factor["noise_model"] && factor["noise_model"]["sigmas"]) {
        auto sigmas = factor["noise_model"]["sigmas"];
        if (sigmas.size() == 6) {
            for (size_t i = 0; i < 6; ++i) {
                between_sigmas(i) = sigmas[i].as<double>();
            }
        } else {
            std::cout << "Invalid sigma count for BetweenFactor, using default values" << std::endl;
            between_sigmas << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01;
        }
    } else {
        std::cout << "No noise model found for BetweenFactor, using default values" << std::endl;
        between_sigmas << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01;
    }
    
    gtsam::noiseModel::Diagonal::shared_ptr between_noise = 
        gtsam::noiseModel::Diagonal::Sigmas(between_sigmas);
    
    // Add factor
    factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, relative_pose, between_noise));
}

void FactorGraphLoader::loadGPSFactor(const YAML::Node& factor) {
    int key = factor["key"].as<int>();
    
    // Extract GPS measurement
    auto gps_measurement = factor["gps_measurement"];
    double x = gps_measurement["x"].as<double>();
    double y = gps_measurement["y"].as<double>();
    double z = gps_measurement["z"].as<double>();
    
    gtsam::Point3 gps_point(x, y, z);
    
    // Cache GPS factor for visualization
    gps_factor_indices_.push_back(std::make_pair(key, gps_point));
    
    // Create noise model from YAML data
    gtsam::Vector3 gps_sigmas;
    if (factor["noise_model"] && factor["noise_model"]["sigmas"]) {
        auto sigmas = factor["noise_model"]["sigmas"];
        if (sigmas.size() == 3) {
            for (size_t i = 0; i < 3; ++i) {
                gps_sigmas(i) = sigmas[i].as<double>();
            }
        } else {
            std::cout << "Invalid sigma count for GPSFactor, using default values" << std::endl;
            gps_sigmas << 1.73205081, 1.73205081, 3.87298335;
        }
    } else {
        std::cout << "No noise model found for GPSFactor, using default values" << std::endl;
        gps_sigmas << 1.73205081, 1.73205081, 3.87298335;
    }
    
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise = 
        gtsam::noiseModel::Diagonal::Sigmas(gps_sigmas);
    
    // Add factor
    factor_graph_.add(gtsam::GPSFactor(key, gps_point, gps_noise));
}

bool FactorGraphLoader::loadPointClouds() {
    
    std::cout << "Loading point clouds from: " << base_path_ << std::endl;
    int loaded_count = 0;
    // Iterate through keyframe data to load corresponding clouds
    for (auto& keyframe_pair : keyframe_data_) {
        int id = keyframe_pair.first;
        auto& keyframe_data = keyframe_pair.second;
        
        // Try to load cloud from the subdirectory structure (e.g., 000000/cloud.pcd, 000001/cloud.pcd)
        std::string cloud_file = getCloudDirectory(id) + "/cloud.pcd";
        
        // Check if file exists
        std::ifstream test_file(cloud_file);
        if (!test_file.good()) {
            std::cout << "No cloud file found for keyframe " << id << " at: " << cloud_file << std::endl;
            continue;
        }
        test_file.close();

        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        // Load point cloud directly into the keyframe data structure
        if (pcl::io::loadPCDFile<PointType>(cloud_file, *cloud) == -1) {
            std::cout << "Failed to load cloud from: " << cloud_file << std::endl;
            continue;
        }

        keyframe_pair.second->cloud = cloud;
        loaded_count++;
    }
    if (loaded_count == 0) {
        std::cerr << "Failed to load any point clouds." << std::endl;
        return false;
    }
    std::cout << "Successfully loaded " << loaded_count << " point clouds." << std::endl;
    return true;
}

bool FactorGraphLoader::optimizeGraph() {
    if (!is_loaded_ || factor_graph_.size() == 0) {
        std::cerr << "No factors to optimize" << std::endl;
        return false;
    }
    
    std::cout << "Optimizing factor graph with " << factor_graph_.size() << " factors and " << initial_estimate_.size() << " variables" << std::endl;
    
    try {
        // Update ISAM with the factor graph
        isam_->update(factor_graph_, initial_estimate_);
        isam_->update();
        
        // Get optimized estimate
        optimized_estimate_ = isam_->calculateEstimate();
        
        std::cout << "Optimization completed successfully" << std::endl;
        is_optimized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Optimization failed: " << e.what() << std::endl;
        return false;
    }
}

std::string FactorGraphLoader::getYAMLPath() const {
    return base_path_ + "/factor_graph.yaml";
}

std::string FactorGraphLoader::getCloudDirectory(int keyframe_id) const {
    return base_path_ + "/" + (boost::format("%06d") % keyframe_id).str();
}

pcl::PointCloud<PointType>::Ptr FactorGraphLoader::generateConcatenatedCloud(double leaf_size) const {
    pcl::PointCloud<PointType>::Ptr concatenated_cloud(new pcl::PointCloud<PointType>);
    
    std::cout << "Generating concatenated cloud with " << keyframe_data_.size() << " keyframes" << std::endl;
    for (const auto& keyframe_pair : keyframe_data_) {
        const auto& keyframe_data = keyframe_pair.second;
        
        if (keyframe_data->cloud->size() > 0) {
            // Transform cloud to global frame using the keyframe pose
            pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
            
            // Convert GTSAM pose to Eigen transformation matrix
            Eigen::Matrix4d transform_matrix = keyframe_data->pose.matrix();
            Eigen::Matrix4f transform_matrix_float = transform_matrix.cast<float>();

            // Transform the cloud
            pcl::transformPointCloud(*keyframe_data->cloud, *transformed_cloud, transform_matrix_float);
            
            // Concatenate transformed cloud
            *concatenated_cloud += *transformed_cloud;

        }
    }
    
    // Apply voxel grid filter to reduce point density
    if (leaf_size > 0.0 && concatenated_cloud->size() > 0) {
        pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(concatenated_cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*filtered_cloud);
        
        std::cout << "Filtered concatenated cloud from " << concatenated_cloud->size() 
                  << " to " << filtered_cloud->size() << " points (leaf size: " << leaf_size << "m)" << std::endl;
        
        return filtered_cloud;
    }
        
    return concatenated_cloud;
} 

std::vector<int> FactorGraphLoader::getKeyframeIDs() const {
    std::vector<int> ids;
    ids.reserve(keyframe_data_.size());
    for (const auto& pair : keyframe_data_) {
        ids.push_back(pair.first);
    }
    return ids;
}

bool FactorGraphLoader::getLoadedPose(int id, gtsam::Pose3& pose) const {
    if (initial_estimate_.exists(id)) {
        pose = initial_estimate_.at<gtsam::Pose3>(id);
        return true;
    }
    return false;
}

bool FactorGraphLoader::getOptimizedPose(int id, gtsam::Pose3& pose) const {
    if (optimized_estimate_.exists(id)) {
        pose = optimized_estimate_.at<gtsam::Pose3>(id);
        return true;
    }
    return false;
}

pcl::PointCloud<PointType>::Ptr FactorGraphLoader::getKeyframeCloud(int id) const {
    auto it = keyframe_data_.find(id);
    if (it != keyframe_data_.end()) {
        return it->second->cloud;
    }
    return nullptr;
}


