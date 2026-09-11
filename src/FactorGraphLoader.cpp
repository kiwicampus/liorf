#include "liorf/FactorGraphLoader.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <boost/format.hpp>
#include <gtsam/inference/Symbol.h>
#include <cmath>
#include <cstdlib>
#include <sstream>

using namespace gtsam;
using symbol_shorthand::X;

// --- Small parsing helpers for the plain-text factor_graph.yaml format written by
// dumpGraph.cpp's dumpYAML(). Deliberately not using yaml-cpp's generic Node-tree parser here
// (see FactorGraphLoader.h comment on loadYAML) — these just need to handle the exact,
// simple layout that writer produces: "key: value" lines and flow-style "{k: v, ...}" / "[v, ...]".

// Finds "key" in line and parses the number immediately following it (stops at the first
// non-numeric character, e.g. a following comma or brace) via std::stod/std::stoi semantics.
static double extractDoubleAfter(const std::string& line, const std::string& key) {
    size_t p = line.find(key);
    if (p == std::string::npos) return 0.0;
    p += key.size();
    try {
        return std::stod(line.substr(p));
    } catch (...) {
        return 0.0;
    }
}

static long extractIntAfter(const std::string& line, const std::string& key) {
    size_t p = line.find(key);
    if (p == std::string::npos) return 0;
    p += key.size();
    try {
        return std::stol(line.substr(p));
    } catch (...) {
        return 0;
    }
}

// Parses a flow-style sequence like "sigmas: [0.1, 0.2, 0.3]" into its numeric values.
static std::vector<double> extractSigmas(const std::string& line) {
    std::vector<double> values;
    size_t open = line.find('[');
    size_t close = line.find(']', open == std::string::npos ? 0 : open);
    if (open == std::string::npos || close == std::string::npos || close <= open) return values;
    std::string inner = line.substr(open + 1, close - open - 1);
    std::stringstream ss(inner);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            values.push_back(std::stod(token));
        } catch (...) {
            // skip malformed token
        }
    }
    return values;
}

static std::string trimmedContent(const std::string& line) {
    size_t first = line.find_first_not_of(" \t\r");
    return (first == std::string::npos) ? std::string() : line.substr(first);
}

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
    std::ifstream in(yaml_path);
    if (!in.is_open()) {
        std::cerr << "Failed to open YAML file: " << yaml_path << std::endl;
        return false;
    }

    std::cout << "Loading factor graph from: " << yaml_path << std::endl;

    keyframe_data_.clear();
    factor_graph_.resize(0);
    has_gps_datum_ = false;

    enum class Section { NONE, METADATA, VERTICES, FACTORS };
    Section section = Section::NONE;

    bool have_vertex = false;
    int v_id = 0;
    double v_tx = 0, v_ty = 0, v_tz = 0, v_qx = 0, v_qy = 0, v_qz = 0, v_qw = 0;
    bool v_has_ts = false;
    double v_ts = 0;

    bool have_factor = false;
    std::string f_type;
    int f_key = 0, f_key1 = 0, f_key2 = 0;
    double f_x = 0, f_y = 0, f_z = 0, f_qx = 0, f_qy = 0, f_qz = 0, f_qw = 0;
    std::vector<double> f_sigmas;

    auto flushVertex = [&]() {
        if (!have_vertex) return;
        addVertex(v_id, v_tx, v_ty, v_tz, v_qx, v_qy, v_qz, v_qw, v_has_ts, v_ts);
        have_vertex = false;
    };
    auto flushFactor = [&]() {
        if (!have_factor) return;
        if (f_type == "PriorFactor") {
            addPriorFactor(f_key, f_sigmas);
        } else if (f_type == "BetweenFactor") {
            addBetweenFactor(f_key1, f_key2, f_x, f_y, f_z, f_qx, f_qy, f_qz, f_qw, f_sigmas);
        } else if (f_type == "GPSFactor") {
            addGPSFactor(f_key, f_x, f_y, f_z, f_sigmas);
        }
        have_factor = false;
        f_sigmas.clear();
    };

    std::string line;
    while (std::getline(in, line)) {
        std::string content = trimmedContent(line);
        if (content.empty()) continue;

        if (content.rfind("metadata:", 0) == 0) { section = Section::METADATA; continue; }
        if (content.rfind("vertices:", 0) == 0) { flushVertex(); section = Section::VERTICES; continue; }
        if (content.rfind("factors:", 0) == 0) { flushVertex(); flushFactor(); section = Section::FACTORS; continue; }

        if (section == Section::METADATA) {
            if (content.rfind("latitude:", 0) == 0) {
                gps_latitude_ = extractDoubleAfter(content, "latitude:");
                has_gps_datum_ = true;
            } else if (content.rfind("longitude:", 0) == 0) {
                gps_longitude_ = extractDoubleAfter(content, "longitude:");
            } else if (content.rfind("altitude:", 0) == 0) {
                gps_altitude_ = extractDoubleAfter(content, "altitude:");
            }
            continue;
        }

        if (section == Section::VERTICES) {
            if (content.rfind("- id:", 0) == 0) {
                flushVertex();
                have_vertex = true;
                v_id = (int)extractIntAfter(content, "id:");
                v_has_ts = false;
            } else if (content.rfind("translation:", 0) == 0) {
                v_tx = extractDoubleAfter(content, "x:");
                v_ty = extractDoubleAfter(content, "y:");
                v_tz = extractDoubleAfter(content, "z:");
            } else if (content.rfind("rotation:", 0) == 0) {
                v_qx = extractDoubleAfter(content, "x:");
                v_qy = extractDoubleAfter(content, "y:");
                v_qz = extractDoubleAfter(content, "z:");
                v_qw = extractDoubleAfter(content, "w:");
            } else if (content.rfind("timestamp:", 0) == 0) {
                v_ts = extractDoubleAfter(content, "timestamp:");
                v_has_ts = true;
            }
            // "euler:" lines are informational only (not consumed by the loader) — skipped.
            continue;
        }

        if (section == Section::FACTORS) {
            if (content.rfind("- type:", 0) == 0) {
                flushFactor();
                have_factor = true;
                f_type = trimmedContent(content.substr(content.find(':') + 1));
            } else if (content.rfind("key1:", 0) == 0) {
                f_key1 = (int)extractIntAfter(content, "key1:");
            } else if (content.rfind("key2:", 0) == 0) {
                f_key2 = (int)extractIntAfter(content, "key2:");
            } else if (content.rfind("key:", 0) == 0) {
                f_key = (int)extractIntAfter(content, "key:");
            } else if (content.rfind("translation:", 0) == 0) {
                f_x = extractDoubleAfter(content, "x:");
                f_y = extractDoubleAfter(content, "y:");
                f_z = extractDoubleAfter(content, "z:");
            } else if (content.rfind("rotation:", 0) == 0) {
                f_qx = extractDoubleAfter(content, "x:");
                f_qy = extractDoubleAfter(content, "y:");
                f_qz = extractDoubleAfter(content, "z:");
                f_qw = extractDoubleAfter(content, "w:");
            } else if (content.rfind("gps_measurement:", 0) == 0) {
                f_x = extractDoubleAfter(content, "x:");
                f_y = extractDoubleAfter(content, "y:");
                f_z = extractDoubleAfter(content, "z:");
            } else if (content.rfind("sigmas:", 0) == 0) {
                f_sigmas = extractSigmas(content);
            }
            // "prior_pose:"/"measured_pose:"/"noise_model:" lines are just parent headers
            // with no data of their own beyond what's parsed above — skipped.
            continue;
        }
    }
    flushVertex();
    flushFactor();

    if (keyframe_data_.empty()) {
        std::cerr << "Invalid factor_graph.yaml: no vertices parsed" << std::endl;
        return false;
    }

    std::cout << "Loaded " << keyframe_data_.size() << " vertices" << std::endl;
    std::cout << "Loaded " << factor_graph_.size() << " factors" << std::endl;
    if (has_gps_datum_) {
        std::cout << "Loaded GPS datum: " << gps_latitude_ << ", " << gps_longitude_ << ", " << gps_altitude_ << std::endl;
    }
    return true;
}

void FactorGraphLoader::addVertex(int id, double tx, double ty, double tz,
                                   double qx, double qy, double qz, double qw,
                                   bool has_timestamp, double timestamp) {
    gtsam::Rot3 rotation(qw, qx, qy, qz);
    gtsam::Point3 translation(tx, ty, tz);
    gtsam::Pose3 pose(rotation, translation);

    auto keyframe_data = std::make_shared<KeyframeData>(id, pose, has_timestamp ? timestamp : 0.0);
    keyframe_data_[id] = keyframe_data;

    initial_estimate_.insert(id, pose);
}

void FactorGraphLoader::addPriorFactor(int key, const std::vector<double>& sigmas) {
    if (!initial_estimate_.exists(key)) return;
    gtsam::Pose3 pose = initial_estimate_.at<gtsam::Pose3>(key);

    gtsam::Vector6 prior_sigmas;
    if (sigmas.size() == 6) {
        for (size_t i = 0; i < 6; ++i) prior_sigmas(i) = sigmas[i];
    } else {
        prior_sigmas << 0.1, 0.1, M_PI, 10000, 10000, 10000;
    }
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
    factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(key, pose, prior_noise));
}

void FactorGraphLoader::addBetweenFactor(int key1, int key2,
                                          double tx, double ty, double tz,
                                          double qx, double qy, double qz, double qw,
                                          const std::vector<double>& sigmas) {
    gtsam::Rot3 rotation(qw, qx, qy, qz);
    gtsam::Point3 translation(tx, ty, tz);
    gtsam::Pose3 relative_pose(rotation, translation);

    if (std::abs(key1 - key2) > 1) {
        loop_closure_indices_.push_back(std::make_pair(key1, key2));
        loop_closure_poses_.push_back(relative_pose);
    }

    gtsam::Vector6 between_sigmas;
    if (sigmas.size() == 6) {
        for (size_t i = 0; i < 6; ++i) between_sigmas(i) = sigmas[i];
    } else {
        between_sigmas << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01;
    }
    auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(between_sigmas);
    factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, relative_pose, between_noise));
}

void FactorGraphLoader::addGPSFactor(int key, double x, double y, double z, const std::vector<double>& sigmas) {
    gtsam::Point3 gps_point(x, y, z);
    gps_factor_indices_.push_back(std::make_pair(key, gps_point));

    gtsam::Vector3 gps_sigmas;
    if (sigmas.size() == 3) {
        for (size_t i = 0; i < 3; ++i) gps_sigmas(i) = sigmas[i];
    } else {
        gps_sigmas << 1.73205081, 1.73205081, 3.87298335;
    }
    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(gps_sigmas);
    factor_graph_.add(gtsam::GPSFactor(key, gps_point, gps_noise));
}

bool FactorGraphLoader::loadPointClouds() {
    
    std::cout << "Loading point clouds from: " << base_path_ << std::endl;
    int loaded_count = 0;

    // Iterate through keyframe data to load corresponding clouds
    for (auto& keyframe_pair : keyframe_data_) {
        int id = keyframe_pair.first;
        
        // Try to load cloud from the subdirectory structure (e.g., 000000/cloud.pcd, 000001/cloud.pcd)
        std::string cloud_file = getCloudDirectory(id) + "/cloud.pcd";
        
        // Check if file exists
        std::ifstream test_file(cloud_file);
        if (!test_file.good()) {
            std::cout << "No cloud file found for keyframe " << id << " at: " << cloud_file << std::endl;
            continue;
        }
        test_file.close();

        pcl::PointCloud<SessionPointType>::Ptr cloud(new pcl::PointCloud<SessionPointType>);
        if (pcl::io::loadPCDFile<SessionPointType>(cloud_file, *cloud) == -1) {
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

pcl::PointCloud<SessionPointType>::Ptr FactorGraphLoader::generateConcatenatedCloud(double leaf_size) const {
    pcl::PointCloud<SessionPointType>::Ptr concatenated_cloud(new pcl::PointCloud<SessionPointType>);
    
    std::cout << "Generating concatenated cloud with " << keyframe_data_.size() << " keyframes" << std::endl;
    for (const auto& keyframe_pair : keyframe_data_) {
        const auto& keyframe_data = keyframe_pair.second;
        
        if (keyframe_data->cloud->size() > 0) {
            // Transform cloud to global frame using the keyframe pose
            pcl::PointCloud<SessionPointType>::Ptr transformed_cloud(new pcl::PointCloud<SessionPointType>);
            
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
        pcl::PointCloud<SessionPointType>::Ptr filtered_cloud(new pcl::PointCloud<SessionPointType>);
        pcl::VoxelGrid<SessionPointType> voxel_filter;
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

bool FactorGraphLoader::getOptimizedPose(int id, gtsam::Pose3& pose) const {
    if (optimized_estimate_.exists(id)) {
        pose = optimized_estimate_.at<gtsam::Pose3>(id);
        return true;
    }
    return false;
}

bool FactorGraphLoader::getLoadedPose(int id, gtsam::Pose3& pose) const {
    if (initial_estimate_.exists(id)) {
        pose = initial_estimate_.at<gtsam::Pose3>(id);
        return true;
    }
    return false;
}

pcl::PointCloud<SessionPointType>::Ptr FactorGraphLoader::getKeyframeCloud(int id) const {
    auto it = keyframe_data_.find(id);
    if (it != keyframe_data_.end()) {
        return it->second->cloud;
    }
    return nullptr;
}