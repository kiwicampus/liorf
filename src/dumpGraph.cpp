#include "dumpGraph.h"

#include <ros/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <yaml-cpp/yaml.h>
#include <GeographicLib/LocalCartesian.hpp>

void dump(const std::string& dump_directory,
  const gtsam::ISAM2& isam,
  const gtsam::Values& isam_current_estimate,
  const std::vector<double>& keyframe_stamps,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes
) {
  boost::filesystem::create_directories(dump_directory);

  std::vector<gtsam::Pose3> keyframe_poses(isam_current_estimate.size());

  std::ofstream graph_ofs(dump_directory + "/graph.g2o");
  for(const auto& vertex : isam_current_estimate) {
    Eigen::Matrix4d pose = vertex.value.cast<gtsam::Pose3>().matrix();
    keyframe_poses[vertex.key] = vertex.value.cast<gtsam::Pose3>();
    Eigen::Vector3d t = pose.block<3, 1>(0, 3) + Eigen::Vector3d::Random();
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    graph_ofs << "VERTEX_SE3:QUAT " << vertex.key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  graph_ofs << "FIX 0" << "\n";

  std::cout << "factors:" << isam.getFactorsUnsafe().size() << std::endl;
  for(const auto& factor_: isam.getFactorsUnsafe()) {
    factor_->print("factor: ");
    auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor_);
    if(between_factor) {
      Eigen::Matrix4d relative;
      Eigen::Vector3d t;
      Eigen::Quaterniond q;
      Eigen::VectorXd vars;
      Eigen::MatrixXd inf;
      try
      {
        relative = between_factor->measured().matrix();
        t = relative.block<3, 1>(0, 3);
        q = relative.block<3, 3>(0, 0);
        vars = 1.0 / between_factor->noiseModel()->sigmas().array();
        inf = vars.asDiagonal();
      }
      catch(const char* e)
      {
        std::cerr << e << std::endl;
        continue;
      }

      graph_ofs << "EDGE_SE3:QUAT " << between_factor->key1() << " " << between_factor->key2();
      graph_ofs << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

      for(int i = 0; i < inf.rows(); i++) {
        for(int j=i; j<inf.cols(); j++) {
          graph_ofs << " " << inf(i, j);
        }
      }
      graph_ofs << "\n";
      continue;
    }

    // GPS unary prior. interactive_slam / hdl_graph_slam register this as EDGE_SE3_PRIORXYZ.
    auto gps_factor = boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor_);
    if(gps_factor && gps_factor->noiseModel()) {
      gtsam::Point3 gps_measurement = gps_factor->measurementIn();
      // g2o information = cov^{-1} = 1/sigma^2 (3x3 upper triangle)
      Eigen::VectorXd inf_diag = 1.0 / gps_factor->noiseModel()->sigmas().array().square();
      Eigen::MatrixXd inf = inf_diag.asDiagonal();

      graph_ofs << "EDGE_SE3_PRIORXYZ " << gps_factor->key();
      graph_ofs << " " << gps_measurement.x() << " " << gps_measurement.y() << " " << gps_measurement.z();
      for(int i = 0; i < inf.rows(); i++) {
        for(int j = i; j < inf.cols(); j++) {
          graph_ofs << " " << inf(i, j);
        }
      }
      graph_ofs << "\n";
    }
  }
  
  std::cout << "saving clouds" << std::endl;
  for(int i = 0; i < surf_cloud_keyframes.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % dump_directory % i).str();
    boost::filesystem::create_directories(keyframe_directory);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // *cloud += *surf_cloud_keyframes[i];

    // Eigen::Isometry3f camera2lidar = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY()) * Eigen::Isometry3f::Identity();

    // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::transformPointCloud(*cloud, *transformed, camera2lidar);

    // cloud = transformPointCloud(cloud, keyframe_poses[i]);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *surf_cloud_keyframes[i]);

    ros::Time stamp(keyframe_stamps[i]);

    std::ofstream data_ofs(keyframe_directory + "/data");
    data_ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";
    data_ofs << "estimate\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "odom\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "accum_distance -1" << "\n";
    data_ofs << "id " << i << "\n";
  }
}

void dumpYAML(const std::string& dump_directory,
  const gtsam::ISAM2& isam,
  const gtsam::Values& isam_current_estimate,
  const std::vector<double>& keyframe_stamps,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes,
  const GeographicLib::LocalCartesian* gps_trans
) {
  boost::filesystem::create_directories(dump_directory);
  
  std::string yaml_file = dump_directory + "/factor_graph.yaml";
  
  YAML::Node graph;
  
  // Add metadata
  graph["metadata"]["total_keyframes"] = isam_current_estimate.size();
  graph["metadata"]["total_factors"] = isam.getFactorsUnsafe().size();
  graph["metadata"]["dump_timestamp"] = ros::Time::now().toSec();
  
      // Add GPS datum if available
    if (gps_trans != nullptr) {
      graph["metadata"]["gps_datum"] = YAML::Node();
      graph["metadata"]["gps_datum"]["latitude"] = gps_trans->LatitudeOrigin();
      graph["metadata"]["gps_datum"]["longitude"] = gps_trans->LongitudeOrigin();
      graph["metadata"]["gps_datum"]["altitude"] = gps_trans->HeightOrigin();
    }
  
  // Add vertices (keyframe poses)
  YAML::Node vertices = YAML::Node(YAML::NodeType::Sequence);
  for(const auto& vertex : isam_current_estimate) {
    YAML::Node vertex_node;
    vertex_node["id"] = vertex.key;
    
    gtsam::Pose3 pose = vertex.value.cast<gtsam::Pose3>();
    
    // Translation
    YAML::Node translation;
    translation["x"] = pose.translation().x();
    translation["y"] = pose.translation().y();
    translation["z"] = pose.translation().z();
    vertex_node["translation"] = translation;
    
    // Rotation (quaternion)
    YAML::Node rotation;
    gtsam::Quaternion quat = pose.rotation().toQuaternion();
    rotation["x"] = quat.x();
    rotation["y"] = quat.y();
    rotation["z"] = quat.z();
    rotation["w"] = quat.w();
    vertex_node["rotation"] = rotation;
    
    // Euler angles for readability
    YAML::Node euler;
    euler["roll"] = pose.rotation().roll();
    euler["pitch"] = pose.rotation().pitch();
    euler["yaw"] = pose.rotation().yaw();
    vertex_node["euler"] = euler;
    
    // Timestamp if available
    if (vertex.key < keyframe_stamps.size()) {
      vertex_node["timestamp"] = keyframe_stamps[vertex.key];
    }
    
    vertices.push_back(vertex_node);
  }
  graph["vertices"] = vertices;
  
  // Add factors
  YAML::Node factors = YAML::Node(YAML::NodeType::Sequence);
  
  for(const auto& factor_ : isam.getFactorsUnsafe()) {
    YAML::Node factor_node;
    
    // PriorFactor
    auto prior_factor = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor_);
    if(prior_factor) {
      factor_node["type"] = "PriorFactor";
      factor_node["key"] = prior_factor->key();
      
      gtsam::Pose3 prior_pose = prior_factor->prior();
      
      // Prior pose
      YAML::Node prior_pose_node;
      YAML::Node prior_translation;
      prior_translation["x"] = prior_pose.translation().x();
      prior_translation["y"] = prior_pose.translation().y();
      prior_translation["z"] = prior_pose.translation().z();
      prior_pose_node["translation"] = prior_translation;
      
      gtsam::Quaternion prior_quat = prior_pose.rotation().toQuaternion();
      YAML::Node prior_rotation;
      prior_rotation["x"] = prior_quat.x();
      prior_rotation["y"] = prior_quat.y();
      prior_rotation["z"] = prior_quat.z();
      prior_rotation["w"] = prior_quat.w();
      prior_pose_node["rotation"] = prior_rotation;
      
      factor_node["prior_pose"] = prior_pose_node;
      
      // Noise model
      YAML::Node noise_node;
      auto noise_model = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(prior_factor->noiseModel());
      if (noise_model) {
        gtsam::Vector sigmas = noise_model->sigmas();
        YAML::Node sigmas_node = YAML::Node(YAML::NodeType::Sequence);
        for (int i = 0; i < sigmas.size(); i++) {
          sigmas_node.push_back(sigmas(i));
        }
        noise_node["sigmas"] = sigmas_node;
        factor_node["noise_model"] = noise_node;
      }
    }
    
    // BetweenFactor
    auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor_);
    if(between_factor) {
      factor_node["type"] = "BetweenFactor";
      factor_node["key1"] = between_factor->key1();
      factor_node["key2"] = between_factor->key2();
      
      gtsam::Pose3 measured_pose = between_factor->measured();
      
      // Measured pose
      YAML::Node measured_node;
      YAML::Node measured_translation;
      measured_translation["x"] = measured_pose.translation().x();
      measured_translation["y"] = measured_pose.translation().y();
      measured_translation["z"] = measured_pose.translation().z();
      measured_node["translation"] = measured_translation;
      
      gtsam::Quaternion measured_quat = measured_pose.rotation().toQuaternion();
      YAML::Node measured_rotation;
      measured_rotation["x"] = measured_quat.x();
      measured_rotation["y"] = measured_quat.y();
      measured_rotation["z"] = measured_quat.z();
      measured_rotation["w"] = measured_quat.w();
      measured_node["rotation"] = measured_rotation;
      
      factor_node["measured_pose"] = measured_node;
      
      // Noise model
      YAML::Node noise_node;
      auto noise_model = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(between_factor->noiseModel());
      if (noise_model) {
        gtsam::Vector sigmas = noise_model->sigmas();
        YAML::Node sigmas_node = YAML::Node(YAML::NodeType::Sequence);
        for (int i = 0; i < sigmas.size(); i++) {
          sigmas_node.push_back(sigmas(i));
        }
        noise_node["sigmas"] = sigmas_node;
        factor_node["noise_model"] = noise_node;
      }
    }
    
    // GPSFactor
    auto gps_factor = boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor_);
    if(gps_factor) {
      factor_node["type"] = "GPSFactor";
      factor_node["key"] = gps_factor->key();
      
      gtsam::Point3 gps_measurement = gps_factor->measurementIn();
      
      // GPS measurement
      YAML::Node gps_measurement_node;
      gps_measurement_node["x"] = gps_measurement.x();
      gps_measurement_node["y"] = gps_measurement.y();
      gps_measurement_node["z"] = gps_measurement.z();
      factor_node["gps_measurement"] = gps_measurement_node;
      
      // Noise model
      YAML::Node noise_node;
      auto noise_model = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(gps_factor->noiseModel());
      if (noise_model) {
        gtsam::Vector sigmas = noise_model->sigmas();
        YAML::Node sigmas_node = YAML::Node(YAML::NodeType::Sequence);
        for (int i = 0; i < sigmas.size(); i++) {
          sigmas_node.push_back(sigmas(i));
        }
        noise_node["sigmas"] = sigmas_node;
        factor_node["noise_model"] = noise_node;
      }
    }
    
    if (!factor_node["type"]) {
      // Unknown factor type, skip
      continue;
    }
    
    factors.push_back(factor_node);
  }
  
  graph["factors"] = factors;
  
  // Save to file
  std::ofstream yaml_stream(yaml_file);
  yaml_stream << graph;
  yaml_stream.close();
  
  std::cout << "Factor graph saved to YAML: " << yaml_file << std::endl;
}