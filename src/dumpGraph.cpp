#include "dumpGraph.h"

#include <ros/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>
#include <cmath>
#include <iomanip>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <yaml-cpp/yaml.h>
#include <GeographicLib/LocalCartesian.hpp>

bool dump(const std::string& dump_directory,
  const gtsam::NonlinearFactorGraph& factors,
  const gtsam::Values& isam_current_estimate,
  const std::vector<double>& keyframe_stamps,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes
) {
  boost::system::error_code ec;
  boost::filesystem::create_directories(dump_directory, ec);
  if (ec) {
    std::cerr << "dump: cannot create " << dump_directory << ": " << ec.message() << std::endl;
    return false;
  }

  std::vector<gtsam::Pose3> keyframe_poses(isam_current_estimate.size());

  AtomicOfstream graph_file(dump_directory + "/graph.g2o");
  std::ofstream& graph_ofs = graph_file.stream();
  graph_ofs << std::setprecision(17);  // default 6 digits rounded km-scale coordinates to ~1 cm
  for(const auto& vertex : isam_current_estimate) {
    Eigen::Matrix4d pose = vertex.value.cast<gtsam::Pose3>().matrix();
    keyframe_poses[vertex.key] = vertex.value.cast<gtsam::Pose3>();
    Eigen::Vector3d t = pose.block<3, 1>(0, 3);  // (used to add Vector3d::Random(), up to 1 m of noise per axis)
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    graph_ofs << "VERTEX_SE3:QUAT " << vertex.key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  graph_ofs << "FIX 0" << "\n";

  std::cout << "factors:" << factors.size() << std::endl;
  for(const auto& factor_: factors) {
    factor_->print("factor: ");
    auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor_);
    if(between_factor) {
      Eigen::Matrix4d relative;
      Eigen::Vector3d t;
      Eigen::Quaterniond q;
      Eigen::MatrixXd inf;
      try
      {
        relative = between_factor->measured().matrix();
        t = relative.block<3, 1>(0, 3);
        q = relative.block<3, 3>(0, 0);
        // g2o information of EDGE_SE3:QUAT, ordered [x y z qx qy qz]: 1/sigma^2 for the translation, and
        // for the rotation 4/sigma^2, because g2o's rotation error is the quaternion's vector part
        // (~theta/2) while gtsam's sigmas are for the rotation vector (theta). gtsam orders its sigmas
        // [rot, trans]. (Before, this wrote diag(1/sigma) in gtsam order; islam_to_liorf.py
        // --legacy-g2o-info reads such files.)
        // A Robust model has no sigmas() of its own (throws): write its Gaussian part (g2o has no kernel field).
        gtsam::noiseModel::Base::shared_ptr nm = between_factor->noiseModel();
        if (auto robust = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(nm)) nm = robust->noise();
        const Eigen::VectorXd sig = nm->sigmas();
        Eigen::VectorXd d(6);
        d << sig.tail<3>().array().square().inverse(), 4.0 * sig.head<3>().array().square().inverse();
        inf = d.asDiagonal();
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

  // graph.g2o is committed after the keyframes below, so a save that fails part-way leaves the previous
  // g2o (not a new one next to a mix of old and new keyframes). Its temp file stays open until then.
  std::cout << "saving clouds" << std::endl;
  for(int i = 0; i < surf_cloud_keyframes.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % dump_directory % i).str();
    boost::filesystem::create_directories(keyframe_directory, ec);
    if (ec) {
      std::cerr << "dump: cannot create " << keyframe_directory << ": " << ec.message() << std::endl;
      return false;
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // *cloud += *surf_cloud_keyframes[i];

    // Eigen::Isometry3f camera2lidar = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY()) * Eigen::Isometry3f::Identity();

    // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::transformPointCloud(*cloud, *transformed, camera2lidar);

    // cloud = transformPointCloud(cloud, keyframe_poses[i]);
    if (!savePCDFileBinaryAtomic(keyframe_directory + "/cloud.pcd", *surf_cloud_keyframes[i]))
      return false;

    ros::Time stamp(keyframe_stamps[i]);

    AtomicOfstream data_file(keyframe_directory + "/data");
    std::ofstream& data_ofs = data_file.stream();
    data_ofs << std::setprecision(17);
    data_ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";
    data_ofs << "estimate\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "odom\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "accum_distance -1" << "\n";
    data_ofs << "id " << i << "\n";
    if (!data_file.commit()) {
      std::cerr << "dump: could not write " << keyframe_directory << "/data" << std::endl;
      return false;
    }
  }

  if (!graph_file.commit()) {
    std::cerr << "dump: could not write " << dump_directory << "/graph.g2o" << std::endl;
    return false;
  }

  // Keyframe dirs from an earlier, larger save are not touched: they are not part of this graph, but they
  // do mislead anything that lists the directory. Say so instead of silently carrying them along.
  size_t stale = 0;
  for (boost::filesystem::directory_iterator it(dump_directory, ec), end; !ec && it != end; it.increment(ec)) {
    const std::string name = it->path().filename().string();
    if (name.size() >= 6 && name.size() <= 18 && name.find_first_not_of("0123456789") == std::string::npos &&
        std::stoull(name) >= surf_cloud_keyframes.size() && boost::filesystem::is_directory(it->path(), ec))
      ++stale;
  }
  if (stale > 0)
    std::cerr << "dump: WARNING: " << stale << " keyframe directories in " << dump_directory
              << " were not written by this save (ids >= " << surf_cloud_keyframes.size()
              << "), left over from an earlier, larger save; they are not part of this graph." << std::endl;
  return true;
}

// Writes the sigmas of a diagonal noise model as a YAML flow-style sequence, e.g. "[1, 2, 3]".
// A Robust model writes the sigmas of its Gaussian part plus its m-estimator, e.g.
// "robust: {type: huber, k: 1.345}", so FactorGraphLoader can rebuild it. (Before, robust models —
// liorf's inter-session loop closures — were written with no noise_model at all and came back from
// the loader with its tight default sigmas.)
static void writeSigmas(std::ofstream& ofs, const gtsam::noiseModel::Base::shared_ptr& noise_model_base) {
  gtsam::noiseModel::Base::shared_ptr gaussian = noise_model_base;
  std::string robust_type;
  double robust_k = 0.0;
  if (auto robust = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise_model_base)) {
    gaussian = robust->noise();
    const auto& estimator = robust->robust();
    // GTSAM 4.0.3 has no accessor for the estimator parameter; recover it from weight().
    if (boost::dynamic_pointer_cast<gtsam::noiseModel::mEstimator::Huber>(estimator)) {
      const double d = 1e6;                      // Huber: weight(d) = k / d for d > k
      robust_type = "huber";
      robust_k = estimator->weight(d) * d;
    } else if (boost::dynamic_pointer_cast<gtsam::noiseModel::mEstimator::Cauchy>(estimator)) {
      const double w = estimator->weight(1.0);   // Cauchy: weight(1) = k^2 / (k^2 + 1)
      robust_type = "cauchy";
      robust_k = std::sqrt(w / (1.0 - w));
    } else {
      std::cerr << "dumpYAML: unsupported robust estimator, writing its Gaussian sigmas only" << std::endl;
    }
  }
  auto noise_model = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(gaussian);
  if (!noise_model) return;
  gtsam::Vector sigmas = noise_model->sigmas();
  ofs << "    noise_model:\n      sigmas: [";
  for (int i = 0; i < sigmas.size(); i++) {
    if (i > 0) ofs << ", ";
    ofs << sigmas(i);
  }
  ofs << "]\n";
  if (!robust_type.empty())
    ofs << "      robust: {type: " << robust_type << ", k: " << robust_k << "}\n";
}

bool dumpYAML(const std::string& dump_directory,
  const gtsam::NonlinearFactorGraph& factors,
  const gtsam::Values& isam_current_estimate,
  const std::vector<double>& keyframe_stamps,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes,
  const GeographicLib::LocalCartesian* gps_trans
) {
  // Writes the same schema FactorGraphLoader::loadYAML() parses, but streams plain text
  // directly to disk instead of building a YAML::Node tree in memory first. At ~60k vertices
  // + ~60k factors, that tree held ~600k+ individual yaml-cpp Node objects (each vertex/factor
  // needs several nested maps for translation/rotation/noise_model/etc.) — yaml-cpp's per-node
  // overhead (shared_ptr-backed, several heap allocations per map entry) made that tree balloon
  // into tens of GB, which is what was OOM-killing save_map on long chained sessions.
  boost::system::error_code ec;
  boost::filesystem::create_directories(dump_directory, ec);
  if (ec) {
    std::cerr << "dumpYAML: cannot create " << dump_directory << ": " << ec.message() << std::endl;
    return false;
  }

  // Replaced atomically (temp file + rename): this file is the commit point of a save, and a half-written
  // one used to reload as a shorter graph that the next chained save then overwrote the good graph with.
  std::string yaml_path = dump_directory + "/factor_graph.yaml";
  AtomicOfstream yaml_file(yaml_path);
  std::ofstream& ofs = yaml_file.stream();
  ofs << std::setprecision(17);

  // total_factors is the number of factors written below (unsupported types are skipped), so a loader
  // can tell a complete file from a truncated one.
  size_t factors_written = 0;
  for (const auto& factor_ : factors)
    if (boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor_) ||
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor_) ||
        boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor_))
      ++factors_written;

  ofs << "metadata:\n";
  ofs << "  total_keyframes: " << isam_current_estimate.size() << "\n";
  ofs << "  total_factors: " << factors_written << "\n";
  ofs << "  dump_timestamp: " << ros::Time::now().toSec() << "\n";
  if (gps_trans != nullptr) {
    ofs << "  gps_datum:\n";
    ofs << "    latitude: " << gps_trans->LatitudeOrigin() << "\n";
    ofs << "    longitude: " << gps_trans->LongitudeOrigin() << "\n";
    ofs << "    altitude: " << gps_trans->HeightOrigin() << "\n";
  }

  ofs << "vertices:\n";
  for (const auto& vertex : isam_current_estimate) {
    gtsam::Pose3 pose = vertex.value.cast<gtsam::Pose3>();
    gtsam::Quaternion quat = pose.rotation().toQuaternion();
    ofs << "  - id: " << vertex.key << "\n";
    ofs << "    translation: {x: " << pose.translation().x() << ", y: " << pose.translation().y() << ", z: " << pose.translation().z() << "}\n";
    ofs << "    rotation: {x: " << quat.x() << ", y: " << quat.y() << ", z: " << quat.z() << ", w: " << quat.w() << "}\n";
    ofs << "    euler: {roll: " << pose.rotation().roll() << ", pitch: " << pose.rotation().pitch() << ", yaw: " << pose.rotation().yaw() << "}\n";
    if (vertex.key < keyframe_stamps.size()) {
      ofs << "    timestamp: " << keyframe_stamps[vertex.key] << "\n";
    }
  }

  ofs << "factors:\n";
  for (const auto& factor_ : factors) {
    auto prior_factor = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor_);
    if (prior_factor) {
      gtsam::Pose3 prior_pose = prior_factor->prior();
      gtsam::Quaternion prior_quat = prior_pose.rotation().toQuaternion();
      ofs << "  - type: PriorFactor\n";
      ofs << "    key: " << prior_factor->key() << "\n";
      ofs << "    prior_pose:\n";
      ofs << "      translation: {x: " << prior_pose.translation().x() << ", y: " << prior_pose.translation().y() << ", z: " << prior_pose.translation().z() << "}\n";
      ofs << "      rotation: {x: " << prior_quat.x() << ", y: " << prior_quat.y() << ", z: " << prior_quat.z() << ", w: " << prior_quat.w() << "}\n";
      writeSigmas(ofs, prior_factor->noiseModel());
      continue;
    }

    auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor_);
    if (between_factor) {
      gtsam::Pose3 measured_pose = between_factor->measured();
      gtsam::Quaternion measured_quat = measured_pose.rotation().toQuaternion();
      ofs << "  - type: BetweenFactor\n";
      ofs << "    key1: " << between_factor->key1() << "\n";
      ofs << "    key2: " << between_factor->key2() << "\n";
      ofs << "    measured_pose:\n";
      ofs << "      translation: {x: " << measured_pose.translation().x() << ", y: " << measured_pose.translation().y() << ", z: " << measured_pose.translation().z() << "}\n";
      ofs << "      rotation: {x: " << measured_quat.x() << ", y: " << measured_quat.y() << ", z: " << measured_quat.z() << ", w: " << measured_quat.w() << "}\n";
      writeSigmas(ofs, between_factor->noiseModel());
      continue;
    }

    auto gps_factor = boost::dynamic_pointer_cast<gtsam::GPSFactor>(factor_);
    if (gps_factor) {
      gtsam::Point3 gps_measurement = gps_factor->measurementIn();
      ofs << "  - type: GPSFactor\n";
      ofs << "    key: " << gps_factor->key() << "\n";
      ofs << "    gps_measurement: {x: " << gps_measurement.x() << ", y: " << gps_measurement.y() << ", z: " << gps_measurement.z() << "}\n";
      writeSigmas(ofs, gps_factor->noiseModel());
      continue;
    }
    // Unknown factor type, skip — matches the previous behavior.
  }

  if (!yaml_file.commit()) {
    std::cerr << "dumpYAML: could not write " << yaml_path << " (the previous file, if any, is unchanged)" << std::endl;
    return false;
  }

  std::cout << "Factor graph saved to YAML: " << yaml_path << std::endl;
  return true;
}