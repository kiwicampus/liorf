// Offline dense map export.
//
// liorf's cloudGlobal.pcd only contains keyframe clouds (one scan per 0.5 m / 0.2 rad, every 3rd
// point, voxelized to mappingSurfLeafSize). This tool rebuilds the map from *every* scan in the
// recorded bag(s), placed with the final keyframe trajectory from a graph dump (liorf save_map,
// loadSession chains, HBA poses pushed back into factor_graph.yaml, or interactive_slam output):
//
//   1. Every point gets its own pose, interpolated (slerp/lerp) between the two keyframes that
//      bracket its absolute timestamp. That both deskews the scan and applies the loop-closed /
//      GPS-corrected trajectory.
//   2. (default) Each scan is then refined with point-to-plane ICP against a local submap built
//      from the neighbouring keyframe clouds (graph/NNNNNN/cloud.pcd) at their optimized poses.
//      Interpolation alone is off by up to the keyframe spacing when the robot stops or turns
//      between keyframes; the ICP correction is carried over from scan to scan so it tracks
//      those errors instead of re-solving them from scratch.
//   3. Points are spilled to per-tile files on disk, then each tile is voxel-deduplicated
//      (centroid per voxel) and written as a PCD, so RAM is bounded by one tile, not the map.
//
// Output points are in the same map frame as cloudGlobal.pcd.

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <omp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "liorf/FactorGraphLoader.h"

namespace fs = boost::filesystem;

struct Options {
  std::string graph;
  std::vector<std::string> bags;
  std::string out;
  std::string topic = "/livox/filtered";
  std::string imu_topic = "/livox/imu";  // raw sensor-frame gyro, leveled with the same rotation
  double gyro_max_resid = 0.1;           // rad; segments where gyro and keyframes disagree more use slerp
  double stamp_offset = 0.0;             // s added to cloud + IMU header stamps; = stampOffset launch arg used when mapping
  std::string pose_source = "auto";  // auto | yaml | data
  // Same leveling rotation pointcloud_transform_node applies before liorf (3d_mapping.launch).
  double rx = 0.0, ry = 0.35, rz = 0.0;
  double min_range = 1.0;  // > liorf's 0.3: self-hits on the robot would smear along the whole path
  double max_range = 40.0;
  double voxel = 0.03;  // output dedup voxel (m); 0 keeps every point
  double tile = 50.0;
  bool refine = true;
  int submap_kf = 10;  // keyframes on each side of the scan's segment used for the ICP submap
  double icp_leaf = 0.25;
  double icp_max_dist = 1.0;
  int icp_max_iter = 15;
  double icp_min_inlier_ratio = 0.6;
  double max_gap = 60.0;  // keyframe gaps longer than this (s) are treated as "no trajectory"
  double start = 0.0;     // s from the first scan of each bag
  double duration = -1.0;
  int every = 1;
  int max_static_scans = 20;  // consecutive scans kept while stationary (0 = unlimited)
  bool merged = false;
  bool keep_spill = false;
  int threads = 0;
};

static void usage() {
  std::cout <<
R"(Usage: rosrun liorf liorf_densify_map --graph <graph_dir> --bag <bag> [--bag <bag2> ...] --out <dir> [options]

Rebuilds a dense point cloud from every scan in the bag(s) using the optimized keyframe trajectory
of <graph_dir> (the "graph/" folder written by save_map). Output: <dir>/tiles/tile_<ix>_<iy>.pcd
(and <dir>/dense_map.pcd with --merged), in the same frame as cloudGlobal.pcd, plus
<dir>/scan_poses.txt (TUM: stamp x y z qx qy qz qw) with the final pose of every kept scan.

Input
  --topic <name>            lidar topic in the bag (default /livox/filtered)
  --imu-topic <name>        gyro for rotation between keyframes (default /livox/imu)
  --no-imu                  slerp between keyframes instead
  --stamp-offset <s>        seconds added to cloud and IMU header stamps; must equal the stampOffset
                            launch arg used when mapping (needed when the driver stamped with sensor
                            time), default 0
  --gyro-max-resid <rad>    fall back to slerp where gyro vs keyframes disagree more (default 0.1)
  --pose-source auto|yaml|data
                            yaml: factor_graph.yaml vertices (liorf, loadSession chains, HBA)
                            data: NNNNNN/data "estimate" (interactive_slam saves)
                            auto: yaml if factor_graph.yaml exists, else data
  --level-rpy <rx> <ry> <rz>
                            sensor leveling rotation applied before liorf (default 0 0.35 0)
Filtering
  --min-range <m>           default 1.0
  --max-range <m>           default 40
  --max-gap <s>             skip scans between keyframes further apart than this (default 60)
  --max-static-scans <n>    scans kept per stationary period, 0 = all (default 20)
  --start <s> --duration <s>
                            only process a window (relative to each bag's first scan)
  --every <n>               keep every n-th scan (default 1)
Refinement
  --no-refine               interpolation only (faster; blurs where the robot stops/turns)
  --submap-kf <n>           keyframes each side for the ICP submap (default 10)
  --icp-leaf <m>            ICP downsampling (default 0.25)
  --icp-max-dist <m>        max correspondence distance (default 1.0)
  --icp-max-iter <n>        default 15
  --icp-min-inlier-ratio <r>
                            reject (drop) scans whose refined alignment is worse (default 0.6)
Output
  --voxel <m>               dedup voxel, centroid per voxel; 0 keeps every point (default 0.03)
  --tile <m>                tile size (default 50)
  --merged                  also write <out>/dense_map.pcd
  --keep-spill              keep the raw per-tile spill files
  --threads <n>             OpenMP threads (default: all)
)";
}

static bool parseArgs(int argc, char** argv, Options& o) {
  std::vector<std::string> a(argv + 1, argv + argc);
  auto need = [&](size_t& i, int n) {
    if (i + n >= a.size()) throw std::runtime_error("missing value for " + a[i]);
  };
  for (size_t i = 0; i < a.size(); ++i) {
    const std::string& k = a[i];
    if (k == "-h" || k == "--help") return false;
    else if (k == "--graph") { need(i, 1); o.graph = a[++i]; }
    else if (k == "--bag") { need(i, 1); o.bags.push_back(a[++i]); }
    else if (k == "--out") { need(i, 1); o.out = a[++i]; }
    else if (k == "--topic") { need(i, 1); o.topic = a[++i]; }
    else if (k == "--imu-topic") { need(i, 1); o.imu_topic = a[++i]; }
    else if (k == "--no-imu") { o.imu_topic.clear(); }
    else if (k == "--stamp-offset") { need(i, 1); o.stamp_offset = std::stod(a[++i]); }
    else if (k == "--gyro-max-resid") { need(i, 1); o.gyro_max_resid = std::stod(a[++i]); }
    else if (k == "--pose-source") { need(i, 1); o.pose_source = a[++i]; }
    else if (k == "--level-rpy") { need(i, 3); o.rx = std::stod(a[++i]); o.ry = std::stod(a[++i]); o.rz = std::stod(a[++i]); }
    else if (k == "--min-range") { need(i, 1); o.min_range = std::stod(a[++i]); }
    else if (k == "--max-range") { need(i, 1); o.max_range = std::stod(a[++i]); }
    else if (k == "--voxel") { need(i, 1); o.voxel = std::stod(a[++i]); }
    else if (k == "--tile") { need(i, 1); o.tile = std::stod(a[++i]); }
    else if (k == "--no-refine") { o.refine = false; }
    else if (k == "--submap-kf") { need(i, 1); o.submap_kf = std::stoi(a[++i]); }
    else if (k == "--icp-leaf") { need(i, 1); o.icp_leaf = std::stod(a[++i]); }
    else if (k == "--icp-max-dist") { need(i, 1); o.icp_max_dist = std::stod(a[++i]); }
    else if (k == "--icp-max-iter") { need(i, 1); o.icp_max_iter = std::stoi(a[++i]); }
    else if (k == "--icp-min-inlier-ratio") { need(i, 1); o.icp_min_inlier_ratio = std::stod(a[++i]); }
    else if (k == "--max-gap") { need(i, 1); o.max_gap = std::stod(a[++i]); }
    else if (k == "--start") { need(i, 1); o.start = std::stod(a[++i]); }
    else if (k == "--duration") { need(i, 1); o.duration = std::stod(a[++i]); }
    else if (k == "--every") { need(i, 1); o.every = std::max(1, std::stoi(a[++i])); }
    else if (k == "--max-static-scans") { need(i, 1); o.max_static_scans = std::stoi(a[++i]); }
    else if (k == "--merged") { o.merged = true; }
    else if (k == "--keep-spill") { o.keep_spill = true; }
    else if (k == "--threads") { need(i, 1); o.threads = std::stoi(a[++i]); }
    else throw std::runtime_error("unknown argument " + k);
  }
  if (o.graph.empty() || o.bags.empty() || o.out.empty())
    throw std::runtime_error("--graph, --bag and --out are required");
  if (o.tile <= 0 || o.voxel < 0 || (o.voxel > 0 && o.tile / o.voxel > (1 << 20)))
    throw std::runtime_error("bad --tile/--voxel combination");
  return true;
}

// ---------------------------------------------------------------------------------------------
// Trajectory

struct Keyframe {
  int id;
  double t;
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
};

static std::vector<Keyframe> loadYamlPoses(const std::string& graph) {
  FactorGraphLoader loader;
  if (!loader.loadSession(graph, /*optimize=*/false, /*load_clouds=*/false))
    throw std::runtime_error("failed to load " + graph + "/factor_graph.yaml");
  std::vector<Keyframe> kfs;
  size_t no_stamp = 0;
  for (const auto& kv : loader.getKeyframeData()) {
    const auto& kd = kv.second;
    if (kd->timestamp <= 0.0) { ++no_stamp; continue; }
    Eigen::Matrix4d m = kd->pose.matrix();
    kfs.push_back({kd->id, kd->timestamp, Eigen::Quaterniond(m.block<3, 3>(0, 0)), m.block<3, 1>(0, 3)});
  }
  if (no_stamp) std::cout << "WARN: " << no_stamp << " vertices without timestamp ignored\n";
  return kfs;
}

// interactive_slam / hdl_graph_slam layout: NNNNNN/data with "stamp <sec> <nsec>" + "estimate" 4x4.
// Dump dirs can outlive a smaller re-save (stale ids past the vertex count), so bound by graph.g2o.
static std::vector<Keyframe> loadDataPoses(const std::string& graph) {
  long n_vertices = -1;
  std::ifstream g2o(graph + "/graph.g2o");
  if (g2o) {
    n_vertices = 0;
    std::string line;
    while (std::getline(g2o, line))
      if (line.rfind("VERTEX_SE3", 0) == 0) ++n_vertices;
  }
  std::vector<Keyframe> kfs;
  for (long id = 0; n_vertices < 0 || id < n_vertices; ++id) {
    std::ifstream in((boost::format("%s/%06d/data") % graph % id).str());
    if (!in) {
      if (n_vertices < 0) break;
      continue;
    }
    Keyframe kf{(int)id, -1.0, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()};
    std::string tok;
    bool have_pose = false;
    while (in >> tok) {
      if (tok == "stamp") {
        double sec, nsec;
        in >> sec >> nsec;
        kf.t = sec + nsec * 1e-9;
      } else if (tok == "estimate") {
        Eigen::Matrix4d m;
        for (int r = 0; r < 4; ++r)
          for (int c = 0; c < 4; ++c) in >> m(r, c);
        kf.q = Eigen::Quaterniond(m.block<3, 3>(0, 0)).normalized();
        kf.p = m.block<3, 1>(0, 3);
        have_pose = true;
      }
    }
    if (have_pose && kf.t > 0) kfs.push_back(kf);
  }
  return kfs;
}

static Eigen::Quaterniond expSO3(const Eigen::Vector3d& w) {
  const double a = w.norm();
  if (a < 1e-12) return Eigen::Quaterniond(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
  return Eigen::Quaterniond(Eigen::AngleAxisd(a, w / a));
}

static Eigen::Vector3d logSO3(const Eigen::Quaterniond& q) {
  const Eigen::AngleAxisd aa(q.normalized());
  return aa.angle() * aa.axis();
}

// Integrated gyro orientation (leveled lidar frame), used for the rotation *profile* between
// keyframes. Absolute orientation always comes from the keyframes.
class GyroTrack {
 public:
  void add(double t, const Eigen::Vector3d& w) { samples_.push_back({t, w}); }

  void finalize() {
    std::sort(samples_.begin(), samples_.end(), [](const Sample& a, const Sample& b) { return a.t < b.t; });
    t_.resize(samples_.size());
    q_.resize(samples_.size());
    w_.resize(samples_.size());
    for (size_t i = 0; i < samples_.size(); ++i) {
      t_[i] = samples_[i].t;
      // Midpoint rate over [t_i, t_i+1]; after a dropout just hold the sample's own rate.
      const bool next = i + 1 < samples_.size() && samples_[i + 1].t - t_[i] <= kMaxDt;
      w_[i] = next ? 0.5 * (samples_[i].w + samples_[i + 1].w) : samples_[i].w;
      q_[i] = i == 0 ? Eigen::Quaterniond::Identity()
                     : (q_[i - 1] * expSO3(w_[i - 1] * std::min(t_[i] - t_[i - 1], kMaxDt))).normalized();
    }
    samples_.clear();
    samples_.shrink_to_fit();
  }

  bool empty() const { return t_.empty(); }
  size_t size() const { return t_.size(); }

  // True when [a, b] is covered without dropouts.
  bool covers(double a, double b) const {
    if (t_.empty() || a < t_.front() || b > t_.back()) return false;
    size_t i = index(a);
    for (; i + 1 < t_.size() && t_[i] < b; ++i)
      if (t_[i + 1] - t_[i] > kMaxDt) return false;
    return true;
  }

  Eigen::Quaterniond at(double t, size_t& hint) const {
    if (!(hint + 1 < t_.size() && t_[hint] <= t && t < t_[hint + 1])) hint = index(t);
    return q_[hint] * expSO3(w_[hint] * (t - t_[hint]));
  }

 private:
  static constexpr double kMaxDt = 0.05;
  struct Sample { double t; Eigen::Vector3d w; };
  size_t index(double t) const {
    size_t i = std::upper_bound(t_.begin(), t_.end(), t) - t_.begin();
    return i == 0 ? 0 : i - 1;
  }
  std::vector<Sample> samples_;
  std::vector<double> t_;
  std::vector<Eigen::Quaterniond> q_;
  std::vector<Eigen::Vector3d> w_;
};

class Trajectory {
 public:
  explicit Trajectory(std::vector<Keyframe> kfs) : kfs_(std::move(kfs)) {
    std::sort(kfs_.begin(), kfs_.end(), [](const Keyframe& a, const Keyframe& b) { return a.t < b.t; });
    std::vector<Keyframe> uniq;
    for (const auto& k : kfs_)
      if (uniq.empty() || k.t - uniq.back().t > 1e-6) uniq.push_back(k);
    if (uniq.size() != kfs_.size())
      std::cout << "WARN: dropped " << kfs_.size() - uniq.size() << " keyframes with duplicate stamps\n";
    kfs_.swap(uniq);
    for (const auto& k : kfs_) t_.push_back(k.t);
  }

  size_t size() const { return kfs_.size(); }
  const Keyframe& kf(int i) const { return kfs_[i]; }
  double gap(int k) const { return t_[k + 1] - t_[k]; }

  // Segment k with t_[k] <= t <= t_[k+1], or -1 outside the trajectory. `hint` is checked first
  // since consecutive points almost always fall in the same segment.
  int segment(double t, int hint = -1) const {
    if (t_.size() < 2 || t < t_.front() || t > t_.back()) return -1;
    if (hint >= 0 && hint + 1 < (int)t_.size() && t >= t_[hint] && t <= t_[hint + 1]) return hint;
    int k = int(std::upper_bound(t_.begin(), t_.end(), t) - t_.begin()) - 1;
    return std::min(k, (int)t_.size() - 2);
  }

  // With a gyro track, rotation between keyframes a and b follows the integrated gyro, and the
  // mismatch at b (bias, noise) is spread linearly over the segment so both ends stay exactly on
  // the optimized keyframes:  R(t) = R_a * dR_gyro(t_a -> t) * Exp(s * Log(E)).
  // Otherwise (or if the gyro disagrees by more than max_resid) it's a plain slerp.
  void setGyro(const GyroTrack* g, double max_resid) {
    gyro_ = g;
    max_resid_ = max_resid;
  }

  Eigen::Isometry3d interp(int k, double t) const {
    const Keyframe& a = kfs_[k];
    const Keyframe& b = kfs_[k + 1];
    double dt = b.t - a.t;
    double s = dt > 1e-9 ? std::min(1.0, std::max(0.0, (t - a.t) / dt)) : 0.0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const SegRot& sr = segRot(k);
    if (sr.gyro)
      T.linear() = (a.q * (sr.qa_inv * gyro_->at(t, gyro_hint_)) * expSO3(s * sr.log_e)).toRotationMatrix();
    else
      T.linear() = a.q.slerp(s, b.q).toRotationMatrix();
    T.translation() = a.p + s * (b.p - a.p);
    return T;
  }

  // Gyro usage over the segments touched so far: (segments with gyro, without, mean residual rad)
  void gyroStats(size_t& used, size_t& fallback, double& mean_resid) const {
    used = gyro_used_;
    fallback = gyro_fallback_;
    mean_resid = gyro_used_ ? resid_sum_ / gyro_used_ : 0.0;
  }

  Eigen::Isometry3d pose(int i) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = kfs_[i].q.toRotationMatrix();
    T.translation() = kfs_[i].p;
    return T;
  }

 private:
  struct SegRot {
    int k = -1;
    bool gyro = false;
    Eigen::Quaterniond qa_inv;
    Eigen::Vector3d log_e;
  };

  const SegRot& segRot(int k) const {
    if (seg_.k == k) return seg_;
    seg_ = SegRot();
    seg_.k = k;
    const Keyframe& a = kfs_[k];
    const Keyframe& b = kfs_[k + 1];
    if (gyro_ && gyro_->covers(a.t, b.t)) {
      seg_.qa_inv = gyro_->at(a.t, gyro_hint_).inverse();
      const Eigen::Quaterniond pred = a.q * (seg_.qa_inv * gyro_->at(b.t, gyro_hint_));
      seg_.log_e = logSO3(pred.inverse() * b.q);
      seg_.gyro = seg_.log_e.norm() <= max_resid_;
    }
    if (counted_.empty()) counted_.assign(kfs_.size(), 0);
    if (!counted_[k]) {
      counted_[k] = 1;
      if (seg_.gyro) {
        ++gyro_used_;
        resid_sum_ += seg_.log_e.norm();
      } else {
        ++gyro_fallback_;
      }
    }
    return seg_;
  }

  std::vector<Keyframe> kfs_;
  std::vector<double> t_;
  const GyroTrack* gyro_ = nullptr;
  double max_resid_ = 0.1;
  mutable SegRot seg_;
  mutable std::vector<uint8_t> counted_;
  mutable size_t gyro_hint_ = 0, gyro_used_ = 0, gyro_fallback_ = 0;
  mutable double resid_sum_ = 0.0;
};

// ---------------------------------------------------------------------------------------------
// Geometry helpers

using Cloud3f = std::vector<Eigen::Vector3f>;

static inline uint64_t packKey(int64_t x, int64_t y, int64_t z) {
  const int64_t off = 1 << 20;
  return (uint64_t(x + off) & 0x1FFFFF) | ((uint64_t(y + off) & 0x1FFFFF) << 21) |
         ((uint64_t(z + off) & 0x1FFFFF) << 42);
}

static Cloud3f voxelCentroids(const Cloud3f& in, float leaf) {
  struct Acc { Eigen::Vector3f s = Eigen::Vector3f::Zero(); int n = 0; };
  std::unordered_map<uint64_t, Acc> grid;
  grid.reserve(in.size());
  for (const auto& p : in) {
    auto& a = grid[packKey((int64_t)std::floor(p.x() / leaf), (int64_t)std::floor(p.y() / leaf),
                           (int64_t)std::floor(p.z() / leaf))];
    a.s += p;
    ++a.n;
  }
  Cloud3f out;
  out.reserve(grid.size());
  for (const auto& kv : grid) out.push_back(kv.second.s / float(kv.second.n));
  return out;
}

// Local ICP target: neighbouring keyframe clouds at their optimized poses, expressed relative to
// `origin` so float precision holds far from the map origin.
struct Submap {
  int lo = -1, hi = -1, center = -1;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
  std::vector<Eigen::Vector3f> normals;
  std::vector<uint8_t> planar;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
};

class SubmapBuilder {
 public:
  SubmapBuilder(const Trajectory& traj, const Options& o) : traj_(traj), o_(o) {}

  // Returns the submap for segment k, rebuilding only when k drifts away from the current center.
  const Submap* get(int k) {
    const int W = std::max(2, o_.submap_kf);
    if (!sub_ || std::abs(k - sub_->center) > W / 2 || k < sub_->lo || k + 1 > sub_->hi) build(k, W);
    return sub_->cloud->empty() ? nullptr : sub_.get();
  }

  size_t builds() const { return builds_; }
  size_t missingClouds() const { return missing_; }

 private:
  const Cloud3f& keyframeCloud(int i) {
    auto it = cache_.find(i);
    if (it != cache_.end()) return *it->second;
    auto c = std::make_shared<Cloud3f>();
    pcl::PointCloud<pcl::PointXYZI> pc;
    std::string path = (boost::format("%s/%06d/cloud.pcd") % o_.graph % traj_.kf(i).id).str();
    if (fs::exists(path) && pcl::io::loadPCDFile(path, pc) == 0) {
      c->reserve(pc.size());
      for (const auto& p : pc) c->emplace_back(p.x, p.y, p.z);
    } else {
      ++missing_;
    }
    return *cache_.emplace(i, c).first->second;
  }

  void build(int k, int W) {
    ++builds_;
    sub_.reset(new Submap());
    // Grow the window over keyframes that are temporally connected (don't bridge session gaps).
    int lo = k, hi = k + 1;
    while (lo > 0 && k - lo < W && traj_.gap(lo - 1) <= o_.max_gap) --lo;
    while (hi + 1 < (int)traj_.size() && hi - (k + 1) < W && traj_.gap(hi) <= o_.max_gap) ++hi;
    sub_->lo = lo;
    sub_->hi = hi;
    sub_->center = k;
    sub_->origin = traj_.kf(k).p;

    Cloud3f merged;
    for (int i = lo; i <= hi; ++i) {
      const Cloud3f& c = keyframeCloud(i);
      Eigen::Isometry3d T = traj_.pose(i);
      T.translation() -= sub_->origin;
      const Eigen::Isometry3f Tf = T.cast<float>();
      for (const auto& p : c) merged.push_back(Tf * p);
    }
    for (auto it = cache_.begin(); it != cache_.end();)
      it = (it->first < lo - W || it->first > hi + W) ? cache_.erase(it) : std::next(it);

    Cloud3f ds = voxelCentroids(merged, (float)o_.icp_leaf);
    auto& cloud = *sub_->cloud;
    cloud.resize(ds.size());
    for (size_t i = 0; i < ds.size(); ++i) cloud[i].getVector3fMap() = ds[i];
    if (cloud.size() < 50) {
      cloud.clear();
      return;
    }
    sub_->tree.setInputCloud(sub_->cloud);

    // Normals from the 8 nearest neighbours; only near-planar neighbourhoods are used as targets.
    sub_->normals.assign(cloud.size(), Eigen::Vector3f::UnitZ());
    sub_->planar.assign(cloud.size(), 0);
#pragma omp parallel for schedule(dynamic, 256)
    for (int i = 0; i < (int)cloud.size(); ++i) {
      std::vector<int> idx;
      std::vector<float> d2;
      if (sub_->tree.nearestKSearch(cloud[i], 8, idx, d2) < 5) continue;
      const float max_r = 4.0f * float(o_.icp_leaf);
      if (d2.back() > max_r * max_r) continue;
      Eigen::Vector3f mean = Eigen::Vector3f::Zero();
      for (int j : idx) mean += cloud[j].getVector3fMap();
      mean /= float(idx.size());
      Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
      for (int j : idx) {
        Eigen::Vector3f d = cloud[j].getVector3fMap() - mean;
        cov += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
      const Eigen::Vector3f ev = es.eigenvalues();  // ascending
      if (ev.sum() <= 0 || ev(0) / ev.sum() > 0.05f) continue;
      sub_->normals[i] = es.eigenvectors().col(0);
      sub_->planar[i] = 1;
    }
  }

  const Trajectory& traj_;
  const Options& o_;
  std::unique_ptr<Submap> sub_;
  std::unordered_map<int, std::shared_ptr<Cloud3f>> cache_;
  size_t builds_ = 0, missing_ = 0;
};

struct IcpResult {
  bool ok = false;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  double inlier_ratio = 0.0;
  int correspondences = 0;
};

// Point-to-plane Gauss-Newton ICP (left-multiplied correction, Huber weights). Source and target
// are both in the submap's local frame.
static IcpResult icpPointToPlane(const Cloud3f& src, const Submap& sub, const Options& o) {
  IcpResult res;
  typedef Eigen::Matrix<double, 6, 6> Mat6;
  typedef Eigen::Matrix<double, 6, 1> Vec6;
  const double huber = 0.1;

  auto correspond = [&](const Eigen::Vector3f& q, float max_d2, int& idx, float& r) {
    thread_local std::vector<int> ii(1);
    thread_local std::vector<float> dd(1);
    pcl::PointXYZ pq;
    pq.getVector3fMap() = q;
    if (sub.tree.nearestKSearch(pq, 1, ii, dd) < 1 || dd[0] > max_d2 || !sub.planar[ii[0]]) return false;
    idx = ii[0];
    r = sub.normals[idx].dot(q - (*sub.cloud)[idx].getVector3fMap());
    return true;
  };

  for (int it = 0; it < o.icp_max_iter; ++it) {
    const float max_d = float(it < o.icp_max_iter / 2 ? o.icp_max_dist : 0.5 * o.icp_max_dist);
    const Eigen::Isometry3f Tf = res.T.cast<float>();
    Mat6 H = Mat6::Zero();
    Vec6 g = Vec6::Zero();
    int n = 0;
#pragma omp parallel
    {
      Mat6 Hl = Mat6::Zero();
      Vec6 gl = Vec6::Zero();
      int nl = 0;
#pragma omp for nowait
      for (int i = 0; i < (int)src.size(); ++i) {
        Eigen::Vector3f q = Tf * src[i];
        int idx;
        float r;
        if (!correspond(q, max_d * max_d, idx, r)) continue;
        const Eigen::Vector3d nd = sub.normals[idx].cast<double>();
        Vec6 J;
        J.head<3>() = q.cast<double>().cross(nd);
        J.tail<3>() = nd;
        const double w = std::abs(r) < huber ? 1.0 : huber / std::abs(r);
        Hl += w * J * J.transpose();
        gl += w * J * double(r);
        ++nl;
      }
#pragma omp critical
      {
        H += Hl;
        g += gl;
        n += nl;
      }
    }
    if (n < 50) return res;
    H += Mat6::Identity() * (1e-6 * H.trace() + 1e-9);
    const Vec6 dx = -H.ldlt().solve(g);
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    const double ang = dx.head<3>().norm();
    if (ang > 1e-12) step.linear() = Eigen::AngleAxisd(ang, dx.head<3>() / ang).toRotationMatrix();
    step.translation() = dx.tail<3>();
    res.T = step * res.T;
    if (dx.tail<3>().norm() < 1e-4 && ang < 1e-5) break;
  }

  // Quality: share of planar correspondences (within 0.5 m) that sit on the plane (|r| < 10 cm).
  const Eigen::Isometry3f Tf = res.T.cast<float>();
  int valid = 0, inl = 0;
#pragma omp parallel for reduction(+ : valid, inl)
  for (int i = 0; i < (int)src.size(); ++i) {
    int idx;
    float r;
    if (!correspond(Tf * src[i], 0.25f, idx, r)) continue;
    ++valid;
    if (std::abs(r) < 0.1f) ++inl;
  }
  res.correspondences = valid;
  res.inlier_ratio = valid ? double(inl) / valid : 0.0;
  const double ang = Eigen::AngleAxisd(res.T.linear()).angle();
  res.ok = valid >= 50 && res.inlier_ratio >= o.icp_min_inlier_ratio && res.T.translation().norm() < 1.0 &&
           ang < 5.0 * M_PI / 180.0;
  return res;
}

// ---------------------------------------------------------------------------------------------
// Output

// Binary PCD (x y z intensity float32) with fixed-width counts so the header can be rewritten in
// place once the number of points is known.
class PcdWriter {
 public:
  explicit PcdWriter(const std::string& path) : path_(path), ofs_(path, std::ios::binary) {
    if (!ofs_) throw std::runtime_error("cannot write " + path);
    writeHeader();
  }
  void write(const float* xyzi, size_t n) {
    ofs_.write(reinterpret_cast<const char*>(xyzi), n * 4 * sizeof(float));
    n_ += n;
  }
  size_t close() {
    ofs_.seekp(0);
    writeHeader();
    ofs_.close();
    return n_;
  }

 private:
  void writeHeader() {
    const std::string n = (boost::format("%012d") % n_).str();
    ofs_ << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << n << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << n << "\n"
         << "DATA binary\n";
  }
  std::string path_;
  std::ofstream ofs_;
  size_t n_ = 0;
};

// Buffers points per XY tile and appends them to <dir>/<ix>_<iy>.bin when the budget is reached.
class TileSpill {
 public:
  TileSpill(const std::string& dir, double tile, size_t budget_bytes)
      : dir_(dir), tile_(tile), budget_(budget_bytes) {
    fs::create_directories(dir_);
  }
  ~TileSpill() { flush(); }

  void add(const Eigen::Vector3d& p, float intensity) {
    const int32_t ix = (int32_t)std::floor(p.x() / tile_), iy = (int32_t)std::floor(p.y() / tile_);
    const uint64_t key = (uint64_t(uint32_t(ix)) << 32) | uint32_t(iy);
    auto& b = buf_[key];
    b.push_back((float)p.x());
    b.push_back((float)p.y());
    b.push_back((float)p.z());
    b.push_back(intensity);
    buffered_ += 4 * sizeof(float);
    ++total_;
    if (buffered_ >= budget_) flush();
  }

  void flush() {
    for (auto& kv : buf_) {
      if (kv.second.empty()) continue;
      const int32_t ix = int32_t(kv.first >> 32), iy = int32_t(kv.first & 0xFFFFFFFF);
      const std::string path = (boost::format("%s/%d_%d.bin") % dir_ % ix % iy).str();
      FILE* f = std::fopen(path.c_str(), "ab");
      if (!f) throw std::runtime_error("cannot append " + path);
      std::fwrite(kv.second.data(), sizeof(float), kv.second.size(), f);
      std::fclose(f);
    }
    buf_.clear();
    buffered_ = 0;
  }

  size_t total() const { return total_; }

 private:
  std::string dir_;
  double tile_;
  size_t budget_, buffered_ = 0, total_ = 0;
  std::unordered_map<uint64_t, std::vector<float>> buf_;
};

// Turns every spill file into a voxel-deduplicated tile PCD (and optionally one merged PCD).
static size_t finalizeTiles(const std::string& spill_dir, const Options& o) {
  const std::string tile_dir = o.out + "/tiles";
  fs::create_directories(tile_dir);
  std::unique_ptr<PcdWriter> merged;
  if (o.merged) merged.reset(new PcdWriter(o.out + "/dense_map.pcd"));

  std::vector<fs::path> files;
  for (fs::directory_iterator it(spill_dir), end; it != end; ++it)
    if (it->path().extension() == ".bin") files.push_back(it->path());
  std::sort(files.begin(), files.end());

  size_t total = 0;
  const size_t chunk = 1 << 20;  // points per read
  std::vector<float> rec(chunk * 4);
  for (size_t fi = 0; fi < files.size(); ++fi) {
    const std::string stem = files[fi].stem().string();
    int ix = 0, iy = 0;
    if (std::sscanf(stem.c_str(), "%d_%d", &ix, &iy) != 2) continue;
    FILE* f = std::fopen(files[fi].string().c_str(), "rb");
    if (!f) continue;
    PcdWriter tile((boost::format("%s/tile_%d_%d.pcd") % tile_dir % ix % iy).str());

    if (o.voxel <= 0.0) {
      size_t n;
      while ((n = std::fread(rec.data(), 4 * sizeof(float), chunk, f)) > 0) {
        tile.write(rec.data(), n);
        if (merged) merged->write(rec.data(), n);
      }
    } else {
      // Centroid per voxel. Sums are kept relative to the voxel corner so float stays precise.
      struct Acc { float x = 0, y = 0, z = 0, i = 0; uint32_t n = 0; };
      std::unordered_map<uint64_t, Acc> grid;
      const double v = o.voxel, ox = ix * o.tile, oy = iy * o.tile;
      size_t n;
      while ((n = std::fread(rec.data(), 4 * sizeof(float), chunk, f)) > 0) {
        for (size_t j = 0; j < n; ++j) {
          const float* r = &rec[4 * j];
          const int64_t gx = (int64_t)std::floor((r[0] - ox) / v), gy = (int64_t)std::floor((r[1] - oy) / v),
                        gz = (int64_t)std::floor(r[2] / v);
          Acc& a = grid[packKey(gx, gy, gz)];
          a.x += float(r[0] - (ox + gx * v));
          a.y += float(r[1] - (oy + gy * v));
          a.z += float(r[2] - gz * v);
          a.i += r[3];
          ++a.n;
        }
      }
      std::vector<float> outv;
      outv.reserve(grid.size() * 4);
      const int64_t off = 1 << 20;
      for (const auto& kv : grid) {
        const int64_t gx = int64_t(kv.first & 0x1FFFFF) - off, gy = int64_t((kv.first >> 21) & 0x1FFFFF) - off,
                      gz = int64_t((kv.first >> 42) & 0x1FFFFF) - off;
        const Acc& a = kv.second;
        outv.push_back(float(ox + gx * v + a.x / a.n));
        outv.push_back(float(oy + gy * v + a.y / a.n));
        outv.push_back(float(gz * v + a.z / a.n));
        outv.push_back(a.i / a.n);
      }
      tile.write(outv.data(), grid.size());
      if (merged) merged->write(outv.data(), grid.size());
    }
    std::fclose(f);
    total += tile.close();
    if (!o.keep_spill) fs::remove(files[fi]);
    std::cout << "\rtiles " << fi + 1 << "/" << files.size() << std::flush;
  }
  std::cout << "\n";
  if (merged) merged->close();
  if (!o.keep_spill) fs::remove_all(spill_dir);
  return total;
}

// ---------------------------------------------------------------------------------------------
// Scan parsing

struct Field {
  int offset = -1;
  uint8_t type = 0;
  double read(const uint8_t* p) const {
    switch (type) {
      case sensor_msgs::PointField::INT8: { int8_t v; std::memcpy(&v, p + offset, 1); return v; }
      case sensor_msgs::PointField::UINT8: { uint8_t v; std::memcpy(&v, p + offset, 1); return v; }
      case sensor_msgs::PointField::INT16: { int16_t v; std::memcpy(&v, p + offset, 2); return v; }
      case sensor_msgs::PointField::UINT16: { uint16_t v; std::memcpy(&v, p + offset, 2); return v; }
      case sensor_msgs::PointField::INT32: { int32_t v; std::memcpy(&v, p + offset, 4); return v; }
      case sensor_msgs::PointField::UINT32: { uint32_t v; std::memcpy(&v, p + offset, 4); return v; }
      case sensor_msgs::PointField::FLOAT32: { float v; std::memcpy(&v, p + offset, 4); return v; }
      case sensor_msgs::PointField::FLOAT64: { double v; std::memcpy(&v, p + offset, 8); return v; }
    }
    return 0.0;
  }
};

struct Layout {
  Field x, y, z, intensity, time;
  bool ok() const { return x.offset >= 0 && y.offset >= 0 && z.offset >= 0; }
};

static Layout parseLayout(const sensor_msgs::PointCloud2& msg) {
  Layout l;
  for (const auto& f : msg.fields) {
    Field fd;
    fd.offset = f.offset;
    fd.type = f.datatype;
    if (f.name == "x") l.x = fd;
    else if (f.name == "y") l.y = fd;
    else if (f.name == "z") l.z = fd;
    else if (f.name == "intensity" || f.name == "reflectivity") l.intensity = fd;
    else if ((f.name == "timestamp" || f.name == "time" || f.name == "t" || f.name == "offset_time") &&
             l.time.offset < 0)
      l.time = fd;
  }
  return l;
}

// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv) {
  Options o;
  try {
    if (!parseArgs(argc, argv, o)) { usage(); return 0; }
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n\n";
    usage();
    return 1;
  }
  if (o.threads > 0) omp_set_num_threads(o.threads);
  ros::Time::init();
  const auto t_start = std::chrono::steady_clock::now();

  // Trajectory
  std::string src = o.pose_source;
  if (src == "auto") src = fs::exists(o.graph + "/factor_graph.yaml") ? "yaml" : "data";
  std::vector<Keyframe> kfs;
  try {
    kfs = src == "yaml" ? loadYamlPoses(o.graph) : loadDataPoses(o.graph);
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
  Trajectory traj(std::move(kfs));
  if (traj.size() < 2) {
    std::cerr << "ERROR: need at least 2 timestamped keyframes in " << o.graph << " (pose source " << src << ")\n";
    return 1;
  }
  int sessions = 1;
  for (int k = 0; k + 1 < (int)traj.size(); ++k)
    if (traj.gap(k) > o.max_gap) ++sessions;
  std::cout << (boost::format("Trajectory: %d keyframes from %s (%s), %.1f s span, %d segment(s) split by gaps > %.0f s\n") %
                traj.size() % o.graph % src % (traj.kf(traj.size() - 1).t - traj.kf(0).t) % sessions % o.max_gap);

  fs::create_directories(o.out);
  const std::string spill_dir = o.out + "/spill";
  if (fs::exists(spill_dir)) fs::remove_all(spill_dir);
  TileSpill spill(spill_dir, o.tile, size_t(512) << 20);
  SubmapBuilder submaps(traj, o);

  const Eigen::Matrix3d R_level = (Eigen::AngleAxisd(o.rz, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(o.ry, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(o.rx, Eigen::Vector3d::UnitX())).toRotationMatrix();

  GyroTrack gyro;
  if (!o.imu_topic.empty()) {
    for (const std::string& bag_path : o.bags) {
      try {
        rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(o.imu_topic));
        for (const rosbag::MessageInstance& m : view) {
          sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
          if (!imu) continue;
          const auto& w = imu->angular_velocity;
          gyro.add(imu->header.stamp.toSec() + o.stamp_offset, R_level * Eigen::Vector3d(w.x, w.y, w.z));
        }
      } catch (const std::exception& e) {
        std::cerr << "ERROR: cannot read " << o.imu_topic << " from " << bag_path << ": " << e.what() << "\n";
        return 1;
      }
    }
    gyro.finalize();
    if (gyro.empty())
      std::cout << "WARN: no " << o.imu_topic << " messages; rotation between keyframes is slerped\n";
    else
      std::cout << "Gyro: " << gyro.size() << " samples from " << o.imu_topic << "\n";
  }
  if (!gyro.empty()) traj.setGyro(&gyro, o.gyro_max_resid);

  std::ofstream poses_out(o.out + "/scan_poses.txt");
  poses_out << std::setprecision(17);
  // pose_time=scan_start: each pose is the sensor pose at the scan's header stamp. Files without this
  // line predate the t0 fix and their poses sit at (stamp + first-point time offset) instead.
  poses_out << "# liorf_densify_map pose_time=scan_start  columns: stamp x y z qx qy qz qw\n";

  struct Stats {
    size_t scans = 0, window = 0, every = 0, outside = 0, gap = 0, empty = 0, icp_ok = 0, icp_fail = 0,
           no_submap = 0, stationary = 0, kept = 0;
    double inlier_sum = 0;
  } st;
  bool warned_no_time = false;
  double time_scale = -1.0;  // per-point time field units -> seconds, detected from the first scan

  for (const std::string& bag_path : o.bags) {
    rosbag::Bag bag;
    try {
      bag.open(bag_path, rosbag::bagmode::Read);
    } catch (const std::exception& e) {
      std::cerr << "ERROR: cannot open " << bag_path << ": " << e.what() << "\n";
      return 1;
    }
    rosbag::View view(bag, rosbag::TopicQuery(o.topic));
    std::cout << "Bag " << bag_path << ": " << view.size() << " messages on " << o.topic << "\n";
    if (view.size() == 0) continue;

    Eigen::Isometry3d C = Eigen::Isometry3d::Identity();  // carried-over ICP correction (map frame)
    Eigen::Isometry3d static_anchor = Eigen::Isometry3d::Identity();
    int static_count = 0;
    bool have_anchor = false;
    double first_stamp = -1.0;
    size_t idx_in_bag = 0;

    for (const rosbag::MessageInstance& m : view) {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (!msg) continue;
      ++st.scans;
      const double stamp = msg->header.stamp.toSec() + o.stamp_offset;
      if (first_stamp < 0) first_stamp = stamp;
      if (stamp - first_stamp < o.start) { ++st.window; continue; }
      if (o.duration > 0 && stamp - first_stamp > o.start + o.duration) break;
      if (idx_in_bag++ % o.every) { ++st.every; continue; }

      const int k0 = traj.segment(stamp);
      if (k0 < 0) { ++st.outside; C.setIdentity(); continue; }
      if (traj.gap(k0) > o.max_gap) { ++st.gap; C.setIdentity(); continue; }

      const Layout L = parseLayout(*msg);
      if (!L.ok()) { std::cerr << "ERROR: cloud has no x/y/z fields\n"; return 1; }
      const size_t n_pts = size_t(msg->width) * msg->height;
      const uint8_t* data = msg->data.data();
      if (L.time.offset < 0 && !warned_no_time) {
        std::cout << "WARN: no per-point time field; scans are not deskewed\n";
        warned_no_time = true;
      }
      // Time zero is the scan's earliest valid point, not point 0: an organized Ouster cloud starts
      // at a fixed azimuth, so point 0 can be ~one sweep (~97 ms) into the scan. Using it shifted
      // every point time, which the ICP refinement then silently absorbed into scan_poses.txt.
      double t0 = 0.0;
      if (L.time.offset >= 0 && n_pts) {
        t0 = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < n_pts; ++i) {
          const uint8_t* p = data + i * msg->point_step;
          if (!std::isfinite(L.x.read(p)) || !std::isfinite(L.y.read(p)) || !std::isfinite(L.z.read(p))) continue;
          t0 = std::min(t0, L.time.read(p));
        }
        if (!std::isfinite(t0)) t0 = L.time.read(data);
      }
      if (time_scale < 0 && L.time.offset >= 0 && n_pts > 100) {
        double tmin = t0, tmax = t0;
        for (size_t i = 0; i < n_pts; ++i) {
          const double v = L.time.read(data + i * msg->point_step);
          tmin = std::min(tmin, v);
          tmax = std::max(tmax, v);
        }
        const double span = tmax - tmin;  // Livox: ~1e8 ns for a 10 Hz frame
        if (span > 0) {
          time_scale = span > 1e5 ? 1e-9 : span > 1e2 ? 1e-6 : 1.0;
          std::cout << (boost::format("Per-point time field: span %.4g raw units -> %.3f s per scan\n") % span %
                        (span * time_scale));
        }
      }
      const bool have_time = L.time.offset >= 0 && time_scale > 0;

      const Submap* sub = o.refine ? submaps.get(k0) : nullptr;
      if (o.refine && !sub) { ++st.no_submap; continue; }
      const Eigen::Vector3d origin = sub ? sub->origin : traj.kf(k0).p;

      // Per-point pose: interpolated trajectory, then the carried-over correction.
      Cloud3f local;
      std::vector<float> intens;
      local.reserve(n_pts);
      intens.reserve(n_pts);
      int k = k0;
      for (size_t i = 0; i < n_pts; ++i) {
        const uint8_t* p = data + i * msg->point_step;
        const Eigen::Vector3d raw(L.x.read(p), L.y.read(p), L.z.read(p));
        if (!raw.allFinite()) continue;
        const double r = raw.norm();
        if (r < o.min_range || r > o.max_range) continue;
        const double t = have_time ? stamp + (L.time.read(p) - t0) * time_scale : stamp;
        k = traj.segment(t, k);
        if (k < 0 || traj.gap(k) > o.max_gap) { k = k0; continue; }
        const Eigen::Vector3d pm = C * (traj.interp(k, t) * (R_level * raw));
        local.push_back((pm - origin).cast<float>());
        intens.push_back(L.intensity.offset >= 0 ? (float)L.intensity.read(p) : 0.0f);
      }
      if (local.size() < 100) { ++st.empty; continue; }

      if (sub) {
        const Cloud3f ds = voxelCentroids(local, (float)o.icp_leaf);
        const IcpResult icp = icpPointToPlane(ds, *sub, o);
        if (!icp.ok) { ++st.icp_fail; continue; }
        ++st.icp_ok;
        st.inlier_sum += icp.inlier_ratio;
        const Eigen::Isometry3f Tf = icp.T.cast<float>();
        for (auto& q : local) q = Tf * q;
        Eigen::Isometry3d to = Eigen::Isometry3d::Identity();
        to.translation() = origin;
        C = to * icp.T * to.inverse() * C;
      }

      // Stationary limiter: the MID360 pattern fills in over ~2 s; past that a stopped robot only
      // adds passing pedestrians/cars.
      if (o.max_static_scans > 0) {
        const Eigen::Isometry3d P = C * traj.interp(k0, stamp);
        const Eigen::Isometry3d d = have_anchor ? static_anchor.inverse() * P : Eigen::Isometry3d::Identity();
        if (have_anchor && d.translation().norm() < 0.05 && Eigen::AngleAxisd(d.linear()).angle() < 0.02) {
          if (++static_count > o.max_static_scans) { ++st.stationary; continue; }
        } else {
          static_anchor = P;
          have_anchor = true;
          static_count = 0;
        }
      }

      for (size_t i = 0; i < local.size(); ++i) spill.add(origin + local[i].cast<double>(), intens[i]);
      ++st.kept;
      {
        const Eigen::Isometry3d P = C * traj.interp(k0, stamp);
        const Eigen::Quaterniond q(P.linear());
        poses_out << stamp << " " << P.translation().x() << " " << P.translation().y() << " " << P.translation().z()
                  << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
      }

      if (st.kept % 200 == 0) {
        const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        std::cout << (boost::format("\rscans %d kept %d | icp fail %d | mean inliers %.2f | %.1fM pts | %.0f s") %
                      st.scans % st.kept % st.icp_fail % (st.icp_ok ? st.inlier_sum / st.icp_ok : 0.0) %
                      (spill.total() / 1e6) % el)
                  << std::flush;
      }
    }
    std::cout << "\n";
  }
  spill.flush();

  std::cout << (boost::format(
                    "Scans: %d read, %d kept | skipped: %d outside trajectory, %d in gaps, %d window, %d --every, "
                    "%d stationary, %d empty, %d ICP rejected, %d no submap\n") %
                st.scans % st.kept % st.outside % st.gap % st.window % st.every % st.stationary % st.empty %
                st.icp_fail % st.no_submap);
  if (o.refine)
    std::cout << (boost::format("ICP: mean inlier ratio %.3f over %d aligned scans, %d submap builds, %d missing keyframe clouds\n") %
                  (st.icp_ok ? st.inlier_sum / st.icp_ok : 0.0) % st.icp_ok % submaps.builds() % submaps.missingClouds());
  if (!gyro.empty()) {
    size_t used, fallback;
    double resid;
    traj.gyroStats(used, fallback, resid);
    std::cout << (boost::format("Gyro: %d segments gyro-interpolated (mean keyframe mismatch %.4f rad), %d slerp fallback\n") %
                  used % resid % fallback);
  }
  poses_out.close();
  if (st.kept == 0) {
    std::cerr << "ERROR: no scans kept. Check that the bag matches the graph (stamps), --topic, and --max-gap"
                 " (and that graph/NNNNNN/cloud.pcd exist, or use --no-refine).\n";
    return 1;
  }
  std::cout << "Raw points: " << spill.total() << ". Writing tiles (voxel " << o.voxel << " m)...\n";
  const size_t written = finalizeTiles(spill_dir, o);
  const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  std::cout << (boost::format("Done: %d points in %s/tiles%s (%.0f s)\n") % written % o.out %
                (o.merged ? " and dense_map.pcd" : "") % el);
  return 0;
}
