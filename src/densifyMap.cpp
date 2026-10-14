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
//      from the neighbouring keyframe clouds (graph/NNNNNN/cloud.pcd) at their optimized poses,
//      starting from the interpolated pose. Interpolation alone is off by up to the keyframe spacing
//      when the robot stops or turns between keyframes. (Corrections used to be carried from scan to
//      scan; that drifted along flat facades into metre-scale double walls, and kept scans dependent on
//      each other. Now every scan is independent, which also lets them run in parallel.)
//   3. Points are spilled to per-tile files on disk, then each tile is voxel-deduplicated
//      (centroid per voxel) and written as a PCD, so RAM is bounded by the tiles in flight
//      (--tile-workers within --tile-mem-gb), not the map.
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

#include <malloc.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <cstdlib>
#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <thread>
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
  // Drop scans in keyframe segments whose rotation disagrees with the gyro by more than this (rad): the
  // keyframes themselves are wrong there (e.g. the first seconds after a failed-ICP session attach, where
  // liorf starts from a GPS+IMU heading and swings onto the map) and any placement duplicates objects.
  double drop_rot_mismatch = 0.1;        // 0 = keep them (slerp between the keyframes)
  // Keyframe segments skipped at the start of each session (trajectory start or after a gap > max_gap).
  // A session's first keyframe is the attach keyframe: when the attach ICP fails liorf forces it from
  // GPS+IMU and it can be metres off while its rotation still agrees with the gyro.
  int skip_session_start = 1;
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
  // Cap on a scan's ICP correction, measured at the scan's sensor position (tighter than the 1 m / 5 deg
  // ICP step limit).
  double max_correction = 0.5;      // m; 0 disables
  double max_correction_deg = 3.0;  // deg; 0 disables
  double max_gap = 60.0;  // keyframe gaps longer than this (s) are treated as "no trajectory"
  double start = 0.0;     // s from the first scan of each bag
  double duration = -1.0;
  int every = 1;
  int max_static_scans = 20;  // consecutive scans kept while stationary (0 = unlimited)
  bool merged = false;
  bool keep_spill = false;
  int threads = 0;
  int tile_workers = 4;         // tiles in flight (read -> voxelized by the workers -> written, in order)
  double tile_mem_gb = 12.0;    // estimated RAM budget for the tiles in flight
  bool finalize_only = false;   // build tiles from an existing <out>/spill (e.g. after an interrupted run)
  std::vector<std::pair<int, int>> only_tiles;  // spill only points in these tiles (re-create lost tiles)
  // Free-space (ray-casting) removal of dynamic objects from an existing dense map (--fs-ref).
  std::string fs_ref;           // dense dir whose tiles/ are filtered (needs --only-tiles); "" = off
  double fs_voxel = 0.2;        // evidence voxel (m)
  int fs_stride = 4;            // cast every n-th point of a scan
  double fs_margin = 0.4;       // stop rays this far (m) before their endpoint
  int fs_min_free = 5;          // dynamic if seen through by >= this many scans ...
  double fs_ratio = 3.0;        // ... and by >= ratio x the scans that hit it
  double fs_min_angle = 20.0;   // a ray only "sees through" a flat voxel (wall, ground) crossing it at >= this (deg)
  std::string gyro_cache = "auto";  // per-bag raw gyro cache dir; auto = ~/.cache/liorf_densify_gyro, none = off
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
  --max-correction <m>      drop the scan when its ICP correction would move it more than this from the
                            keyframe trajectory (default 0.5, 0 = no cap)
  --max-correction-deg <d>  same, for its rotation (default 3, 0 = no cap)
  --drop-rot-mismatch <rad> drop scans in keyframe segments whose rotation disagrees with the gyro by
                            more than this (default 0.1; 0 = keep them, slerp between the keyframes)
  --skip-session-start <n>  skip scans in the first n keyframe segments of each session, where a forced
                            (failed-ICP) attach keyframe can be metres off (default 1, 0 = keep)
Output
  --voxel <m>               dedup voxel, centroid per voxel; 0 keeps every point (default 0.03)
  --tile <m>                tile size (default 50)
  --merged                  also write <out>/dense_map.pcd
  --keep-spill              keep the raw per-tile spill files
  --threads <n>             worker threads for the scan phase (default: all cores but one). Disk I/O
                            stays sequential: one bag reader and one spill writer; only the per-scan CPU
                            work runs in parallel
  --tile-workers <n>        tiles in flight (default 4)
  --tile-mem-gb <g>         RAM budget for the tiles in flight, estimated from their spill size
                            (default 12); a larger tile waits until earlier ones are written
  --finalize-only           only build tiles from an existing <out>/spill (resume a run interrupted in the
                            tile phase; tiles already in <out>/tiles are kept). A run interrupted in the
                            scan phase has an incomplete spill: rerun it instead. With --merged,
                            dense_map.pcd is then rebuilt from all tiles in <out>/tiles
  --only-tiles <ix_iy,...>  process scans as usual but spill only points falling in these tiles
                            (re-create specific tiles, e.g. 23_3,24_-1). With --merged, dense_map.pcd
                            is rebuilt from all tiles in <out>/tiles
  --gyro-cache <dir|none>   cache each bag's raw gyro (default ~/.cache/liorf_densify_gyro): reading
                            the IMU topic otherwise means reading the whole bag once more
Free-space removal of dynamic objects -- EXPERIMENTAL (carves foliage and some wall edges; not used for
production maps, use mapping_tools/sor_tiles.py). Replaces writing a dense map:
  --fs-ref <dense_dir>      filter <dense_dir>/tiles (the --only-tiles ones) by ray casting every kept scan:
                            voxels that scans see through are free; a voxel is dynamic when >= --fs-min-free
                            scans saw through it and >= --fs-ratio x the scans that hit it. Writes
                            <out>/tiles (filtered) and <out>/removed. Use the same poses/settings as the map
  --fs-voxel <m>            evidence voxel (default 0.2)       --fs-stride <n>   rays: every n-th point (4)
  --fs-margin <m>           ray stops this short of its endpoint (0.4)
  --fs-min-free <n>         (default 5)                         --fs-ratio <r>    (default 3.0)
  --fs-min-angle <deg>      rays skimming a flat voxel (wall, ground) at less than this don't count as seeing
                            through it (default 20): avoids carving surfaces seen at grazing angles.
                            --out must differ from --fs-ref
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
    else if (k == "--drop-rot-mismatch") { need(i, 1); o.drop_rot_mismatch = std::stod(a[++i]); }
    else if (k == "--skip-session-start") { need(i, 1); o.skip_session_start = std::stoi(a[++i]); }
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
    else if (k == "--max-correction") { need(i, 1); o.max_correction = std::stod(a[++i]); }
    else if (k == "--max-correction-deg") { need(i, 1); o.max_correction_deg = std::stod(a[++i]); }
    else if (k == "--max-gap") { need(i, 1); o.max_gap = std::stod(a[++i]); }
    else if (k == "--start") { need(i, 1); o.start = std::stod(a[++i]); }
    else if (k == "--duration") { need(i, 1); o.duration = std::stod(a[++i]); }
    else if (k == "--every") { need(i, 1); o.every = std::max(1, std::stoi(a[++i])); }
    else if (k == "--max-static-scans") { need(i, 1); o.max_static_scans = std::stoi(a[++i]); }
    else if (k == "--merged") { o.merged = true; }
    else if (k == "--keep-spill") { o.keep_spill = true; }
    else if (k == "--threads") { need(i, 1); o.threads = std::stoi(a[++i]); }
    else if (k == "--tile-workers") { need(i, 1); o.tile_workers = std::stoi(a[++i]); }
    else if (k == "--tile-mem-gb") { need(i, 1); o.tile_mem_gb = std::stod(a[++i]); }
    else if (k == "--finalize-only") { o.finalize_only = true; }
    else if (k == "--fs-ref") { need(i, 1); o.fs_ref = a[++i]; }
    else if (k == "--fs-voxel") { need(i, 1); o.fs_voxel = std::stod(a[++i]); }
    else if (k == "--fs-stride") { need(i, 1); o.fs_stride = std::max(1, std::stoi(a[++i])); }
    else if (k == "--fs-margin") { need(i, 1); o.fs_margin = std::stod(a[++i]); }
    else if (k == "--fs-min-free") { need(i, 1); o.fs_min_free = std::stoi(a[++i]); }
    else if (k == "--fs-ratio") { need(i, 1); o.fs_ratio = std::stod(a[++i]); }
    else if (k == "--fs-min-angle") { need(i, 1); o.fs_min_angle = std::stod(a[++i]); }
    else if (k == "--only-tiles") {
      need(i, 1);
      std::stringstream ss(a[++i]);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        int tx, ty;
        if (std::sscanf(tok.c_str(), "%d_%d", &tx, &ty) != 2) throw std::runtime_error("bad --only-tiles entry " + tok);
        o.only_tiles.emplace_back(tx, ty);
      }
    }
    else if (k == "--gyro-cache") { need(i, 1); o.gyro_cache = a[++i]; }
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
    seg_ = SegRot();
    gyro_hint_ = 0;
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

  // Rotation disagreement (rad) between the keyframes of segment k and the gyro; 0 if no gyro data.
  double rotMismatch(int k) const {
    const SegRot& s = segRot(k);
    return s.measured ? s.log_e.norm() : 0.0;
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
    bool measured = false;  // gyro covered the segment, so log_e is meaningful
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
      seg_.measured = true;
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
    if (!dir_.empty()) fs::create_directories(dir_);  // "" = never written (free-space mode)
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

// Writes <out>/dense_map.pcd by concatenating every tile in <out>/tiles (sequential read + write).
static size_t mergeAllTiles(const Options& o) {
  std::vector<fs::path> tiles;
  for (fs::directory_iterator it(o.out + "/tiles"), end; it != end; ++it)
    if (it->path().extension() == ".pcd") tiles.push_back(it->path());
  std::sort(tiles.begin(), tiles.end());
  PcdWriter merged(o.out + "/dense_map.pcd");
  std::vector<float> buf;
  for (const fs::path& tp : tiles) {
    std::ifstream in(tp.string(), std::ios::binary);
    std::string line;
    while (std::getline(in, line) && line.rfind("DATA", 0) != 0) {}
    const std::streampos start = in.tellg();
    in.seekg(0, std::ios::end);
    const size_t bytes = size_t(in.tellg() - start);
    in.seekg(start);
    buf.resize(bytes / sizeof(float));
    in.read(reinterpret_cast<char*>(buf.data()), bytes);
    merged.write(buf.data(), buf.size() / 4);
  }
  return merged.close();
}

// Fixed worker pool; each task gets the id of the worker running it (for per-worker state).
class Pool {
 public:
  explicit Pool(int n) {
    for (int i = 0; i < n; ++i) th_.emplace_back([this, i] { loop(i); });
  }
  ~Pool() {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : th_) t.join();
  }
  template <class F>
  auto submit(F f) -> std::future<decltype(f(0))> {
    using R = decltype(f(0));
    auto task = std::make_shared<std::packaged_task<R(int)>>(std::move(f));
    std::future<R> fut = task->get_future();
    {
      std::lock_guard<std::mutex> lk(m_);
      q_.emplace_back([task](int w) { (*task)(w); });
    }
    cv_.notify_one();
    return fut;
  }
  int size() const { return (int)th_.size(); }

 private:
  void loop(int w) {
    omp_set_num_threads(1);  // parallelism comes from the pool; keeps ICP deterministic too
    for (;;) {
      std::function<void(int)> job;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [this] { return stop_ || !q_.empty(); });
        if (stop_ && q_.empty()) return;
        job = std::move(q_.front());
        q_.pop_front();
      }
      job(w);
    }
  }
  std::vector<std::thread> th_;
  std::deque<std::function<void(int)>> q_;
  std::mutex m_;
  std::condition_variable cv_;
  bool stop_ = false;
};

// Turns the spill files into voxel-deduplicated tile PCDs (and optionally one merged PCD). Disk access
// stays sequential: this thread reads one tile's spill files at a time and writes finished tiles in
// order; only the voxelization of up to tile_workers tiles runs on the pool in between.
static size_t finalizeTiles(const std::string& spill_dir, const Options& o, bool write_merged) {
  // Own pool of tile_workers threads: the big per-tile voxel grids then live in at most that many malloc
  // arenas (+ malloc_trim after each tile). Voxelizing on all ~20 scan workers left ~28 GB of fragmented
  // free heap and stalled a run in swap; capping arenas globally instead slowed the scan phase ~5x.
  Pool pool(std::max(1, o.tile_workers));
  const std::string tile_dir = o.out + "/tiles";
  fs::create_directories(tile_dir);
  std::unique_ptr<PcdWriter> merged;
  if (write_merged) merged.reset(new PcdWriter(o.out + "/dense_map.pcd"));

  std::map<std::string, std::vector<fs::path>> groups;  // "<ix>_<iy>" -> spill files
  for (fs::recursive_directory_iterator it(spill_dir), end; it != end; ++it)
    if (fs::is_regular_file(it->path()) && it->path().extension() == ".bin")
      groups[it->path().stem().string()].push_back(it->path());

  struct TileOut { int ix = 0, iy = 0; std::vector<float> pts; size_t n = 0; };
  struct InFlight { std::future<TileOut> fut; std::vector<fs::path> files; double mem_gb = 0; };
  std::deque<InFlight> inflight;
  size_t total = 0, done = 0;
  double inflight_gb = 0.0;
  auto write_front = [&]() {
    InFlight f = std::move(inflight.front());
    inflight.pop_front();
    TileOut t = f.fut.get();
    PcdWriter tile((boost::format("%s/tile_%d_%d.pcd") % tile_dir % t.ix % t.iy).str());
    tile.write(t.pts.data(), t.n);
    if (merged) merged->write(t.pts.data(), t.n);
    total += tile.close();
    // The spill is only removed once its tile is safely written (an interrupted run loses nothing).
    if (!o.keep_spill)
      for (const fs::path& fp : f.files) fs::remove(fp);
    inflight_gb -= f.mem_gb;
    std::vector<float>().swap(t.pts);
    malloc_trim(0);  // hand freed voxel-grid memory back instead of keeping it fragmented in the heap
    std::cout << "\rtiles " << ++done << "/" << groups.size() << std::flush;
  };
  const size_t max_inflight = (size_t)std::max(1, o.tile_workers);
  for (const auto& g : groups) {
    int ix = 0, iy = 0;
    if (std::sscanf(g.first.c_str(), "%d_%d", &ix, &iy) != 2) continue;
    // Estimated peak RAM of this tile: raw points (16 B) + voxel grid (~72 B per point at 1 cm, where
    // almost every point is its own voxel) + output (16 B).
    size_t spill_bytes = 0;
    for (const fs::path& fp : g.second) spill_bytes += fs::file_size(fp);
    const double mem_gb = spill_bytes / 16.0 * (16 + 72 + 16) / 1e9;
    while (!inflight.empty() && (inflight.size() >= max_inflight || inflight_gb + mem_gb > o.tile_mem_gb))
      write_front();
    auto raw = std::make_shared<std::vector<float>>();
    {
      size_t bytes = 0;
      for (const fs::path& fp : g.second) bytes += fs::file_size(fp);
      raw->resize(bytes / sizeof(float));
      size_t off = 0;
      for (const fs::path& fp : g.second) {
        FILE* f = std::fopen(fp.string().c_str(), "rb");
        if (!f) throw std::runtime_error("cannot read " + fp.string());
        off += std::fread(raw->data() + off, sizeof(float), raw->size() - off, f);
        std::fclose(f);
      }
      raw->resize(off - off % 4);
    }
    InFlight fl;
    fl.files = g.second;
    fl.mem_gb = mem_gb;
    inflight_gb += mem_gb;
    fl.fut = pool.submit([raw, ix, iy, &o](int) {
      TileOut t;
      t.ix = ix;
      t.iy = iy;
      const size_t n = raw->size() / 4;
      if (o.voxel <= 0.0) {
        t.pts = std::move(*raw);
        t.n = n;
        return t;
      }
      // Centroid per voxel. Sums are kept relative to the voxel corner so float stays precise.
      struct Acc { float x = 0, y = 0, z = 0, i = 0; uint32_t n = 0; };
      std::unordered_map<uint64_t, Acc> grid;
      const double v = o.voxel, ox = ix * o.tile, oy = iy * o.tile;
      for (size_t j = 0; j < n; ++j) {
        const float* r = &(*raw)[4 * j];
        const int64_t gx = (int64_t)std::floor((r[0] - ox) / v), gy = (int64_t)std::floor((r[1] - oy) / v),
                      gz = (int64_t)std::floor(r[2] / v);
        Acc& a = grid[packKey(gx, gy, gz)];
        a.x += float(r[0] - (ox + gx * v));
        a.y += float(r[1] - (oy + gy * v));
        a.z += float(r[2] - gz * v);
        a.i += r[3];
        ++a.n;
      }
      std::vector<float>().swap(*raw);
      t.pts.reserve(grid.size() * 4);
      const int64_t off = 1 << 20;
      for (const auto& kv : grid) {
        const int64_t gx = int64_t(kv.first & 0x1FFFFF) - off, gy = int64_t((kv.first >> 21) & 0x1FFFFF) - off,
                      gz = int64_t((kv.first >> 42) & 0x1FFFFF) - off;
        const Acc& a = kv.second;
        t.pts.push_back(float(ox + gx * v + a.x / a.n));
        t.pts.push_back(float(oy + gy * v + a.y / a.n));
        t.pts.push_back(float(gz * v + a.z / a.n));
        t.pts.push_back(a.i / a.n);
      }
      t.n = grid.size();
      return t;
    });
    inflight.push_back(std::move(fl));
  }
  while (!inflight.empty()) write_front();
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

// Free-space evidence over the voxels of an existing dense map (read-only during the scan pass).
struct FsMap {
  double v = 0.2;
  std::unordered_map<uint64_t, uint32_t> idx;  // voxel key -> index
  std::vector<uint32_t> hit, fre;              // scans that ended a ray in / saw through each voxel
  std::vector<Eigen::Vector3f> normal;         // surface normal of flat voxels, zero if not flat
  Eigen::Vector3d lo, hi;                      // bounds of the filtered tiles
  uint64_t key(const Eigen::Vector3d& p) const {
    return packKey((int64_t)std::floor(p.x() / v), (int64_t)std::floor(p.y() / v), (int64_t)std::floor(p.z() / v));
  }
  bool near(const Eigen::Vector3d& p, double r) const {
    return p.x() > lo.x() - r && p.x() < hi.x() + r && p.y() > lo.y() - r && p.y() < hi.y() + r;
  }
};

// Voxel indices one scan hit (ray endpoints) and saw through (traversed, up to margin before the end),
// each listed once per scan.
static void castScan(const FsMap& M, const Eigen::Vector3d& sensor, const Eigen::Vector3d& origin,
                     const Cloud3f& local, const Options& o, std::vector<uint32_t>& hits, std::vector<uint32_t>& frees) {
  const double v = M.v;
  const double min_sin = std::sin(o.fs_min_angle * M_PI / 180.0);
  for (size_t i = 0; i < local.size(); ++i) {  // hits: every point
    const Eigen::Vector3d b = origin + local[i].cast<double>();
    if (!M.near(b, 0.0)) continue;
    auto it = M.idx.find(M.key(b));
    if (it != M.idx.end()) hits.push_back(it->second);
  }
  for (size_t i = 0; i < local.size(); i += (size_t)o.fs_stride) {  // frees: every stride-th ray
    const Eigen::Vector3d b = origin + local[i].cast<double>();
    if (!M.near(b, 0.0) && !M.near(sensor, 0.0)) {
      // ray must cross the filtered area: cheap 2D segment-box rejection
      const double tx0 = (M.lo.x() - sensor.x()) / (b.x() - sensor.x()), tx1 = (M.hi.x() - sensor.x()) / (b.x() - sensor.x());
      const double ty0 = (M.lo.y() - sensor.y()) / (b.y() - sensor.y()), ty1 = (M.hi.y() - sensor.y()) / (b.y() - sensor.y());
      const double tmin = std::max(std::min(tx0, tx1), std::min(ty0, ty1)), tmax = std::min(std::max(tx0, tx1), std::max(ty0, ty1));
      if (!(tmax >= std::max(0.0, tmin) && tmin <= 1.0)) continue;
    }
    const Eigen::Vector3d d = b - sensor;
    const double L = d.norm(), Lmax = L - o.fs_margin;
    if (Lmax <= 0) continue;
    const Eigen::Vector3d dir = d / L;
    int64_t c[3], step[3];
    double tmax[3], tdelta[3];
    for (int a = 0; a < 3; ++a) {
      c[a] = (int64_t)std::floor(sensor[a] / v);
      if (std::abs(dir[a]) < 1e-12) { step[a] = 0; tmax[a] = tdelta[a] = std::numeric_limits<double>::infinity(); continue; }
      step[a] = dir[a] > 0 ? 1 : -1;
      const double edge = (c[a] + (step[a] > 0 ? 1 : 0)) * v;
      tmax[a] = (edge - sensor[a]) / dir[a];
      tdelta[a] = v / std::abs(dir[a]);
    }
    double t = 0.0;
    while (t <= Lmax) {
      auto f = M.idx.find(packKey(c[0], c[1], c[2]));
      if (f != M.idx.end()) {
        const Eigen::Vector3f& n = M.normal[f->second];
        // flat voxel crossed at a grazing angle: the ray skims the surface, it doesn't see through it
        if (n.squaredNorm() == 0.0f || std::abs(n.cast<double>().dot(dir)) >= min_sin) frees.push_back(f->second);
      }
      const int a = tmax[0] < tmax[1] ? (tmax[0] < tmax[2] ? 0 : 2) : (tmax[1] < tmax[2] ? 1 : 2);
      t = tmax[a];
      tmax[a] += tdelta[a];
      c[a] += step[a];
    }
  }
  auto uniq = [](std::vector<uint32_t>& x) { std::sort(x.begin(), x.end()); x.erase(std::unique(x.begin(), x.end()), x.end()); };
  uniq(hits);
  uniq(frees);
}

// Per-worker state: a trajectory copy (its segment / gyro caches are not thread-safe) and a submap cache.
struct ScanWorker {
  Trajectory traj;
  SubmapBuilder submaps;
  double time_scale = -1.0;  // per-point time field units -> seconds, detected from the first scan
  bool warned_no_time = false;
  ScanWorker(const Trajectory& t, const Options& o) : traj(t), submaps(traj, o) {}
};

struct ScanIn {
  sensor_msgs::PointCloud2::ConstPtr msg;
  double stamp = 0.0;
  int k0 = -1;
};

struct ScanOut {
  enum Status { KEPT, EMPTY, NO_SUBMAP, ICP_FAIL, OVER_CORRECTION, BAD_LAYOUT } status = EMPTY;
  double stamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // sensor pose at stamp
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Cloud3f local;  // map-frame points minus origin
  std::vector<float> intens;
  bool icp = false;
  double inlier = 0.0;
  std::vector<uint32_t> fs_hits, fs_frees;  // free-space evidence (--fs-ref)
  bool fs_cast = false;                     // near the filtered tiles, so it was ray cast
};

static std::mutex g_print_mu;

// One scan: deskew with the interpolated keyframe trajectory, then refine with ICP against the local
// keyframe submap starting from that trajectory. No state carries over from the previous scan, so scans
// are processed in parallel; the per-scan ICP step is limited to 1 m / 5 deg (icpPointToPlane) and to
// --max-correction / --max-correction-deg measured at the sensor.
static ScanOut processScan(ScanWorker& w, const ScanIn& in, const Options& o, const Eigen::Matrix3d& R_level) {
  ScanOut out;
  out.stamp = in.stamp;
  const sensor_msgs::PointCloud2& msg = *in.msg;
  const double stamp = in.stamp;
  const int k0 = in.k0;
  const Trajectory& traj = w.traj;
  const Layout L = parseLayout(msg);
  if (!L.ok()) { out.status = ScanOut::BAD_LAYOUT; return out; }
  const size_t n_pts = size_t(msg.width) * msg.height;
  const uint8_t* data = msg.data.data();
  if (L.time.offset < 0 && !w.warned_no_time) {
    std::lock_guard<std::mutex> lk(g_print_mu);
    std::cout << "WARN: no per-point time field; scans are not deskewed\n";
    w.warned_no_time = true;
  }
  // Time zero is the scan's earliest valid point, not point 0: an organized Ouster cloud starts
  // at a fixed azimuth, so point 0 can be ~one sweep (~97 ms) into the scan. Using it shifted
  // every point time, which the ICP refinement then silently absorbed into scan_poses.txt.
  double t0 = 0.0;
  if (L.time.offset >= 0 && n_pts) {
    t0 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n_pts; ++i) {
      const uint8_t* p = data + i * msg.point_step;
      if (!std::isfinite(L.x.read(p)) || !std::isfinite(L.y.read(p)) || !std::isfinite(L.z.read(p))) continue;
      t0 = std::min(t0, L.time.read(p));
    }
    if (!std::isfinite(t0)) t0 = L.time.read(data);
  }
  if (w.time_scale < 0 && L.time.offset >= 0 && n_pts > 100) {
    double tmin = t0, tmax = t0;
    for (size_t i = 0; i < n_pts; ++i) {
      const double v = L.time.read(data + i * msg.point_step);
      tmin = std::min(tmin, v);
      tmax = std::max(tmax, v);
    }
    const double span = tmax - tmin;  // Livox: ~1e8 ns for a 10 Hz frame
    if (span > 0) w.time_scale = span > 1e5 ? 1e-9 : span > 1e2 ? 1e-6 : 1.0;
  }
  const bool have_time = L.time.offset >= 0 && w.time_scale > 0;

  const Submap* sub = o.refine ? w.submaps.get(k0) : nullptr;
  if (o.refine && !sub) { out.status = ScanOut::NO_SUBMAP; return out; }
  const Eigen::Vector3d origin = sub ? sub->origin : traj.kf(k0).p;

  // Per-point pose: interpolated trajectory.
  Cloud3f& local = out.local;
  std::vector<float>& intens = out.intens;
  local.reserve(n_pts);
  intens.reserve(n_pts);
  int k = k0;
  for (size_t i = 0; i < n_pts; ++i) {
    const uint8_t* p = data + i * msg.point_step;
    const Eigen::Vector3d raw(L.x.read(p), L.y.read(p), L.z.read(p));
    if (!raw.allFinite()) continue;
    const double r = raw.norm();
    if (r < o.min_range || r > o.max_range) continue;
    const double t = have_time ? stamp + (L.time.read(p) - t0) * w.time_scale : stamp;
    k = traj.segment(t, k);
    if (k < 0 || traj.gap(k) > o.max_gap) { k = k0; continue; }
    const Eigen::Vector3d pm = traj.interp(k, t) * (R_level * raw);
    local.push_back((pm - origin).cast<float>());
    intens.push_back(L.intensity.offset >= 0 ? (float)L.intensity.read(p) : 0.0f);
  }
  if (local.size() < 100) { out.status = ScanOut::EMPTY; return out; }

  Eigen::Isometry3d corr = Eigen::Isometry3d::Identity();  // ICP correction (map frame)
  if (sub) {
    const Cloud3f ds = voxelCentroids(local, (float)o.icp_leaf);
    const IcpResult icp = icpPointToPlane(ds, *sub, o);
    if (!icp.ok) { out.status = ScanOut::ICP_FAIL; return out; }
    Eigen::Isometry3d to = Eigen::Isometry3d::Identity();
    to.translation() = origin;
    corr = to * icp.T * to.inverse();
    // Size of the correction where the scan is (not about the map origin, where a small rotation
    // would read as a large translation).
    const Eigen::Vector3d sensor = traj.interp(k0, stamp).translation();
    if ((o.max_correction > 0 && (corr * sensor - sensor).norm() > o.max_correction) ||
        (o.max_correction_deg > 0 && Eigen::AngleAxisd(corr.linear()).angle() > o.max_correction_deg * M_PI / 180.0)) {
      out.status = ScanOut::OVER_CORRECTION;
      return out;
    }
    out.icp = true;
    out.inlier = icp.inlier_ratio;
    const Eigen::Isometry3f Tf = icp.T.cast<float>();
    for (auto& q : local) q = Tf * q;
  }
  out.origin = origin;
  out.pose = corr * traj.interp(k0, stamp);
  out.status = ScanOut::KEPT;
  return out;
}

// Reads one bag's gyro (leveled with R_level, stamps shifted by --stamp-offset) into g. Pulling the IMU
// topic out of a bag reads the whole file (IMU messages sit in every chunk), so the raw samples are
// cached per bag in o.gyro_cache, keyed by bag file name, size and mtime.
static bool loadBagGyro(rosbag::Bag& bag, const std::string& bag_path, const Options& o,
                        const Eigen::Matrix3d& R_level, GyroTrack& g, std::string& err) {
  std::vector<double> raw;  // t wx wy wz (raw sensor frame, no stamp offset)
  std::string cache;
  if (!o.gyro_cache.empty()) {
    boost::system::error_code ec1, ec2;
    const auto sz = fs::file_size(bag_path, ec1);
    const auto mt = fs::last_write_time(bag_path, ec2);
    if (!ec1 && !ec2) {
      std::string topic = o.imu_topic;
      std::replace(topic.begin(), topic.end(), '/', '_');
      cache = (boost::format("%s/%s_%d_%d%s.gyro") % o.gyro_cache % fs::path(bag_path).filename().string() %
               (unsigned long long)sz % (long long)mt % topic).str();
      std::ifstream in(cache, std::ios::binary | std::ios::ate);
      if (in) {
        const std::streamoff bytes = in.tellg();
        if (bytes > 0 && bytes % (4 * sizeof(double)) == 0) {
          raw.resize(size_t(bytes) / sizeof(double));
          in.seekg(0);
          if (!in.read(reinterpret_cast<char*>(raw.data()), bytes)) raw.clear();
        }
      }
    }
  }
  if (raw.empty()) {
    try {
      rosbag::View view(bag, rosbag::TopicQuery(o.imu_topic));  // same opened bag: its index is read once
      for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        if (!imu) continue;
        const auto& w = imu->angular_velocity;
        raw.insert(raw.end(), {imu->header.stamp.toSec(), w.x, w.y, w.z});
      }
    } catch (const std::exception& e) {
      err = "cannot read " + o.imu_topic + " from " + bag_path + ": " + e.what();
      return false;
    }
    if (!cache.empty() && !raw.empty()) {
      boost::system::error_code ec;
      fs::create_directories(o.gyro_cache, ec);
      const std::string tmp = cache + ".part";
      std::ofstream out(tmp, std::ios::binary);
      if (out && out.write(reinterpret_cast<const char*>(raw.data()), raw.size() * sizeof(double))) {
        out.close();
        fs::rename(tmp, cache, ec);
      }
    }
  }
  for (size_t i = 0; i + 3 < raw.size(); i += 4)
    g.add(raw[i] + o.stamp_offset, R_level * Eigen::Vector3d(raw[i + 1], raw[i + 2], raw[i + 3]));
  g.finalize();
  return true;
}

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

  const Eigen::Matrix3d R_level = (Eigen::AngleAxisd(o.rz, Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(o.ry, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(o.rx, Eigen::Vector3d::UnitX())).toRotationMatrix();
  if (o.gyro_cache == "none") o.gyro_cache.clear();
  else if (o.gyro_cache == "auto") {
    const char* home = std::getenv("HOME");
    o.gyro_cache = std::string(home ? home : "/tmp") + "/.cache/liorf_densify_gyro";
  }

  struct Stats {
    size_t scans = 0, window = 0, every = 0, outside = 0, gap = 0, empty = 0, icp_ok = 0, icp_fail = 0,
           no_submap = 0, stationary = 0, kept = 0, over_correction = 0, rot_mismatch = 0, session_start = 0;
    double inlier_sum = 0;
  } st;

  const int n_workers = o.threads > 0 ? o.threads : std::max(1, (int)std::thread::hardware_concurrency() - 1);
  if (o.finalize_only) {
    if (!fs::exists(spill_dir)) { std::cerr << "ERROR: --finalize-only: no " << spill_dir << "\n"; return 1; }
    std::cout << "Finalize only: building tiles from " << spill_dir << " (" << o.tile_workers << " tiles in flight, "
              << o.tile_mem_gb << " GB budget)\n" << std::flush;
    const size_t written = finalizeTiles(spill_dir, o, false);
    size_t merged_pts = 0;
    if (o.merged) merged_pts = mergeAllTiles(o);
    const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    std::cout << (boost::format("Done: %d points in new tiles%s (%.0f s)\n") % written %
                  (o.merged ? (boost::format(", dense_map.pcd rebuilt from all tiles: %d points") % merged_pts).str()
                            : std::string()) % el);
    return 0;
  }
  std::vector<std::unique_ptr<ScanWorker>> workers;
  for (int i = 0; i < n_workers; ++i) workers.emplace_back(new ScanWorker(traj, o));
  Pool pool(n_workers);
  std::cout << (boost::format("Workers: %d (sequential bag reading and spill writing); gyro cache: %s\n") %
                n_workers % (o.gyro_cache.empty() ? "off" : o.gyro_cache)) << std::flush;

  const std::set<std::pair<int, int>> only_tiles(o.only_tiles.begin(), o.only_tiles.end());
  FsMap fsmap;
  const bool fs_mode = !o.fs_ref.empty();
  if (fs_mode) {
    if (only_tiles.empty()) { std::cerr << "ERROR: --fs-ref needs --only-tiles\n"; return 1; }
    if (fs::exists(o.fs_ref) && fs::equivalent(fs::path(o.out), fs::path(o.fs_ref))) {
      std::cerr << "ERROR: --out must differ from --fs-ref (it would overwrite the reference tiles)\n";
      return 1;
    }
    fsmap.v = o.fs_voxel;
    fsmap.lo = Eigen::Vector3d::Constant(1e18);
    fsmap.hi = Eigen::Vector3d::Constant(-1e18);
    size_t npts = 0;
    std::vector<std::array<double, 10>> mom;  // n, sum xyz, sum xx xy xz yy yz zz (relative to voxel corner)
    for (const auto& t : only_tiles) {
      const std::string tp = (boost::format("%s/tiles/tile_%d_%d.pcd") % o.fs_ref % t.first % t.second).str();
      pcl::PointCloud<pcl::PointXYZI> pc;
      if (pcl::io::loadPCDFile(tp, pc) != 0) { std::cerr << "ERROR: cannot read " << tp << "\n"; return 1; }
      for (const auto& q : pc) {
        const Eigen::Vector3d p(q.x, q.y, q.z);
        const auto ins = fsmap.idx.emplace(fsmap.key(p), (uint32_t)fsmap.idx.size());
        if (ins.second) mom.push_back({});
        auto& m = mom[ins.first->second];
        const Eigen::Vector3d r = p - (p / fsmap.v).array().floor().matrix() * fsmap.v;
        m[0] += 1; m[1] += r.x(); m[2] += r.y(); m[3] += r.z();
        m[4] += r.x() * r.x(); m[5] += r.x() * r.y(); m[6] += r.x() * r.z();
        m[7] += r.y() * r.y(); m[8] += r.y() * r.z(); m[9] += r.z() * r.z();
      }
      npts += pc.size();
      fsmap.lo = fsmap.lo.cwiseMin(Eigen::Vector3d(t.first * o.tile, t.second * o.tile, 0));
      fsmap.hi = fsmap.hi.cwiseMax(Eigen::Vector3d((t.first + 1) * o.tile, (t.second + 1) * o.tile, 0));
    }
    fsmap.hit.assign(fsmap.idx.size(), 0);
    fsmap.fre.assign(fsmap.idx.size(), 0);
    fsmap.normal.assign(fsmap.idx.size(), Eigen::Vector3f::Zero());
    size_t flat = 0;
    for (size_t i = 0; i < mom.size(); ++i) {
      const auto& m = mom[i];
      if (m[0] < 5) continue;
      const Eigen::Vector3d mu(m[1] / m[0], m[2] / m[0], m[3] / m[0]);
      Eigen::Matrix3d Cv;
      Cv << m[4] / m[0], m[5] / m[0], m[6] / m[0], m[5] / m[0], m[7] / m[0], m[8] / m[0], m[6] / m[0], m[8] / m[0], m[9] / m[0];
      Cv -= mu * mu.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Cv);
      const Eigen::Vector3d ev = es.eigenvalues();  // ascending
      if (ev(0) < 0.12 * ev.sum() && ev(1) > 0.08 * ev.sum()) {  // flat: one thin direction, two spread ones
        fsmap.normal[i] = es.eigenvectors().col(0).cast<float>().normalized();
        ++flat;
      }
    }
    std::cout << (boost::format("Free-space: %d of %d voxels flat (walls/ground; grazing rays ignored there)\n") % flat %
                  mom.size()) << std::flush;
    std::cout << (boost::format("Free-space: %d map points of %d tile(s) -> %d voxels of %.2f m\n") % npts %
                  only_tiles.size() % fsmap.idx.size() % fsmap.v) << std::flush;
  }
  if (!only_tiles.empty() && !fs_mode) std::cout << "Only spilling points in " << only_tiles.size() << " tile(s)\n";
  // Free-space mode only filters the reference tiles: no spill and no scan_poses.txt.
  if (!fs_mode && fs::exists(spill_dir)) fs::remove_all(spill_dir);
  TileSpill spill(fs_mode ? std::string() : spill_dir, o.tile, size_t(512) << 20);
  std::ofstream poses_out;
  if (!fs_mode) {
    poses_out.open(o.out + "/scan_poses.txt");
    poses_out << std::setprecision(17);
    // pose_time=scan_start: each pose is the sensor pose at the scan's header stamp. Files without this
    // line predate the t0 fix and their poses sit at (stamp + first-point time offset) instead.
    poses_out << "# liorf_densify_map pose_time=scan_start  columns: stamp x y z qx qy qz qw\n";
  }
  size_t gyro_samples = 0, fs_cast = 0;
  bool bad_layout = false;
  const size_t kBlock = 50;                       // consecutive scans per task (keeps submap reuse)
  const size_t max_inflight = 2 * (size_t)n_workers;

  for (const std::string& bag_path : o.bags) {
    const auto t_bag = std::chrono::steady_clock::now();
    rosbag::Bag bag;
    try {
      bag.open(bag_path, rosbag::bagmode::Read);
    } catch (const std::exception& e) {
      std::cerr << "ERROR: cannot open " << bag_path << ": " << e.what() << "\n";
      return 1;
    }
    GyroTrack gyro;
    if (!o.imu_topic.empty()) {
      std::string err;
      if (!loadBagGyro(bag, bag_path, o, R_level, gyro, err)) { std::cerr << "ERROR: " << err << "\n"; return 1; }
      gyro_samples += gyro.size();
      if (gyro.empty()) std::cout << "WARN: no " << o.imu_topic << " messages in " << bag_path << "; rotation between keyframes is slerped\n";
    }
    const GyroTrack* gp = gyro.empty() ? nullptr : &gyro;
    traj.setGyro(gp, o.gyro_max_resid);  // reader's copy (rotation-mismatch filter)
    for (auto& w : workers) w->traj.setGyro(gp, o.gyro_max_resid);

    rosbag::View view(bag, rosbag::TopicQuery(o.topic));
    std::cout << "Bag " << bag_path << ": " << view.size() << " messages on " << o.topic << ", " << gyro.size()
              << " gyro samples\n" << std::flush;
    const size_t kept_before = st.kept, scans_before = st.scans;

    Eigen::Isometry3d static_anchor = Eigen::Isometry3d::Identity();
    int static_count = 0;
    bool have_anchor = false;
    std::deque<std::future<std::vector<ScanOut>>> inflight;
    std::vector<ScanIn> block;

    // In-order commit: stationary limiter, spill (single writer), scan_poses.
    auto commit = [&](std::vector<ScanOut>&& outs) {
      for (ScanOut& s : outs) {
        switch (s.status) {
          case ScanOut::BAD_LAYOUT: bad_layout = true; continue;
          case ScanOut::EMPTY: ++st.empty; continue;
          case ScanOut::NO_SUBMAP: ++st.no_submap; continue;
          case ScanOut::ICP_FAIL: ++st.icp_fail; continue;
          case ScanOut::OVER_CORRECTION: ++st.over_correction; continue;
          case ScanOut::KEPT: break;
        }
        if (fs_mode) {
          for (uint32_t i : s.fs_hits) ++fsmap.hit[i];
          for (uint32_t i : s.fs_frees) ++fsmap.fre[i];
          if (s.icp) { ++st.icp_ok; st.inlier_sum += s.inlier; }
          ++st.kept;
          if (s.fs_cast) ++fs_cast;
          continue;
        }
        if (s.icp) {
          ++st.icp_ok;
          st.inlier_sum += s.inlier;
        }
        // Stationary limiter: the MID360 pattern fills in over ~2 s; past that a stopped robot only
        // adds passing pedestrians/cars.
        if (o.max_static_scans > 0) {
          const Eigen::Isometry3d d = have_anchor ? static_anchor.inverse() * s.pose : Eigen::Isometry3d::Identity();
          if (have_anchor && d.translation().norm() < 0.05 && Eigen::AngleAxisd(d.linear()).angle() < 0.02) {
            if (++static_count > o.max_static_scans) { ++st.stationary; continue; }
          } else {
            static_anchor = s.pose;
            have_anchor = true;
            static_count = 0;
          }
        }
        if (only_tiles.empty()) {
          for (size_t i = 0; i < s.local.size(); ++i) spill.add(s.origin + s.local[i].cast<double>(), s.intens[i]);
        } else {
          for (size_t i = 0; i < s.local.size(); ++i) {
            const Eigen::Vector3d pt = s.origin + s.local[i].cast<double>();
            const std::pair<int, int> key((int)std::floor(pt.x() / o.tile), (int)std::floor(pt.y() / o.tile));
            if (only_tiles.count(key)) spill.add(pt, s.intens[i]);
          }
        }
        ++st.kept;
        const Eigen::Quaterniond q(s.pose.linear());
        poses_out << s.stamp << " " << s.pose.translation().x() << " " << s.pose.translation().y() << " "
                  << s.pose.translation().z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        if (st.kept % 2000 == 0) {
          const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
          std::cout << (boost::format("scans %d kept %d | icp fail %d | over max-correction %d | mean inliers %.2f | %.1fM pts | %.0f s\n") %
                        st.scans % st.kept % st.icp_fail % st.over_correction % (st.icp_ok ? st.inlier_sum / st.icp_ok : 0.0) %
                        (spill.total() / 1e6) % el)
                    << std::flush;
        }
      }
    };
    auto submit = [&]() {
      if (block.empty()) return;
      inflight.push_back(pool.submit([b = std::move(block), &workers, &o, &R_level, &fsmap, fs_mode](int wid) {
        std::vector<ScanOut> outs;
        outs.reserve(b.size());
        for (const ScanIn& in : b) {
          outs.push_back(processScan(*workers[wid], in, o, R_level));
          ScanOut& so = outs.back();
          if (fs_mode && so.status == ScanOut::KEPT && fsmap.near(so.pose.translation(), o.max_range)) {
            castScan(fsmap, so.pose.translation(), so.origin, so.local, o, so.fs_hits, so.fs_frees);
            so.fs_cast = true;
            Cloud3f().swap(so.local);  // points not needed further in this mode
            std::vector<float>().swap(so.intens);
          }
        }
        return outs;
      }));
      block = std::vector<ScanIn>();
      while (inflight.size() > max_inflight) {
        commit(inflight.front().get());
        inflight.pop_front();
      }
    };

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
      if (k0 < 0) { ++st.outside; continue; }
      if (traj.gap(k0) > o.max_gap) { ++st.gap; continue; }
      if (o.skip_session_start > 0) {
        // k0 lies within the first skip_session_start segments of its session?
        bool start = false;
        for (int j = k0; j > k0 - o.skip_session_start; --j)
          if (j == 0 || traj.gap(j - 1) > o.max_gap) { start = true; break; }
        if (start) { ++st.session_start; continue; }
      }
      // Always evaluated: it also counts each segment once for the gyro summary (workers' trajectory
      // copies would count segments shared between blocks more than once).
      const double rot_mismatch = traj.rotMismatch(k0);
      if (o.drop_rot_mismatch > 0 && rot_mismatch > o.drop_rot_mismatch) { ++st.rot_mismatch; continue; }
      block.push_back({msg, stamp, k0});
      if (block.size() >= kBlock) submit();
    }
    submit();
    while (!inflight.empty()) {
      commit(inflight.front().get());
      inflight.pop_front();
    }
    if (bad_layout) { std::cerr << "ERROR: cloud has no x/y/z fields in " << bag_path << "\n"; return 1; }
    traj.setGyro(nullptr, o.gyro_max_resid);  // gyro goes out of scope
    for (auto& w : workers) w->traj.setGyro(nullptr, o.gyro_max_resid);
    std::cout << (boost::format("done %s: %d scans read, %d kept (%.0f s)\n") % fs::path(bag_path).stem().string() %
                  (st.scans - scans_before) % (st.kept - kept_before) %
                  std::chrono::duration<double>(std::chrono::steady_clock::now() - t_bag).count())
              << std::flush;
  }
  spill.flush();
  poses_out.close();
  if (fs_mode) {
    size_t dyn_vox = 0;
    std::vector<uint8_t> dynamic(fsmap.idx.size(), 0);
    for (size_t i = 0; i < dynamic.size(); ++i)
      if ((int)fsmap.fre[i] >= o.fs_min_free && fsmap.fre[i] >= o.fs_ratio * fsmap.hit[i]) { dynamic[i] = 1; ++dyn_vox; }
    fs::create_directories(o.out + "/tiles");
    fs::create_directories(o.out + "/removed");
    size_t kept_pts = 0, rem_pts = 0;
    for (const auto& t : only_tiles) {
      const std::string name = (boost::format("tile_%d_%d.pcd") % t.first % t.second).str();
      pcl::PointCloud<pcl::PointXYZI> pc;
      pcl::io::loadPCDFile(o.fs_ref + "/tiles/" + name, pc);
      std::vector<float> keep, rem;
      keep.reserve(pc.size() * 4);
      for (const auto& q : pc) {
        const auto it = fsmap.idx.find(fsmap.key(Eigen::Vector3d(q.x, q.y, q.z)));
        std::vector<float>& dst = (it != fsmap.idx.end() && dynamic[it->second]) ? rem : keep;
        dst.insert(dst.end(), {q.x, q.y, q.z, q.intensity});
      }
      PcdWriter wk(o.out + "/tiles/" + name);
      wk.write(keep.data(), keep.size() / 4);
      kept_pts += wk.close();
      PcdWriter wr(o.out + "/removed/" + name);
      wr.write(rem.data(), rem.size() / 4);
      rem_pts += wr.close();
    }
    // evidence summary
    size_t seen = 0;
    for (size_t i = 0; i < dynamic.size(); ++i) seen += (fsmap.hit[i] + fsmap.fre[i]) > 0;
    std::cout << (boost::format("Free-space: %d scans cast, %d/%d voxels observed, %d dynamic (%.2f%%) | points kept %d, removed %d (%.2f%%)\n") %
                  fs_cast % seen % dynamic.size() % dyn_vox % (100.0 * dyn_vox / std::max<size_t>(1, dynamic.size())) %
                  kept_pts % rem_pts % (100.0 * rem_pts / std::max<size_t>(1, kept_pts + rem_pts)));
    return 0;
  }

  size_t builds = 0, missing = 0, gyro_used = 0, gyro_fallback = 0;
  double gyro_resid_mean = 0.0;
  for (auto& w : workers) {
    builds += w->submaps.builds();
    missing += w->submaps.missingClouds();
  }
  traj.gyroStats(gyro_used, gyro_fallback, gyro_resid_mean);
  std::cout << (boost::format(
                    "Scans: %d read, %d kept | skipped: %d outside trajectory, %d in gaps, %d window, %d --every, "
                    "%d stationary, %d empty, %d ICP rejected, %d over --max-correction, %d keyframe/gyro mismatch, %d session start, %d no submap\n") %
                st.scans % st.kept % st.outside % st.gap % st.window % st.every % st.stationary % st.empty %
                st.icp_fail % st.over_correction % st.rot_mismatch % st.session_start % st.no_submap);
  if (o.refine)
    std::cout << (boost::format("ICP: mean inlier ratio %.3f over %d aligned scans, %d submap builds, %d missing keyframe clouds\n") %
                  (st.icp_ok ? st.inlier_sum / st.icp_ok : 0.0) % st.icp_ok % builds % missing);
  if (gyro_samples) {
    std::cout << "Gyro: " << gyro_samples << " samples from " << o.imu_topic << "\n";
    std::cout << (boost::format("Gyro: %d segments gyro-interpolated (mean keyframe mismatch %.4f rad), %d slerp fallback\n") %
                  gyro_used % gyro_resid_mean % gyro_fallback);
  }
  if (st.kept == 0) {
    std::cerr << "ERROR: no scans kept. Check that the bag matches the graph (stamps), --topic, and --max-gap"
                 " (and that graph/NNNNNN/cloud.pcd exist, or use --no-refine).\n";
    return 1;
  }
  std::cout << "Raw points: " << spill.total() << ". Writing tiles (voxel " << o.voxel << " m, " << o.tile_workers
            << " tiles in flight)...\n" << std::flush;
  workers.clear();  // scan-phase caches (trajectory copies, submaps) are not needed for the tiles
  malloc_trim(0);
  // With --only-tiles the new tiles are only part of the map: rebuild dense_map.pcd from all tiles instead.
  const size_t written = finalizeTiles(spill_dir, o, o.merged && only_tiles.empty());
  if (o.merged && !only_tiles.empty()) {
    const size_t merged_pts = mergeAllTiles(o);
    std::cout << "dense_map.pcd rebuilt from all tiles in " << o.out << "/tiles: " << merged_pts << " points\n";
  }
  const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  std::cout << (boost::format("Done: %d points in %s/tiles%s (%.0f s)\n") % written % o.out %
                (o.merged ? " and dense_map.pcd" : "") % el);
  return 0;
}
