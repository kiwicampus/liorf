// Finds and quantifies candidate loop closures for keyframe pairs that a spatial-proximity scan
// (find_unclosed_revisits.py) flagged as physically close but with no recorded BetweenFactor in
// the graph. For each candidate pair (anchor keyframe id in pass A, anchor keyframe id in pass B):
//
//   1. Build a small submap around each anchor from its neighbouring keyframe clouds, in that
//      anchor's own local frame (same construction as liorf_densify_map's SubmapBuilder).
//   2. Use the current graph poses' relative transform as the initial guess (by construction the
//      anchors were found within a few metres of each other, so this is already close).
//   3. Refine with the same point-to-plane ICP as liorf_densify_map, but with a wider initial
//      capture radius (these are metres-scale corrections, not the sub-metre per-scan refinement
//      liorf_densify_map does).
//   4. Write a before/after overlay PCD (pass A colored via intensity=0, pass B via intensity=1)
//      so the result can be visually spot-checked, and print the resulting relative pose + fitness
//      so a human/script can decide whether to accept it as a new loop-closure factor.
//
// This does NOT modify any graph file itself -- it only reports candidates.
//
// Usage: rosrun liorf liorf_find_loop_closures --graph <dir> --events <events.txt> --out <dir>
//   events.txt: one "event_idx aid bid" line per candidate (aid/bid = anchor keyframe ids).

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <boost/format.hpp>
#include <omp.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "liorf/FactorGraphLoader.h"

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

struct Submap {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
  std::vector<Eigen::Vector3f> normals;
  std::vector<uint8_t> planar;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
};

struct IcpResult {
  bool ok = false;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  double inlier_ratio = 0.0;
  int correspondences = 0;
};

// Same point-to-plane Gauss-Newton ICP as liorf_densify_map, but with caller-supplied distance
// schedule (revisit corrections can be metres, not sub-metre) and no small-correction accept gate
// -- the caller decides what to trust from inlier_ratio/correspondences, since a human will also
// look at the overlay cloud.
static IcpResult icpPointToPlane(const Cloud3f& src, const Submap& sub, int max_iter,
                                  const std::vector<double>& dist_schedule) {
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

  for (int it = 0; it < max_iter; ++it) {
    const size_t stage = std::min(dist_schedule.size() - 1, dist_schedule.size() * it / max_iter);
    const float max_d = float(dist_schedule[stage]);
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
    if (n < 50) continue;
    H += Mat6::Identity() * (1e-6 * H.trace() + 1e-9);
    const Vec6 dx = -H.ldlt().solve(g);
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    const double ang = dx.head<3>().norm();
    if (ang > 1e-12) step.linear() = Eigen::AngleAxisd(ang, dx.head<3>() / ang).toRotationMatrix();
    step.translation() = dx.tail<3>();
    res.T = step * res.T;
  }

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
  res.ok = valid >= 50;
  return res;
}

static Cloud3f loadKeyframeCloud(const std::string& graph, int id) {
  Cloud3f out;
  pcl::PointCloud<pcl::PointXYZI> pc;
  std::string path = (boost::format("%s/%06d/cloud.pcd") % graph % id).str();
  if (pcl::io::loadPCDFile(path, pc) == 0) {
    out.reserve(pc.size());
    for (const auto& p : pc) out.emplace_back(p.x, p.y, p.z);
  }
  return out;
}

// Builds a merged, voxelized, normal-annotated submap around `anchor`, spanning +/-window
// keyframe ids present in `poses`, expressed in `anchor`'s own local frame.
static Submap buildSubmap(const std::string& graph, const std::map<int, gtsam::Pose3>& poses, int anchor,
                           int window, float leaf) {
  Submap sub;
  auto it = poses.find(anchor);
  if (it == poses.end()) return sub;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.linear() = it->second.rotation().matrix();
  origin.translation() = it->second.translation();
  Eigen::Isometry3d origin_inv = origin.inverse();

  Cloud3f merged;
  for (auto pit = poses.lower_bound(anchor - window); pit != poses.end() && pit->first <= anchor + window; ++pit) {
    Cloud3f c = loadKeyframeCloud(graph, pit->first);
    if (c.empty()) continue;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = pit->second.rotation().matrix();
    T.translation() = pit->second.translation();
    Eigen::Isometry3d rel = origin_inv * T;
    Eigen::Isometry3f relf = rel.cast<float>();
    for (const auto& p : c) merged.push_back(relf * p);
  }

  Cloud3f ds = voxelCentroids(merged, leaf);
  auto& cloud = *sub.cloud;
  cloud.resize(ds.size());
  for (size_t i = 0; i < ds.size(); ++i) cloud[i].getVector3fMap() = ds[i];
  if (cloud.size() < 50) {
    cloud.clear();
    return sub;
  }
  sub.tree.setInputCloud(sub.cloud);
  sub.normals.assign(cloud.size(), Eigen::Vector3f::UnitZ());
  sub.planar.assign(cloud.size(), 0);
#pragma omp parallel for schedule(dynamic, 256)
  for (int i = 0; i < (int)cloud.size(); ++i) {
    std::vector<int> idx;
    std::vector<float> d2;
    if (sub.tree.nearestKSearch(cloud[i], 8, idx, d2) < 5) continue;
    const float max_r = 4.0f * leaf;
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
    const Eigen::Vector3f ev = es.eigenvalues();
    if (ev.sum() <= 0 || ev(0) / ev.sum() > 0.05f) continue;
    sub.normals[i] = es.eigenvectors().col(0);
    sub.planar[i] = 1;
  }
  return sub;
}

static void writeOverlay(const std::string& path, const Submap& a, const Cloud3f& b_in_a_frame) {
  pcl::PointCloud<pcl::PointXYZI> out;
  out.reserve(a.cloud->size() + b_in_a_frame.size());
  for (const auto& p : *a.cloud) {
    pcl::PointXYZI q;
    q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = 0.f;
    out.push_back(q);
  }
  for (const auto& p : b_in_a_frame) {
    pcl::PointXYZI q;
    q.x = p.x(); q.y = p.y(); q.z = p.z(); q.intensity = 1.f;
    out.push_back(q);
  }
  pcl::io::savePCDFileBinary(path, out);
}

int main(int argc, char** argv) {
  std::string graph, events_path, out_dir;
  int window = 15;
  float leaf = 0.15f;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (a == "--graph") graph = next();
    else if (a == "--events") events_path = next();
    else if (a == "--out") out_dir = next();
    else if (a == "--window") window = std::stoi(next());
    else if (a == "--leaf") leaf = std::stof(next());
  }
  if (graph.empty() || events_path.empty() || out_dir.empty()) {
    std::cerr << "Usage: liorf_find_loop_closures --graph <dir> --events <events.txt> --out <dir> "
                 "[--window 15] [--leaf 0.15]\n";
    return 1;
  }
  boost::filesystem::create_directories(out_dir);

  FactorGraphLoader loader;
  if (!loader.loadSession(graph, /*optimize=*/false, /*load_clouds=*/false)) {
    std::cerr << "ERROR: failed to load " << graph << std::endl;
    return 1;
  }
  std::map<int, gtsam::Pose3> poses;
  for (const auto& kv : loader.getKeyframeData()) poses[kv.first] = kv.second->pose;
  std::cout << "Loaded " << poses.size() << " keyframe poses from " << graph << std::endl;

  std::ifstream ev(events_path);
  std::ofstream results(out_dir + "/results.txt");
  results << "# event_idx aid bid correspondences inlier_ratio corr_t corr_deg tx ty tz qw qx qy qz\n";
  std::string line;
  while (std::getline(ev, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    int idx, aid, bid;
    if (!(iss >> idx >> aid >> bid)) continue;
    if (!poses.count(aid) || !poses.count(bid)) {
      std::cerr << "event " << idx << ": missing pose for " << aid << " or " << bid << ", skipping\n";
      continue;
    }

    Submap subA = buildSubmap(graph, poses, aid, window, leaf);
    Submap subB = buildSubmap(graph, poses, bid, window, leaf);
    if (subA.cloud->empty() || subB.cloud->empty()) {
      std::cerr << "event " << idx << ": empty submap, skipping\n";
      continue;
    }

    // T0: current graph's estimate of the relative pose from aid's frame to bid's frame.
    Eigen::Isometry3d Ta = Eigen::Isometry3d::Identity();
    Ta.linear() = poses.at(aid).rotation().matrix();
    Ta.translation() = poses.at(aid).translation();
    Eigen::Isometry3d Tb = Eigen::Isometry3d::Identity();
    Tb.linear() = poses.at(bid).rotation().matrix();
    Tb.translation() = poses.at(bid).translation();
    Eigen::Isometry3d T0 = Ta.inverse() * Tb;

    // B's submap cloud is already expressed in bid's own frame; bring it into aid's frame via T0
    // before running ICP (ICP below starts from identity on top of that).
    Eigen::Isometry3f T0f = T0.cast<float>();
    Cloud3f src;
    src.reserve(subB.cloud->size());
    for (const auto& p : *subB.cloud) src.push_back(T0f * p.getVector3fMap());

    writeOverlay((boost::format("%s/event_%03d_before.pcd") % out_dir % idx).str(), subA, src);

    std::vector<double> schedule{3.0, 1.5, 0.6, 0.3};
    IcpResult r = icpPointToPlane(src, subA, 40, schedule);

    Eigen::Isometry3d T_final = r.T * T0;
    Eigen::Isometry3d correction = r.T;
    double corr_t = correction.translation().norm();
    double corr_deg = Eigen::AngleAxisd(correction.linear()).angle() * 180.0 / M_PI;

    Cloud3f after;
    after.reserve(src.size());
    Eigen::Isometry3f rf = r.T.cast<float>();
    for (const auto& p : src) after.push_back(rf * p);
    writeOverlay((boost::format("%s/event_%03d_after.pcd") % out_dir % idx).str(), subA, after);

    Eigen::Quaterniond q(T_final.linear());
    q.normalize();
    Eigen::Vector3d t = T_final.translation();
    printf("event %3d: aid=%6d bid=%6d  corr=%d inlier=%.3f  correction: |dt|=%.3f m, dtheta=%.2f deg\n",
           idx, aid, bid, r.correspondences, r.inlier_ratio, corr_t, corr_deg);
    results << idx << " " << aid << " " << bid << " " << r.correspondences << " " << r.inlier_ratio << " "
            << corr_t << " " << corr_deg << " " << std::setprecision(17) << t.x() << " " << t.y() << " "
            << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
  }
  results.close();
  std::cout << "Done. Overlays + results.txt written to " << out_dir << std::endl;
  return 0;
}
