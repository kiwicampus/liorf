#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <boost/format.hpp>
#include <GeographicLib/LocalCartesian.hpp>

// The graph is passed explicitly so a caller can export more than one ISAM2 holds
// (e.g. fixed session factors + this run's factors when liorf/fixSessionPoses is on).
void dump(
    const std::string& dump_directory, const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes
);

void dumpYAML(
    const std::string& dump_directory, const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes,
    const GeographicLib::LocalCartesian* gps_trans = nullptr
);
