#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <boost/format.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "liorf/FactorGraphLoader.h"

void dump(
    const std::string& dump_directory, const gtsam::ISAM2& isam,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<PointType>::Ptr>& surf_cloud_keyframes
);

void dumpYAML(
    const std::string& dump_directory, const gtsam::ISAM2& isam,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<PointType>::Ptr>& surf_cloud_keyframes,
    const GeographicLib::LocalCartesian* gps_trans = nullptr
);