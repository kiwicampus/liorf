#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <boost/format.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <iostream>

// A file that shows up at `path` only once it is complete. The content goes to path + ".tmp"; commit()
// flushes it, checks that no write failed (full disk, I/O error), syncs it to disk and renames it over `path`
// (atomic on POSIX). A reader, or the next run after an OOM-kill / full disk mid-save, therefore sees either
// the previous file or the whole new one, never a truncated one. If it is destroyed without a successful
// commit() the temp file is deleted and `path` is left as it was.
class AtomicOfstream {
public:
    explicit AtomicOfstream(const std::string& path)
        : path_(path), tmp_(path + ".tmp"), ofs_(tmp_, std::ios::binary) {}
    AtomicOfstream(const AtomicOfstream&) = delete;
    AtomicOfstream& operator=(const AtomicOfstream&) = delete;
    ~AtomicOfstream() { discard(); }

    std::ofstream& stream() { return ofs_; }

    // Returns false (and leaves `path` untouched) if the stream failed at any point or the rename failed.
    bool commit() {
        ofs_.flush();
        const bool good = ofs_.good();  // fail/bad bit stay set after any earlier failed write
        ofs_.close();
        if (!good || ofs_.fail()) {
            std::cerr << "ERROR: writing " << tmp_ << " failed (disk full or I/O error?)" << std::endl;
            discard();
            return false;
        }
        const int fd = ::open(tmp_.c_str(), O_RDONLY);
        if (fd >= 0) { ::fsync(fd); ::close(fd); }
        if (std::rename(tmp_.c_str(), path_.c_str()) != 0) {
            std::perror(("ERROR: renaming " + tmp_ + " to " + path_).c_str());
            discard();
            return false;
        }
        committed_ = true;
        return true;
    }

private:
    void discard() {
        if (committed_) return;
        if (ofs_.is_open()) ofs_.close();
        std::remove(tmp_.c_str());
    }

    std::string path_, tmp_;
    std::ofstream ofs_;
    bool committed_ = false;
};

// savePCDFileBinary() to a temp file + rename, so an interrupted save never leaves a truncated .pcd. PCL
// reports a failed write (full disk, bad path) by THROWING pcl::IOException, which would otherwise escape
// a ROS service callback and kill the node; this returns false (after logging) instead.
template <typename PointT>
bool savePCDFileBinaryAtomic(const std::string& path, const pcl::PointCloud<PointT>& cloud) {
    const std::string tmp = path + ".tmp";
    try {
        if (pcl::io::savePCDFileBinary(tmp, cloud) < 0)
            throw std::runtime_error("savePCDFileBinary returned an error");
        if (std::rename(tmp.c_str(), path.c_str()) != 0)
            throw std::runtime_error(std::string("rename failed: ") + std::strerror(errno));
    } catch (const std::exception& e) {
        std::cerr << "ERROR: could not write " << path << ": " << e.what() << std::endl;
        std::remove(tmp.c_str());
        return false;
    }
    return true;
}

// The graph is passed explicitly so a caller can export more than one ISAM2 holds
// (e.g. fixed session factors + this run's factors when liorf/fixSessionPoses is on).
// Both return false if anything could not be written. Every file is replaced atomically (see
// AtomicOfstream), so a failed or interrupted save leaves the previously saved files in place. Call dump()
// first and dumpYAML() last: factor_graph.yaml is what loadSession() reads, so it is the commit point.
bool dump(
    const std::string& dump_directory, const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes
);

bool dumpYAML(
    const std::string& dump_directory, const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes,
    const GeographicLib::LocalCartesian* gps_trans = nullptr
);
