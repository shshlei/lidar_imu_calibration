// Created by usl on 12/10/20.

#ifndef LIDAR_IMU_CALIBRATION_LIDARODOMETRY_H
#define LIDAR_IMU_CALIBRATION_LIDARODOMETRY_H

#include "lidar_imu_calibration/relpose/relativePose.h"
#include "lidar_imu_calibration/types/Pose.h"
#include "lidar_imu_calibration/utils/eigen_utils.h"
#include "lidar_imu_calibration/utils/math_utils.h"
#include "lidar_imu_calibration/utils/pcl_utils.h"
#include "lidar_imu_calibration/utils/quat_ops.h"

#include <pclomp/ndt_omp.h>

#include <vector>
#include <memory>
#include <string>
#include <fstream>

namespace lidar_imu_calibration
{
class LidarOdometry
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<LidarOdometry> Ptr;

  struct Odom
  {
    double timestamp;
    Eigen::Matrix4d pose;  // current scan to first scan
  };

  explicit LidarOdometry(double ndtResolution = 0.5,
    const std::string & lo_trajectory_filename = "",
    bool downsample_for_mapping = false);

  static pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndtInit(
    double ndt_resolution);

  void feedScan(double timestamp,
    const VPointCloud::Ptr & cur_scan,
    const Eigen::Matrix4d & pose_predict = Eigen::Matrix4d::Identity());

  void detectPlanarTargetInFirstFrame(double timestamp);

  void clearOdometryData();

  void setTargetMap(const VPointCloud::Ptr map_cloud_in);

  const VPointCloud::Ptr getTargetMap()
  {
    return map_cloud_;
  }

  const VPointCloud::Ptr getScanInTarget()
  {
    return scan_in_target_global_;
  }

  const pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr & getNDTPtr() const
  {
    return ndt_omp_;
  }

  const Eigen::aligned_vector<Odom> & get_odom_data() const
  {
    return odom_;
  }

  const Odom & get_current_odometry() const
  {
    return odom_curr;
  }

  const relativePose get_latest_relativePose() const
  {
    return latestRP;
  }

  const Eigen::aligned_vector<Odom> & get_kf_odom_data() const
  {
    return kf_odom_;
  }

  const bool & isKeyFrame() const
  {
    return is_KF_;
  }

  void append_and_update(bool update_map = true);

private:
  void registration(const VPointCloud::Ptr & cur_scan,
    const Eigen::Matrix4d & pose_predict,
    Eigen::Matrix4d & pose_out,
    VPointCloud::Ptr scan_in_target);
  void updateKeyScan(const VPointCloud & cur_scan, const Odom & odom);
  bool checkKeyScan(const Odom & odom);

  // Normalize angle to be between [-180, 180]
  static inline double normalize_angle(double ang_degree)
  {
    if (ang_degree > 180)
      ang_degree -= 360;
    if (ang_degree < -180)
      ang_degree += 360;
    return ang_degree;
  }

private:
  VPointCloud::Ptr map_cloud_;
  VPointCloud::Ptr scan_in_target_global_;
  pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndt_omp_;

  std::vector<size_t> key_frame_index_;
  Eigen::aligned_vector<Odom> kf_odom_;
  Eigen::aligned_vector<Odom> odom_;
  Odom odom_curr;
  relativePose latestRP;  // latest relative pose
  bool is_KF_;

  std::ofstream trajfile_csv;
  VPointCloud current_scan;

  bool first_scan = true;
  bool downsampleForMapping;
};
}  // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_LIDARODOMETRY_H
