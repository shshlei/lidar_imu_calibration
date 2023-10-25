// Created by usl on 11/29/20.

#ifndef LIDAR_IMU_CALIBRATION_RELATIVEPOSE_H
#define LIDAR_IMU_CALIBRATION_RELATIVEPOSE_H

#include <Eigen/Core>

namespace lidar_imu_calibration
{
class relativePose
{
public:
  /// Time stamp of scan i
  double timestamp_i;
  /// Time stamp of scan j
  double timestamp_j;
  /// Odometry pose
  Eigen::Matrix4d odometry_ij;
};
}  // namespace lidar_imu_calibration 
#endif  // LIDAR_IMU_CALIBRATION_RELATIVEPOSE_H
