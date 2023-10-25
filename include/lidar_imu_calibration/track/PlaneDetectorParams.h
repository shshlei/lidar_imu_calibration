// Created by usl on 1/4/21.

#ifndef LIDAR_IMU_CALIBRATION_PLANEDETECTORPARAMS_H
#define LIDAR_IMU_CALIBRATION_PLANEDETECTORPARAMS_H

namespace lidar_imu_calibration
{
struct PlaneDetectorParams
{
  void print()
  {
    std::cout << "--PlaneTargetDetector Parameters--" << std::endl;
  }
};
}  // namespace lidar_imu_calibration
#endif  // LIDAR_IMU_CALIBRATION_PLANEDETECTORPARAMS_H
