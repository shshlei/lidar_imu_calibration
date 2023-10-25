// Created by usl on 1/3/21.

#ifndef LIDAR_IMU_CALIBRATION_PLANEDETECTOR_H
#define LIDAR_IMU_CALIBRATION_PLANEDETECTOR_H

#define PCL_NO_PRECOMPILE  // !! BEFORE ANY PCL INCLUDE!!

#include "lidar_imu_calibration/utils/eigen_utils.h"
#include "lidar_imu_calibration/utils/math_utils.h"
#include "lidar_imu_calibration/utils/pcl_utils.h"
#include "lidar_imu_calibration/utils/quat_ops.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <track/PlaneDetectorParams.h>

namespace lidar_imu_calibration
{
class PlaneDetector
{
public:
  PlaneDetector(const PlaneDetectorParams & _params)
  {
    _first_frame = true;
    std::cout << "--PlaneTargetDetector Initialized!--" << std::endl;
    _params.print();
  }

  void feedData();

private:
  bool _first_frame;
};
}  // namespace lidar_imu_calibration
#endif  // LIDAR_IMU_CALIBRATION_PLANEDETECTOR_H
