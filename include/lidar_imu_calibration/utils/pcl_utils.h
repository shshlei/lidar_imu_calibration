// Created by usl on 12/27/20.

#ifndef LIDAR_IMU_CALIBRATION_PCL_UTILS_H
#define LIDAR_IMU_CALIBRATION_PCL_UTILS_H

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
		
namespace lidar_imu_calibration
{

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef pcl::PointXYZRGB colorPointT;
typedef pcl::PointCloud<colorPointT> colorPointCloudT;

struct PointXYZIR8Y
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float         intensity;            ///< laser intensity reading
  std::uint16_t ring;                 ///< laser ring number
  float         time;                 ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
}EIGEN_ALIGN16;

inline void downsampleCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
  pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
  float in_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(in_cloud);
  sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  sor.filter(*out_cloud);
}
}  // namespace lidar_imu_calibration

POINT_CLOUD_REGISTER_POINT_STRUCT(
  lidar_imu_calibration::PointXYZIR8Y,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

typedef lidar_imu_calibration::PointXYZIR8Y TPoint;
typedef pcl::PointCloud<TPoint> TPointCloud;

namespace lidar_imu_calibration
{
inline void TPointCloud2VPointCloud(const TPointCloud::Ptr input_pc, VPointCloud::Ptr output_pc)
{
  output_pc->header = input_pc->header;
  output_pc->height = input_pc->height;
  output_pc->width = input_pc->width;
  output_pc->is_dense = input_pc->is_dense;
  output_pc->resize(output_pc->width * output_pc->height);
  for (std::uint32_t h = 0; h < input_pc->height; h++) {
    for (std::uint32_t w = 0; w < input_pc->width; w++) {
      if (std::isnan(input_pc->at(w, h).x) || std::isnan(input_pc->at(w, h).y) || std::isnan(input_pc->at(w, h).z)) continue;
      lidar_imu_calibration::VPoint point;
      point.x = input_pc->at(w, h).x;
      point.y = input_pc->at(w, h).y;
      point.z = input_pc->at(w, h).z;
      point.intensity = input_pc->at(w, h).intensity;
      output_pc->at(w, h) = point;
    }
  }
}

inline void downsampleCloud(const pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>::Ptr in_cloud,
  pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>::Ptr out_cloud,
  float in_leaf_size)
{
  pcl::VoxelGrid<lidar_imu_calibration::PointXYZIR8Y> sor;
  sor.setInputCloud(in_cloud);
  sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  sor.filter(*out_cloud);
}
}  // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_PCL_UTILS_H
