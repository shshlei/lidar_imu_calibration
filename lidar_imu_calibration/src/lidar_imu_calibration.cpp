// Created by usl on 12/14/20.

#define PCL_NO_PRECOMPILE  // !! BEFORE ANY PCL INCLUDE!!

#include "core/lincalibManager.h"
#include "core/lincalibManagerOptions.h"
#include "ros/ros.h"
#include "track/lidarOdometry.h"
#include "utils/parse_ros.h"
#include "utils/pcl_utils.h"
#include "utils/quat_ops.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/visualization/cloud_viewer.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

using namespace lin_estimator;

lincalibManager * sys;

void downSampleCloud(const pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud_in_pcl,
  pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud_out_pcl,
  int ring_downsample_factor)
{
  cloud_out_pcl->header = cloud_in_pcl->header;
  cloud_out_pcl->is_dense = cloud_in_pcl->is_dense;
  cloud_out_pcl->height = cloud_in_pcl->height;
  cloud_out_pcl->width = cloud_in_pcl->width;
  cloud_out_pcl->resize(cloud_in_pcl->width * cloud_in_pcl->height);
  for (int h = 0; h < cloud_in_pcl->height; h++) {
    for (int w = 0; w < cloud_in_pcl->width; w++) {
      lin_core::PointXYZIR8Y cloud_point = cloud_in_pcl->at(w, h);
      /// Ignore non even rings
      if (cloud_point.ring % ring_downsample_factor != 0)
        continue;
      /// Ignore points with NaNs
      if (isnan(cloud_point.x) || isnan(cloud_point.y) || isnan(cloud_point.z)) {
        continue;
      }
      cloud_out_pcl->at(w, h) = cloud_point;
    }
  }
}

int main(int argc, char ** argv)
{
  /// Launch ros node
  ros::init(argc, argv, "ros_test_node");
  ros::NodeHandle nh("~");
  /// Create our lincalib system
  lincalibManagerOptions params = parse_ros_nodehandler(nh);
  sys = new lincalibManager(params);

  /// Our topics (IMU and LIDAR)
  std::string topic_imu;
  std::string topic_lidar;
  nh.param<std::string>("topic_imu", topic_imu, "/imu");
  nh.param<std::string>("topic_lidar", topic_lidar, "/lidar");

  /// Location of the ROS bag we want to read in
  std::string path_to_bag;
  nh.param<std::string>("path_bag", path_to_bag, "/home/usl/datasets/ouster_vectornav.bag");
  ROS_INFO_STREAM("ROS BAG PATH is: " << path_to_bag.c_str());

  /// Get our start location and how much of the bag we want to play
  /// Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  ROS_INFO_STREAM("bag start: " << bag_start);
  ROS_INFO_STREAM("bag duration: " << bag_durr);

  /// Load rosbag here, and find messages we can play
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  /// We should load the bag as a view
  /// Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  /// Start a few seconds in from the full view time
  /// If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish =  (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
  ROS_INFO_STREAM("Time start = " << time_init.toSec());
  ROS_INFO_STREAM("Time end = " << time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  /// Check to make sure we have data to play
  if (view.size() == 0) {
    ROS_ERROR_STREAM("No messages to play on specified topics.  Exiting.");
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // Step through the rosbag
  int lidar_scan_no = 0;
  int no_of_imu = 0;

  ros::Time start_time = ros::Time::now();
  for (const rosbag::MessageInstance & m : view) {
    /// If ROS wants us to stop, break out
    if (!ros::ok()) break;

    /// Handle IMU measurement
    sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();
    if (s_imu != nullptr && m.getTopic() == topic_imu) {
      double time_imu = (*s_imu).header.stamp.toSec();
      Eigen::Matrix<double, 3, 1> wm, am;
      wm << (*s_imu).angular_velocity.x, (*s_imu).angular_velocity.y, (*s_imu).angular_velocity.z;
      am << (*s_imu).linear_acceleration.x, (*s_imu).linear_acceleration.y, (*s_imu).linear_acceleration.z;

      /// Send it to our linkalibr system
      sys->feed_measurement_imu(time_imu, wm, am);
      no_of_imu++;
    }

    /// Handle Lidar measurement
    sensor_msgs::PointCloud2::ConstPtr s_lidar = m.instantiate<sensor_msgs::PointCloud2>();
    if (s_lidar != nullptr && m.getTopic() == topic_lidar) {
      ROS_INFO_STREAM("Lidar scan no: " << lidar_scan_no);
      lidar_scan_no++;
      ROS_INFO_STREAM("No of imu measurements: " << no_of_imu);
      no_of_imu = 0;

      pcl::PointCloud<lin_core::PointXYZIR8Y>::Ptr cloud(new pcl::PointCloud<lin_core::PointXYZIR8Y>);
      pcl::fromROSMsg(*s_lidar, *cloud);
      double time_lidar = (*s_lidar).header.stamp.toSec();
      sys->feed_measurement_lidar(time_lidar, cloud);
    }
  }

  ros::Time end_time = ros::Time::now();
  ROS_INFO_STREAM("Reached end of bag");
  ROS_INFO_STREAM("Kalman Filtering took : " << end_time.toSec() - start_time.toSec() << " [s]");

  /// Write the final I_T_L to a text file
  State * state = sys->get_state();
  Pose * calibration = state->_calib_LIDARtoIMU;
  Eigen::Matrix3d I_R_L = calibration->Rot();
  Eigen::Vector3d I_t_L = calibration->pos();
  Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
  I_T_L.block(0, 0, 3, 3) = I_R_L;
  I_T_L.block(0, 3, 3, 1) = I_t_L;
  std::cout << "Writing KF calibration result to: " << params.lidar_inertial_calibration_result_filename << std::endl;
  std::ofstream result_calibration;
  result_calibration.open(params.lidar_inertial_calibration_result_filename.c_str());
  result_calibration << I_T_L;
  result_calibration.close();
  std::cout << "I_T_L: " << std::endl;
  std::cout << I_T_L << std::endl;
  Eigen::Vector3d eulerXYZ = I_R_L.eulerAngles(0, 1, 2) * 180 / M_PI;
  ROS_INFO_STREAM("Translation [in m]: " << I_t_L.transpose());
  ROS_INFO_STREAM("Euler Angles [in deg]: " << eulerXYZ.transpose());

  return EXIT_SUCCESS;
}
