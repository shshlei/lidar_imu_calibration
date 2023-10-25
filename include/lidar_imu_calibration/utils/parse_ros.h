// Created by usl on 12/9/20.
#ifndef LIDAR_IMU_CALIBRATION_PARSE_ROS_H
#define LIDAR_IMU_CALIBRATION_PARSE_ROS_H

#include "lidar_imu_calibration/core/calibManagerOptions.h"

#include <rclcpp/rclcpp.hpp>

namespace lidar_imu_calibration
{
class ParamServer : public rclcpp::Node
{
public:

  ParamServer(const std::string & node_name);

  const lidar_imu_calibration::CalibManagerOptions & getCalibManagerOptions() const;

protected:

  lidar_imu_calibration::CalibManagerOptions params_;
};

ParamServer::ParamServer(const std::string & node_name) : Node(node_name)
{
  // Main EKF parameters
  RCLCPP_INFO_STREAM(get_logger(), "Reading General Estimator Parameters");
  params_.state_options.do_fej = declare_parameter("use_fej", params_.state_options.do_fej);
  params_.state_options.imu_avg = declare_parameter("use_imuavg", params_.state_options.imu_avg);
  params_.state_options.use_rk4_integration = declare_parameter("use_rk4int", params_.state_options.use_rk4_integration);
  params_.state_options.do_calib_lidar_imu_timeoffset = declare_parameter("calib_lidar_imu_timeoffset", params_.state_options.do_calib_lidar_imu_timeoffset);
  params_.state_options.do_calib_lidar_imu_extrinsic = declare_parameter("calib_lidar_imu_extrinsics", params_.state_options.do_calib_lidar_imu_extrinsic);
  params_.state_options.max_clone_size = declare_parameter("max_clones", params_.state_options.max_clone_size);

  params_.state_options.trans_x_noise_lidarimu = declare_parameter("state_init_x_noise_lidarimu", params_.state_options.trans_x_noise_lidarimu);
  params_.state_options.trans_y_noise_lidarimu = declare_parameter("state_init_y_noise_lidarimu", params_.state_options.trans_y_noise_lidarimu);
  params_.state_options.trans_z_noise_lidarimu = declare_parameter("state_init_z_noise_lidarimu", params_.state_options.trans_z_noise_lidarimu);
  params_.state_options.rot_x_noise_lidarimu = declare_parameter("state_init_rx_noise_lidarimu", params_.state_options.rot_x_noise_lidarimu);
  params_.state_options.rot_y_noise_lidarimu = declare_parameter("state_init_ry_noise_lidarimu", params_.state_options.rot_y_noise_lidarimu);
  params_.state_options.rot_z_noise_lidarimu = declare_parameter("state_init_rz_noise_lidarimu", params_.state_options.rot_z_noise_lidarimu);
  params_.state_options.time_offset_noise_lidarimu = declare_parameter("state_init_timeoffset_noise_lidarimu", params_.state_options.time_offset_noise_lidarimu);

  /// Filter initialization
  RCLCPP_INFO_STREAM(get_logger(), "Reading Filter Initialization Parameters");
  params_.init_window_time = declare_parameter("init_window_time", params_.init_window_time);
  params_.init_imu_thresh = declare_parameter("init_imu_thresh", params_.init_imu_thresh);

  /// Noise
  // Our noise values for inertial sensor
  RCLCPP_INFO_STREAM(get_logger(), "Reading IMU Noise Parameters");
  params_.imu_noises.sigma_w = declare_parameter("gyroscope_noise_density", params_.imu_noises.sigma_w);
  params_.imu_noises.sigma_a = declare_parameter("accelerometer_noise_density", params_.imu_noises.sigma_a);
  params_.imu_noises.sigma_wb = declare_parameter("gyroscope_random_walk", params_.imu_noises.sigma_wb);
  params_.imu_noises.sigma_ab = declare_parameter("accelerometer_random_walk", params_.imu_noises.sigma_ab);

  // Read update parameters
  RCLCPP_INFO_STREAM(get_logger(), "Reading Updater Chi2 Multiplier");
  params_.updaterOptions.chi2_multiplier = declare_parameter("updater_chi2_multiplier", params_.updaterOptions.chi2_multiplier);
  params_.updaterOptions.do_chi2_check = declare_parameter("updater_do_chi2_check", params_.updaterOptions.do_chi2_check);

  RCLCPP_INFO_STREAM(get_logger(), "Reading Rotation and Noise Update");
  params_.updaterOptions.noise_rotation = declare_parameter("updater_rotation_noise_lidarimu", params_.updaterOptions.noise_rotation);
  params_.updaterOptions.noise_translation = declare_parameter("updater_translation_noise_lidarimu", params_.updaterOptions.noise_translation);

  /// Global gravity
  RCLCPP_INFO_STREAM(get_logger(), "Reading Gravity");
  std::vector<double> gravity = {params_.gravity(0), params_.gravity(1), params_.gravity(2)};
  gravity = declare_parameter("gravity", gravity);
  params_.gravity << gravity.at(0), gravity.at(1), gravity.at(2);

  /// State
  // Timeoffset from lidar to IMU
  RCLCPP_INFO_STREAM(get_logger(), "Reading initial Timeoffset");
  params_.calib_lidar_imu_dt = declare_parameter("calib_lidar_imu_dt", params_.calib_lidar_imu_dt);

  /// NDT Resolution
  RCLCPP_INFO_STREAM(get_logger(), "Reading NDT Resolution");
  params_.ndt_resolution = declare_parameter("ndt_resolution", params_.ndt_resolution);

  /// Undistortion Flag
  RCLCPP_INFO_STREAM(get_logger(), "Reading Undistortion Flag");
  params_.do_undistortion = declare_parameter("do_undistortion", params_.do_undistortion);

  RCLCPP_INFO_STREAM(get_logger(), "Reading lin output file names");
  params_.inertial_trajectory_filename = declare_parameter("inertial_trajectory_filename", params_.inertial_trajectory_filename);
  params_.inertial_bias_filename = declare_parameter("inertial_bias_filename", params_.inertial_bias_filename);
  params_.inertial_velocity_filename = declare_parameter("inertial_velocity_filename", params_.inertial_velocity_filename);
  params_.lidar_inertial_calib_extrinsic_filename = declare_parameter("lidar_inertial_calib_extrinsic_filename", params_.lidar_inertial_calib_extrinsic_filename);
  params_.lidar_inertial_calib_dt_filename = declare_parameter("lidar_inertial_calib_dt_filename", params_.lidar_inertial_calib_dt_filename);

  RCLCPP_INFO_STREAM(get_logger(), "Reading lo output trajectory file name");
  params_.lidar_odometry_trajectory_filename = declare_parameter("lidar_odometry_trajectory_filename", params_.lidar_odometry_trajectory_filename);

  /// File to read the initial calibration result from
  RCLCPP_INFO_STREAM(get_logger(), "Reading the filename to read the initial calibration result to");
  params_.init_lidar_inertial_calibration_result_filename = declare_parameter("init_lidar_inertial_calibration_result_filename", params_.init_lidar_inertial_calibration_result_filename);

  std::ifstream initial_calib_lidar_inertial(params_.init_lidar_inertial_calibration_result_filename);
  std::string word_lidar;
  int i = 0;
  int j = 0;
  Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
  // Read in from ROS, and save into our eigen mat
  RCLCPP_INFO_STREAM(get_logger(), "Reading initial I_T_L");
  while (initial_calib_lidar_inertial >> word_lidar) {
    I_T_L(i, j) = atof(word_lidar.c_str());
    j++;
    if (j > 3) {
      j = 0;
      i++;
    }
  }
  /// Load these into our state
  Eigen::Matrix<double, 7, 1> lidar_imu_extrinsics_ITL;
  lidar_imu_extrinsics_ITL.block(0, 0, 4, 1) = rot_2_quat(I_T_L.block(0, 0, 3, 3));
  lidar_imu_extrinsics_ITL.block(4, 0, 3, 1) = I_T_L.block(0, 3, 3, 1);
  params_.lidar_imu_extrinsics = lidar_imu_extrinsics_ITL;

  /// File to write the final lidar inertial calibration result to
  RCLCPP_INFO_STREAM(get_logger(), "Reading the filename to write the final lidar inertial calibration result to");
  params_.lidar_inertial_calibration_result_filename = declare_parameter("lidar_inertial_calibration_result_filename", params_.lidar_inertial_calibration_result_filename);

  /// Limit map size params_
  params_.limit_map_size = declare_parameter("limit_map_size", params_.limit_map_size);
  params_.no_of_scans_for_map = declare_parameter("no_of_scans_for_map", params_.no_of_scans_for_map);

  params_.downSampleForMapping = declare_parameter("downsample_for_mapping", params_.downSampleForMapping);
}

const lidar_imu_calibration::CalibManagerOptions & ParamServer::getCalibManagerOptions() const
{
  return params_;
}

}  // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_PARSE_ROS_H
