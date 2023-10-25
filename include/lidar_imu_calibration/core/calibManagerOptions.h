// Created by usl on 12/8/20.

#ifndef LIDAR_IMU_CALIBRATION_CALIBMANAGEROPTIONS_H
#define LIDAR_IMU_CALIBRATION_CALIBMANAGEROPTIONS_H

#include "lidar_imu_calibration/state/StateOptions.h"
#include "lidar_imu_calibration/state/Propagator.h"
#include "lidar_imu_calibration/update/UpdaterOptions.h"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace lidar_imu_calibration 
{
/// Struct which stores all options needed for state estimation
struct CalibManagerOptions
{
  /// ESTIMATOR ===============================

  StateOptions state_options;

  // Amount of time we will initialize over (seconds)
  double init_window_time = 1.0;

  //  Variance threshold on our acceleration to be classified as moving
  double init_imu_thresh = 1.0;

  // This function will print out all estimator settings loaded.
  void print_estimator()
  {
    printf("ESTIMATOR PARAMETERS:\n");
    state_options.print();
    printf("\t- init_window_time: %.2f\n", init_window_time);
    printf("\t- init_imu_thresh: %.2f\n", init_imu_thresh);
  }

  /// NOISE / CHI2 ============================
  // IMU noise (gyroscope and accelerometer)
  Propagator::NoiseManager imu_noises;

  // Update options for estimator (noise and chi2 multiplier)
  UpdaterOptions updaterOptions;

  // This function will print out all noise parameters loaded.
  void print_noise()
  {
    printf("NOISE PARAMETERS:\n");
    imu_noises.print();
    printf("\tUpdater Estimator Feats:\n");
    updaterOptions.print();
  }

  /// STATE DEFAULTS ============================
  // Gravity in global frame
  Eigen::Vector3d gravity = {0.0, 0.0, 9.81};

  // Time offset between lidar and IMU
  double calib_lidar_imu_dt = 0.0;
  // Lidar IMU extrinsics (q_LtoI, p_LinI). Note the
  // difference between "to" and "in"
  Eigen::Matrix<double, 7, 1> lidar_imu_extrinsics;

  // This function will print out all state defaults loaded.
  void print_state()
  {
    printf("STATE PARAMETERS:\n");
    printf("\t- gravity: %.3f, %.3f, %.3f\n", gravity(0), gravity(1), gravity(2));
    printf("\t- calib_lidar_imu_dt: %.4f\n", calib_lidar_imu_dt);
    std::cout << "lidar_imu_extrinsic(0:3):" << std::endl
              << lidar_imu_extrinsics.block(0, 0, 4, 1).transpose() << std::endl;
    std::cout << "lidar_imu_extrinsic(4:6):" << std::endl
              << lidar_imu_extrinsics.block(4, 0, 3, 1).transpose() << std::endl;
  }

  /// LIDAR Odometry (Tracker)
  // Resolution of space in 3D for doing NDT based matching
  double ndt_resolution = 0.5;

  /// Pose Tracking
  void print_trackers()
  {
    printf("LIDAR Odometry PARAMETERS:\n");
    printf("\t- ndt_resolution: %.3f\n", ndt_resolution);
  }

  bool do_undistortion = true;

  /// CSV file as output
  std::string inertial_trajectory_filename;
  std::string inertial_bias_filename;
  std::string inertial_velocity_filename;
  std::string lidar_inertial_calib_extrinsic_filename;
  std::string lidar_inertial_calib_dt_filename;
  std::string lidar_odometry_trajectory_filename;

  /// Initial lidar inertial calibration result filename
  std::string init_lidar_inertial_calibration_result_filename;

  /// Final lidar inertial calibration result filename
  std::string lidar_inertial_calibration_result_filename;

  /// Map size params
  bool limit_map_size = true;
  int no_of_scans_for_map = 100;

  /// Map clouddownsampling flag
  bool downSampleForMapping = false;
};
} // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_CALIBMANAGEROPTIONS_H
