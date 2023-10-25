// Created by usl on 12/8/20.

#ifndef LIDAR_IMU_CALIBRATION_CALIBMANAGER_H
#define LIDAR_IMU_CALIBRATION_CALIBMANAGER_H

#include "lidar_imu_calibration/utils/pcl_utils.h"
#include "lidar_imu_calibration/core/calibManagerOptions.h"
#include "lidar_imu_calibration/init/InertialInitializer.h"
#include "lidar_imu_calibration/state/Propagator.h"
#include "lidar_imu_calibration/state/State.h"
#include "lidar_imu_calibration/track/lidarOdometry.h"
#include "lidar_imu_calibration/update/UpdaterLidarOdometry.h"

#include <Eigen/Core>

#include <map>
#include <fstream>

namespace lidar_imu_calibration 
{
/// Core class that manages the entire system
class CalibManager
{
public:
  /// Constructor that will load all configuration variables
  CalibManager(const CalibManagerOptions & param_);

  /// Feed function for inertial data
  void feed_measurement_imu(double timestamp, const Eigen::Vector3d & wm, const Eigen::Vector3d & am);

  /// Feed function for lidar data
  void feed_measurement_lidar(double timestamp, const pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>::Ptr cloud_in);

  /// If we initialized or not
  bool initialized() const
  {
    return is_initialized_linkalibr;
  }

  /// Timestamp the system was initialized at
  double initialized_time() const
  {
    return startup_time;
  }

  /// Accessor to get the current state
  State * get_state() const
  {
    return state;
  }

  /// Accessor to get the current propagator
  Propagator * get_propagator() const
  {
    return propagator;
  }

  /// Accessor to Lidar Odometry object
  LidarOdometry::Ptr get_track_lidar() const
  {
    return LOdom;
  }

  /// Returns the last timestamp we have marginalized (true if we have a state)
  bool hist_last_marg_state(double & timestamp, Eigen::Matrix<double, 7, 1> & stateinG)
  {
    if (hist_last_marginalized_time != -1) {
      timestamp = hist_last_marginalized_time;
      stateinG = hist_stateinG.at(hist_last_marginalized_time);
      return true;
    }
    else {
      timestamp = -1;
      stateinG.setZero();
      return false;
    }
  }

  /// This will deskew a single point
  Eigen::Vector3d deskewPoint(const Eigen::Matrix<double, 13, 1> & start_point_state, const Eigen::Matrix<double, 13, 1> & current_point_state,
    const Eigen::Vector3d & skewedPoint, const Eigen::Matrix3d & I_R_L, const Eigen::Vector3d & I_t_L);

  /// This will deskew the entire scan/pointcloud
  void do_undistortion(double timestamp,
    const pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>::Ptr & scan_raw,
    pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>::Ptr & scan_out,
    Eigen::Matrix4d & T_ndt_predict);

  /// This function will try to initialize the state
  /// This function could also be repurposed to re-initialize the system after failure
  bool try_to_initialize();

  /// Boolean if we are initialized or not
  bool is_initialized_linkalibr = false;

  /// G_T_I1;
  Eigen::Matrix4d G_T_I1 = Eigen::Matrix4d::Identity();

  /// Get the time stamp of the first scan (used for building map)
  double get_map_time() const
  {
    return map_time;
  }

  /// Print State for debugging
  void printState();

protected:
  /// This will do propagation and updates
  void do_propagate_update(double timestamp);

  /// This will do planar target update
  void do_planar_update(double timestamp);

  /// The following will update our historical tracking information
  void update_keyframe_historical_information();

  /// Manager of parameters
  CalibManagerOptions params;

  /// Our master state object
  State * state;

  /// Propagator of our state
  Propagator * propagator;

  /// State initializer
  InertialInitializer * initializer;

  /// HEC Updater
  UpdaterLidarOdometry * updaterLidarOdometry;

  /// Lidar Odometry object (Tracker)
  LidarOdometry::Ptr LOdom;

  /// Track the distance travelled
  double timelastupdate = -1;
  double distance = 0;

  /// Start-up time of the filter
  double startup_time = -1;

  /// Historical information of the filter
  double hist_last_marginalized_time = -1;
  std::map<double, Eigen::Matrix<double, 7, 1> > hist_stateinG;

  std::ofstream trajfile_csv;
  std::ofstream bias_csv;
  std::ofstream velocity_csv;
  std::ofstream calib_extrinsic_csv;
  std::ofstream calib_dt_csv;

  /// For Surfel Association
  double map_time;
  bool first_propagation = true;

  /// Update flags
  bool did_update1 = false, did_update2 = false;
};
} // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_CALIBMANAGER_H
