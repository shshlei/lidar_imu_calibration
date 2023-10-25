// Created by usl on 11/6/20.

#ifndef LIDAR_IMU_CALIBRATION_STATE_H
#define LIDAR_IMU_CALIBRATION_STATE_H

#include "lidar_imu_calibration/state/StateOptions.h"
#include "lidar_imu_calibration/types/IMU.h"
#include "lidar_imu_calibration/types/Pose.h"
#include "lidar_imu_calibration/types/Type.h"
#include "lidar_imu_calibration/types/Vec.h"

#include <map>
#include <vector>
#include <iostream>

namespace lidar_imu_calibration
{
class State
{
public:
  State(const StateOptions & options_);
  virtual ~State() = default; 

  /// Returns timestep of the clone that we will marginalize
  double margtimestep()
  {
    double time = INFINITY;
    for (std::pair<const double, Pose *> & clone_imu : _clones_IMU) {
      if (clone_imu.first < time) {
        time = clone_imu.first;
      }
    }
    return time;
  }

  /// Calculates the current max size of the covariance
  int max_covariance_size() const
  {
    return (int)_Cov.rows();
  }

  /// Current timestamp (should be the last update time)
  double _timestamp;

  /// Struct constaining filter options
  StateOptions _options;

  /// Pointer to active IMU State (q_GtoI, p_IinG, V_IinG, bg, ba)
  IMU * _imu;

  /// Map between scanning times and clone poses (q_GtoIi, p_IiinG)
  std::map<double, Pose *> _clones_IMU;

  /// Time offset between base IMU to lidar (t_imu = t_lidar + t_offset)
  Vec * _calib_dt_LIDARtoIMU;

  /// Calib pose for IMU Lidar system I_R_L, I_t_L
  Pose * _calib_LIDARtoIMU;

private:
  /// Define that the state helper is a friend class of this class
  /// So that it can access the following private members
  friend class StateHelper;
  /// Covariance of all active variables
  Eigen::MatrixXd _Cov;
  /// Vector of variables
  std::vector<Type *> _variables;
};
} // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_STATE_H
