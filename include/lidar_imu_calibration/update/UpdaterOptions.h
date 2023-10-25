// Created by usl on 11/28/20.

#ifndef LIDAR_IMU_CALIBRATION_UPDATEROPTIONS_H
#define LIDAR_IMU_CALIBRATION_UPDATEROPTIONS_H

#include <stdio.h>
		
namespace lidar_imu_calibration
{
/// Struct which stores general updater options
struct UpdaterOptions
{
  /// What chi2_multiploer should we apply
  int chi2_multiplier = 5;

  /// Noise levels
  double noise_translation = 0.1;
  double noise_rotation = 0.1;
  /// Do chi2 check?
  bool do_chi2_check = true;

  /// Print function of what parameters we have loaded
  void print()
  {
    printf("\t- chi2 multiplier: %d\n", chi2_multiplier);
    printf("\t- Noise translation: %f\n", noise_translation);
    printf("\t- Noise rotation: %f\n", noise_rotation);
    printf("\t- Do chi2 check during update: %d\n", do_chi2_check);
  }
};
};      // namespace lidar_imu_calibration
#endif  // LIDAR_IMU_CALIBRATION_UPDATEROPTIONS_H
