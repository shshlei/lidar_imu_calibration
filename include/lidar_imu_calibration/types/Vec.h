// Created by usl on 11/5/20.

#ifndef LIDAR_IMU_CALIBRATION_VEC_H
#define LIDAR_IMU_CALIBRATION_VEC_H

#include "lidar_imu_calibration/types/Type.h"

namespace lidar_imu_calibration
{
/**
 * @brief Derived Type class that implements vector variables
 */
class Vec : public Type
{
public:
  /**
   * @brief Default constructor for Vec
   * @param dim Size of the vector (will be same as error state)
   */
  Vec(int dim)
  : Type(dim)
  {
    _value = Eigen::VectorXd::Zero(dim);
    _fe = Eigen::VectorXd::Zero(dim);
  }

  ~Vec() {}

  /**
   * @brief Implements the update operation through standard vector addition
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd dx) override
  {
    assert(dx.rows() == _size);
    set_value(_value + dx);
  }

  /**
   * @brief Performs all the cloning
   */
  Type * clone() override
  {
    Type * Clone = new Vec(_size);
    Clone->set_value(value());
    Clone->set_fe(fe());
    return Clone;
  }
};
};  // namespace lidar_imu_calibration

#endif  // LIDAR_IMU_CALIBRATION_VEC_H
