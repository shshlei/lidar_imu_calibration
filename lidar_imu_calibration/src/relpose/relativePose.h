//
// Created by usl on 11/29/20.
//

#ifndef LIN_CORE_RELATIVEPOSE_H
#define LIN_CORE_RELATIVEPOSE_H

#include "types/Pose.h"

#include <Eigen/Eigen>

#include <iostream>
#include <vector>

using namespace lin_type;

namespace lin_core
{
class relativePose
{
public:
  /// Time stamp of scan i
  double timestamp_i;
  /// Time stamp of scan j
  double timestamp_j;
  /// Odometry pose
  Eigen::Matrix4d odometry_ij;
};
}  // namespace lin_core
#endif  //LIN_CORE_RELATIVEPOSE_H
