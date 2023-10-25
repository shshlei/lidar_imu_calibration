// Created by usl on 3/6/21.

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <lidar_imu_calibration/msg/imu_packet.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <memory>
#include <iostream>
#include <fstream>

/// GTSAM Factor
using gtsam::symbol_shorthand::R;  // Rotation

class HECFactor : public gtsam::NoiseModelFactor1<gtsam::Rot3>
{
private:
  gtsam::Point3 m_axis_I_;
  gtsam::Point3 m_axis_L_;

public:
  HECFactor(gtsam::Key i, gtsam::Point3 axis_I, gtsam::Point3 axis_L, const gtsam::SharedNoiseModel & model)
  : gtsam::NoiseModelFactor1<gtsam::Rot3>(model, i), m_axis_I_(axis_I), m_axis_L_(axis_L) {}

  gtsam::Vector evaluateError(const gtsam::Rot3 & I_R_L, boost::optional<gtsam::Matrix &> H = boost::none) const
  {
    gtsam::Matrix H_Rp_R, H_Rp_p;
    gtsam::Point3 error = m_axis_I_ - I_R_L.rotate(m_axis_L_, H_Rp_R, H_Rp_p);
    if (H)
      (*H) = (gtsam::Matrix(3, 3) << -H_Rp_R).finished();
    return (gtsam::Vector(3) << error.x(), error.y(), error.z()).finished();
  }
};

class LidarImuCalibrationInitOptimizerNode : public rclcpp::Node
{
public:

  typedef message_filters::sync_policies::ApproximateTime<lidar_imu_calibration::msg::ImuPacket, geometry_msgs::msg::PoseStamped> SyncPolicy;

  LidarImuCalibrationInitOptimizerNode() : Node("lidar_imu_init_rotation_optimizer_node")
  {
    imupacket_sub_.subscribe(this, "/imu_packet");
    pose_sub_.subscribe(this, "/lidar_odometry");
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), imupacket_sub_, pose_sub_);
    sync_->registerCallback(&LidarImuCalibrationInitOptimizerNode::callback, this);

    accelerometer_noise_density_ = declare_parameter("accelerometer_noise_density", accelerometer_noise_density_);
    gyroscope_noise_density_ = declare_parameter("gyroscope_noise_density", gyroscope_noise_density_);
    max_frames_ = declare_parameter("max_frames", max_frames_);
    calibration_result_filename_ = declare_parameter("calibration_result_filename", calibration_result_filename_);
    RCLCPP_INFO_STREAM(get_logger(), "calibration_result_filename: " << calibration_result_filename_);

    double imuGravity = 9.81;
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * std::pow(accelerometer_noise_density_, 2);
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * std::pow(gyroscope_noise_density_, 2);  //
    p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * std::pow(1e-4, 2);                   // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
  }

  virtual ~LidarImuCalibrationInitOptimizerNode()
  {
    delete imuIntegratorOpt_;
    imuIntegratorOpt_ = nullptr;
  }

private:

  void callback(const lidar_imu_calibration::msg::ImuPacket::SharedPtr imupacket_msg, const geometry_msgs::msg::PoseStamped ::SharedPtr pose_msg)
  {
    for (std::size_t i = 1; i < imupacket_msg->stamps.size(); i++) {
      double dt = stamp2Sec(imupacket_msg->stamps[i]) - stamp2Sec(imupacket_msg->stamps[i - 1]);
      gtsam::Vector3 omega1 = gtsam::Vector3(imupacket_msg->gyroreadings[i - 1].x,
        imupacket_msg->gyroreadings[i - 1].y,
        imupacket_msg->gyroreadings[i - 1].z);
      gtsam::Vector3 omega2 = gtsam::Vector3(imupacket_msg->gyroreadings[i].x,
        imupacket_msg->gyroreadings[i].y,
        imupacket_msg->gyroreadings[i].z);

      gtsam::Vector3 accel1 = gtsam::Vector3(imupacket_msg->accelreadings[i - 1].x,
        imupacket_msg->accelreadings[i - 1].y,
        imupacket_msg->accelreadings[i - 1].z);
      gtsam::Vector3 accel2 = gtsam::Vector3(imupacket_msg->accelreadings[i].x,
        imupacket_msg->accelreadings[i].y,
        imupacket_msg->accelreadings[i].z);

      imuIntegratorOpt_->integrateMeasurement(0.5 * (accel1 + accel2), 0.5 * (omega1 + omega2), dt);
    }
    Eigen::Matrix3d deltaR_I = imuIntegratorOpt_->deltaRij().matrix();

    Eigen::Quaterniond quat_L;
    quat_L.x() = pose_msg->pose.orientation.x;
    quat_L.y() = pose_msg->pose.orientation.y;
    quat_L.z() = pose_msg->pose.orientation.z;
    quat_L.w() = pose_msg->pose.orientation.w;
    Eigen::Matrix3d deltaR_L(quat_L);

    Eigen::Vector3d axisAngle_lidar;
    Eigen::Vector3d axisAngle_imu;
    ceres::RotationMatrixToAngleAxis(deltaR_L.data(), axisAngle_lidar.data());
    ceres::RotationMatrixToAngleAxis(deltaR_I.data(), axisAngle_imu.data());

    /// GTSAM stuff
    graph_.add(boost::make_shared<HECFactor>(R(0), gtsam::Point3(axisAngle_imu.x(), axisAngle_imu.y(), axisAngle_imu.z()),
      gtsam::Point3(axisAngle_lidar.x(), axisAngle_lidar.y(), axisAngle_lidar.z()), rotationNoise_));
    RCLCPP_INFO_STREAM(get_logger(), "Frame: " << no_of_frames_ << " / " << max_frames_);
    if (no_of_frames_ == max_frames_) {
      solve();
    }
    no_of_frames_++;
    imuIntegratorOpt_->resetIntegration();
  }

  void solve()
  {
    gtsam::Rot3 priorRot = gtsam::Rot3::Identity();
    initial_values_.insert(R(0), priorRot);
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_values_).optimize();
    gtsam::Rot3 finalResult = result.at<gtsam::Rot3>(R(0));
    gtsam::Marginals marginals(graph_, result);
    std::cout << "Rot3: \n" << std::endl;
    std::cout << finalResult.matrix() << std::endl;
    std::cout << "Euler Angles: " << finalResult.matrix().eulerAngles(0, 1, 2).transpose() * 180 / M_PI << std::endl;
    std::cout << "Marginal Covariance" << std::endl;
    std::cout << marginals.marginalCovariance(R(0)) << std::endl;

    std::ofstream result_file;
    result_file.open(calibration_result_filename_.c_str());
    Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
    I_T_L.block(0, 0, 3, 3) = finalResult.matrix();
    I_T_L.block(0, 3, 3, 1) = Eigen::Vector3d::Zero();
    result_file << I_T_L;
    result_file.close();

    rclcpp::shutdown();
  }

  message_filters::Subscriber<lidar_imu_calibration::msg::ImuPacket> imupacket_sub_;

  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

private:

  template <typename T>
  double stamp2Sec(const T & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  gtsam::PreintegratedImuMeasurements * imuIntegratorOpt_{nullptr};

  double accelerometer_noise_density_ = 00001;

  double gyroscope_noise_density_ = 0.001;

  int no_of_frames_ = 0;

  int max_frames_ = 500;

  std::string calibration_result_filename_;

  /// GTSAM stuff
  gtsam::NonlinearFactorGraph graph_;

  gtsam::Values initial_values_;

  gtsam::noiseModel::Diagonal::shared_ptr rotationNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector3(1, 1, 1)));
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarImuCalibrationInitOptimizerNode>());
  rclcpp::shutdown();
  return 0;
}
