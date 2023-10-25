// Created by usl on 12/14/20.

#define PCL_NO_PRECOMPILE  // !! BEFORE ANY PCL INCLUDE!!

#include "lidar_imu_calibration/core/calibManager.h"
#include "lidar_imu_calibration/utils/parse_ros.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rcutils/time.h>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>
#include <fstream>
#include <memory>

class LidarImuCalibrationNode : public lidar_imu_calibration::ParamServer
{
public:

  LidarImuCalibrationNode() : ParamServer("lidar_imu_calib_node")
  {
    sys_ = new lidar_imu_calibration::CalibManager(getCalibManagerOptions());

    imu_topic_ = declare_parameter("imu_topic", "/imu");
    lidar_topic_ = declare_parameter("lidar_topic", "/lidar");

    bag_path_ = declare_parameter("bag_path", "");
    bag_start_ = declare_parameter("bag_start", 0);
    bag_durr_ = declare_parameter("bag_durr", -1);

    // Read Data
    reader_.open(bag_path_);
    const rosbag2_storage::BagMetadata & mdata = reader_.get_metadata();
    rcutils_time_point_value_t start = mdata.starting_time.time_since_epoch().count() + RCUTILS_S_TO_NS(bag_start_);
    reader_.seek(start);
    rcutils_time_point_value_t end = start;
    if (bag_durr_ > 0) end = start + RCUTILS_S_TO_NS(bag_durr_);

    rclcpp::Time start_time = now();
    RCLCPP_INFO_STREAM(get_logger(), "Reading bag ...");
    while (reader_.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
      if (msg->topic_name != lidar_topic_ && msg->topic_name != imu_topic_) continue;
      if (bag_durr_ > 0 && msg->time_stamp > end) break;

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (msg->topic_name == lidar_topic_) {
        sensor_msgs::msg::PointCloud2::SharedPtr s_lidar = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_serialization_.deserialize_message(&serialized_msg, s_lidar.get());

        pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>::Ptr cloud(new pcl::PointCloud<lidar_imu_calibration::PointXYZIR8Y>);
        pcl::fromROSMsg(*s_lidar, *cloud);
        double time_lidar = stamp2Sec(s_lidar->header.stamp);
        sys_->feed_measurement_lidar(time_lidar, cloud);
      }
      else {
        sensor_msgs::msg::Imu::SharedPtr s_imu = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization_.deserialize_message(&serialized_msg, s_imu.get());

        double time_imu = stamp2Sec(s_imu->header.stamp);
        Eigen::Vector3d wm, am;
        wm << s_imu->angular_velocity.x, s_imu->angular_velocity.y, s_imu->angular_velocity.z;
        am << s_imu->linear_acceleration.x, s_imu->linear_acceleration.y, s_imu->linear_acceleration.z;
        sys_->feed_measurement_imu(time_imu, wm, am);
      }
    }

    rclcpp::Time end_time = now();
    RCLCPP_INFO_STREAM(get_logger(), "Reached end of bag");
    RCLCPP_INFO_STREAM(get_logger(), "Kalman Filtering took : " << stamp2Sec(end_time) - stamp2Sec(start_time) << " [s]");

    /// Write the final I_T_L to a text file
    lidar_imu_calibration::State * state = sys_->get_state();
    lidar_imu_calibration::Pose * calibration = state->_calib_LIDARtoIMU;
    Eigen::Matrix3d I_R_L = calibration->Rot();
    Eigen::Vector3d I_t_L = calibration->pos();
    Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
    I_T_L.block(0, 0, 3, 3) = I_R_L;
    I_T_L.block(0, 3, 3, 1) = I_t_L;

    RCLCPP_INFO_STREAM(get_logger(), "Writing KF calibration result to: " << getCalibManagerOptions().lidar_inertial_calibration_result_filename);
    std::ofstream result_calibration;
    result_calibration.open(getCalibManagerOptions().lidar_inertial_calibration_result_filename.c_str());
    result_calibration << I_T_L;
    result_calibration.close();

    std::cout << "I_T_L: " << std::endl;
    std::cout << I_T_L << std::endl;
    Eigen::Vector3d eulerXYZ = I_R_L.eulerAngles(0, 1, 2) * 180 / M_PI;
    RCLCPP_INFO_STREAM(get_logger(), "Translation [in m]: " << I_t_L.transpose());
    RCLCPP_INFO_STREAM(get_logger(), "Euler Angles [in deg]: " << eulerXYZ.transpose());
  }

private:

  template <typename T>
  double stamp2Sec(const T & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  template <typename T>
  double stamp2NanoSec(const T & stamp)
  {
    return RCUTILS_S_TO_NS(rclcpp::Time(stamp).seconds());
  }

  lidar_imu_calibration::CalibManager * sys_{nullptr};

  rosbag2_cpp::Reader reader_;

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization_;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization_;

  std::string imu_topic_;

  std::string lidar_topic_;

  /// Location of the ROS bag we want to read in
  std::string bag_path_;

  /// Get our start location and how much of the bag we want to play
  /// Make the bag duration < 0 to just process to the end of the bag
  double bag_start_, bag_durr_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarImuCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
