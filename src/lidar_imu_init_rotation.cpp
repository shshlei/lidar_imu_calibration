// Created by usl on 3/5/21.

#define PCL_NO_PRECOMPILE  // !! BEFORE ANY PCL INCLUDE!!

#include "lidar_imu_calibration/track/lidarOdometry.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rcutils/time.h>
#include <rosbag2_cpp/reader.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lidar_imu_calibration/msg/imu_packet.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <memory>

class LidarImuCalibrationInitNode : public rclcpp::Node
{
public:

  LidarImuCalibrationInitNode() : Node("lidar_imu_init_rotation_node")
  {
    // Publisher
    imu_packet_publisher_ = create_publisher<lidar_imu_calibration::msg::ImuPacket>("/imu_packet", 1);
    lodom_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/lidar_odometry", 1);

    // Topics
    imu_topic_ = declare_parameter("imu_topic", "/imu");
    lidar_topic_ = declare_parameter("lidar_topic", "/lidar");
    ndt_resolution_ = declare_parameter("ndt_resolution", 0.5);

    // Bag
    bag_path_ = declare_parameter("bag_path", "");
    bag_start_ = declare_parameter("bag_start", 0);
    bag_durr_ = declare_parameter("bag_durr", -1);

    // Read Data
    reader_.open(bag_path_);
    const rosbag2_storage::BagMetadata & mdata = reader_.get_metadata();
    rcutils_time_point_value_t start = mdata.starting_time.time_since_epoch().count();
    if (bag_start_ > 0) start = start + RCUTILS_S_TO_NS(bag_start_);
    reader_.seek(start);
    rcutils_time_point_value_t end = start; 
    if (bag_durr_ > 0) end = start + RCUTILS_S_TO_NS(bag_durr_);

    lidar_imu_calibration::msg::ImuPacket imupacket;
    lodom_ = std::make_shared<lidar_imu_calibration::LidarOdometry>(ndt_resolution_, "", true);
    while (reader_.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
      if (msg->topic_name != lidar_topic_ && msg->topic_name != imu_topic_) continue;
      if (bag_durr_ > 0 && msg->time_stamp > end) break;

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (msg->topic_name == lidar_topic_) {
        sensor_msgs::msg::PointCloud2::SharedPtr s_lidar = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_serialization_.deserialize_message(&serialized_msg, s_lidar.get());

        lidar_imu_calibration::VPointCloud::Ptr cloud_pcl(new lidar_imu_calibration::VPointCloud);
        pcl::fromROSMsg(*s_lidar, *cloud_pcl);

        lodom_->feedScan(stamp2Sec(s_lidar->header.stamp), cloud_pcl);
        lodom_->append_and_update(true);

        Eigen::Matrix3d deltaR_L = lodom_->get_latest_relativePose().odometry_ij.block(0, 0, 3, 3);
        Eigen::Quaterniond delta_qL(deltaR_L);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = s_lidar->header.stamp;
        pose.pose.orientation.x = delta_qL.x();
        pose.pose.orientation.y = delta_qL.y();
        pose.pose.orientation.z = delta_qL.z();
        pose.pose.orientation.w = delta_qL.w();
        lodom_publisher_->publish(pose);

        imupacket.header.stamp = s_lidar->header.stamp;
        imu_packet_publisher_->publish(imupacket);
        imupacket.stamps.clear();
        imupacket.accelreadings.clear();
        imupacket.gyroreadings.clear();
      }
      else {
        sensor_msgs::msg::Imu::SharedPtr s_imu = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization_.deserialize_message(&serialized_msg, s_imu.get());

        imupacket.stamps.push_back(s_imu->header.stamp);

        geometry_msgs::msg::Vector3 accelReading;
        accelReading.x = s_imu->linear_acceleration.x;
        accelReading.y = s_imu->linear_acceleration.y;
        accelReading.z = s_imu->linear_acceleration.z;
        imupacket.accelreadings.push_back(accelReading);
        geometry_msgs::msg::Vector3 gyroReading;
        gyroReading.x = s_imu->angular_velocity.x;
        gyroReading.y = s_imu->angular_velocity.y;
        gyroReading.z = s_imu->angular_velocity.z;
        imupacket.gyroreadings.push_back(gyroReading);
      }
    }
    RCLCPP_INFO(get_logger(), "Finished collecting data.");
  }

private:

  template <typename T>
  double stamp2Sec(const T & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

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

  double ndt_resolution_;

  // initialize caliber
  lidar_imu_calibration::LidarOdometry::Ptr lodom_;

  rclcpp::Publisher<lidar_imu_calibration::msg::ImuPacket>::SharedPtr imu_packet_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lodom_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarImuCalibrationInitNode>());
  rclcpp::shutdown();
  return 0;
}
