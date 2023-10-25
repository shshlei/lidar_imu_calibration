cmake_minimum_required(VERSION 3.8)
project(lidar_imu_calibration)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)

# Set common system includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
  ${Boost_LIBRARIES}
  gtsam
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ImuPacket.msg" DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_add_library(lidar_imu_calibration_lib SHARED
  src/init/InertialInitializer.cpp
  src/state/State.cpp
  src/state/StateHelper.cpp
  src/state/Propagator.cpp
  src/update/UpdaterLidarOdometry.cpp
  src/track/lidarOdometry.cpp
  src/core/calibManager.cpp
)
target_link_libraries(lidar_imu_calibration_lib ${cpp_typesupport_target} ${thirdparty_libraries})

ament_auto_add_executable(lidar_imu_init_rotation_node src/lidar_imu_init_rotation.cpp)
target_link_libraries(lidar_imu_init_rotation_node lidar_imu_calibration_lib)

ament_auto_add_executable(lidar_imu_init_rotation_optimizer_node src/lidar_imu_init_rotation_optimizer.cpp)
target_link_libraries(lidar_imu_init_rotation_optimizer_node lidar_imu_calibration_lib)

ament_auto_add_executable(lidar_imu_calibration_node src/lidar_imu_calibration.cpp)
target_link_libraries(lidar_imu_calibration_node lidar_imu_calibration_lib)

ament_auto_package(INSTALL_TO_SHARE launch)
