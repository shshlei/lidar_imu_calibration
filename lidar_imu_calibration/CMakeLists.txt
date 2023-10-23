cmake_minimum_required(VERSION 3.10)
project(lidar_imu_calibration)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find catkin (the ROS build system)
find_package(catkin QUIET COMPONENTS
  roscpp
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  ndt_omp
  tf
  imu_packet
  pcl_ros
  pcl_conversions
)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
#find_package(PCL REQUIRED)
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# Describe catkin project
catkin_package(
  CATKIN_DEPENDS roscpp rosbag ndt_omp tf std_msgs geometry_msgs sensor_msgs nav_msgs imu_packet
  INCLUDE_DIRS src
  LIBRARIES lidar_imu_calibration_lib
)

# Include our header files
include_directories(
  src
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  # ${PCL_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam
)

##################################################
# Make the core library
##################################################
add_library(lidar_imu_calibration_lib SHARED
  src/init/InertialInitializer.cpp
  src/state/State.cpp
  src/state/StateHelper.cpp
  src/state/Propagator.cpp
  src/update/UpdaterLidarOdometry.cpp
  src/track/lidarOdometry.cpp
  src/core/lincalibManager.cpp
)
target_link_libraries(lidar_imu_calibration_lib ${thirdparty_libraries})
target_include_directories(lidar_imu_calibration_lib PUBLIC src)

##################################################
# Adding different executables
##################################################
add_executable(lidar_imu_rotation_init src/lidar_imu_rotation_init.cpp)
target_link_libraries(lidar_imu_rotation_init lidar_imu_calibration_lib ${catkin_LIBRARIES})

add_executable(lidar_imu_rotation_optimizer src/lidar_imu_rotation_optimizer.cpp)
target_link_libraries(lidar_imu_rotation_optimizer lidar_imu_calibration_lib ${catkin_LIBRARIES})

add_executable(lidar_imu_calibration src/lidar_imu_calibration.cpp)
target_link_libraries(lidar_imu_calibration lidar_imu_calibration_lib ${catkin_LIBRARIES})
