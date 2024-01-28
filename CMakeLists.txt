cmake_minimum_required(VERSION 3.5)
project(helix_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(COMPILE_FLAGS -Ofast)

find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(OpenMP)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  ndt_omp # should before pcl_ros
  pcl_ros
  livox_ros_driver
)
add_compile_definitions(FMT_HEADER_ONLY)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Kontiki)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Kontiki/include
)

set_property(SOURCE DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PROPERTY COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")

add_library(${PROJECT_NAME}
  src/calib_result_writer.cpp
  src/ndt_aligner.cpp
  src/odom.cpp
  src/odom_aligner.cpp
  src/sensor.cpp
  src/surfel_map.cpp
  src/sensor_system.cpp
  src/trajectory.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

add_executable(calib_node
  src/calib_node.cpp
)

add_executable(path_helper
  src/path_helper.cpp
)

target_link_libraries(calib_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${CERES_LIBRARIES}
  fmt::fmt
  Kontiki
  glog
  # not link Boost
)

target_link_libraries(path_helper 
  ${catkin_LIBRARIES}
  fmt::fmt
)
