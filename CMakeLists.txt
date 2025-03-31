cmake_minimum_required(VERSION 3.8)
project(myslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

# build all libraries as shared libraries
set(BUILD_SHARED_LIBS ON)

set(libraries
    myslam_common
    myslam)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(myslam_common src/myslam_common.cpp)
ament_target_dependencies(myslam_common ${dependencies})

add_library(myslam src/myslam.cpp)
target_link_libraries(myslam myslam_common)
add_executable(myslam_node src/myslam_node.cpp)
target_link_libraries(myslam_node myslam)

install(TARGETS myslam_node
                ${libraries}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
