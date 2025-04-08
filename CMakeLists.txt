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
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread)

set(ros_dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  message_filters
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  geometry_msgs
  nav_msgs
)

# build all libraries as shared libraries
set(BUILD_SHARED_LIBS ON)

set(libraries
    mapper_utils
    myslam)

include_directories(include ${EIGEN3_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS})

add_library(mapper_utils src/mapper_utils.cpp)
ament_target_dependencies(mapper_utils ${ros_dependencies})
target_link_libraries(mapper_utils ${Boost_LIBRARIES})

add_library(myslam src/myslam.cpp)
target_link_libraries(myslam mapper_utils)
add_executable(myslam_node src/myslam_node.cpp)
target_link_libraries(myslam_node myslam)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs nav_msgs sensor_msgs) 
target_link_libraries(test_node mapper_utils)

install(TARGETS myslam_node
                ${libraries}
                test_node
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch description worlds config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
