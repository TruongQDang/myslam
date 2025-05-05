cmake_minimum_required(VERSION 3.8)
project(myslam)

set(CMAKE_BUILD_TYPE Debug) #None, Debug, Release, RelWithDebInfo, MinSizeRel
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
cmake_policy(SET CMP0077 NEW)

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
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

#karto_sdk lib
set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

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
  visualization_msgs
)

set(libraries
    ceres_solver_plugin
    myslam)

include_directories(include lib/karto_sdk/include
                            ${EIGEN3_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
                            ${CERES_INCLUDES})

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
ament_target_dependencies(ceres_solver_plugin ${ros_dependencies})
target_link_libraries(ceres_solver_plugin ${CERES_LIBRARIES}
                                          ${Boost_LIBRARIES})

#### Mapping executibles
add_library(myslam src/myslam.cpp src/loop_closure_assistant.cpp)
ament_target_dependencies(myslam
                          ${ros_dependencies})
target_link_libraries(myslam karto_myslam 
                             ceres_solver_plugin 
                             ${Boost_LIBRARIES})
add_executable(myslam_node src/myslam_node.cpp)
target_link_libraries(myslam_node myslam)


#### Install
install(TARGETS myslam_node
                ${libraries}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch description worlds config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
