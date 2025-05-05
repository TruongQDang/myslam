cmake_minimum_required(VERSION 3.8)
project(karto_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(TBB REQUIRED NO_CMAKE_PACKAGE_REGISTRY)

set(ros_dependencies
  rclcpp
  rclcpp_lifecycle
)

include_directories(include ${EIGEN3_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR}
                            ${BOOST_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(karto_myslam SHARED src/mapper.cpp src/karto.cpp)
ament_target_dependencies(karto_myslam ${ros_dependencies})
target_link_libraries(karto_myslam ${Boost_LIBRARIES} TBB::tbb)

install(DIRECTORY include/
	DESTINATION include/
)