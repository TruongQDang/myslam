#ifndef MYSLAM_TYPES_HPP
#define MYSLAM_TYPES_HPP

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <Eigen/Core>
#include <unordered_map>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace myslam_types
{

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;
typedef std::unordered_map<int, Eigen::Vector3d>::const_iterator ConstGraphIterator;
typedef std::unordered_map<int, Eigen::Vector3d>::iterator GraphIterator;

} // namespace myslam_types

#endif // MYSLAM_TYPES_HPP