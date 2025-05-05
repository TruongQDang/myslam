#ifndef MYSLAM_TYPES_HPP
#define MYSLAM_TYPES_HPP

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <Eigen/Geometry>
#include <unordered_map>
#include "math.hpp"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace myslam_types
{



/////////////////////////////////////////////////////////

/**
 * @brief 
 */

/////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////

/**
 * An array that can be resized as long as the size
 * does not exceed the initial capacity
 */


///////////////////////////////////////////////////////////////


typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;
typedef std::unordered_map<int, Eigen::Vector3d>::const_iterator ConstGraphIterator;
typedef std::unordered_map<int, Eigen::Vector3d>::iterator GraphIterator;

typedef std::vector<Pose2> Pose2Vector;
typedef Eigen::Matrix<int32_t, 2, 1> Vector2i;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Matrix3d Matrix3d;

} // namespace myslam_types

#endif // MYSLAM_TYPES_HPP