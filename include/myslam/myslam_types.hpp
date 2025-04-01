#ifndef MYSLAM_TYPES_HPP
#define MYSLAM_TYPES_HPP

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include <Eigen/Dense>
#include <cmath>

namespace myslam_types
{

/**
 * Defines a position (x, y) in 2-dimensional space and heading.
 */
class Pose2
{
public:
        /**
         * Default Constructor
         */
        Pose2()
            : heading_(0.0)
        {
        }

        /**
         * Constructor initializing pose parameters
         * @param position position
         * @param heading heading
         **/
        Pose2(const Eigen::Vector2d &position, double heading)
        : position_(position), heading_(heading) 
        {
        }

        /**
         * Constructor initializing pose parameters
         * @param x x-coordinate
         * @param y y-coordinate
         * @param heading heading
         **/
        Pose2(double x, double y, double heading)
            : position_(Eigen::Vector2d(x,y)), heading_(heading)
        {
        }

        /**
         * Returns the x-coordinate
         * @return the x-coordinate of the pose
         */
        inline double get_x() const
        {
                return position_.x();
        }

        /**
         * Returns the y-coordinate
         * @return the y-coordinate of the pose
         */
        inline double get_y() const
        {
                return position_.y();
        }

        /**
         * Returns the heading of the pose (in radians)
         * @return the heading of the pose
         */
        inline double get_heading() const
        {
                return heading_;
        }

        /**
         * Returns the position
         * @return the position of the pose
         */
        inline Eigen::Vector2d get_position() const
        {
                return position_;
        }

        /**
         * Return the squared distance between two Pose2
         * @return squared distance
         */
        inline double SquaredDistance(const Pose2 &other_pose) const
        {
                return (position_ - other_pose.position_).squaredNorm();
        }

private:
        Eigen::Vector2d position_;
        double heading_;
};


typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;



} // namespace myslam_types

#endif // MYSLAM_TYPES_HPP