#ifndef MYSLAM_TYPES_HPP
#define MYSLAM_TYPES_HPP

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <Eigen/Geometry>

namespace myslam_types
{

class Pose2
{
public:
        Pose2()
                : pose(Eigen::Isometry2d::Identity())
        {
        }

        Pose2(double x, double y, double heading)
        : Pose2()
        {
                pose.pretranslate(Eigen::Vector2d(x,y));
                pose.rotate(Eigen::Rotation2D<double>(heading));
        }

        inline double getX() const
        {
                return pose.translation().x(); 
        }

        inline double getY() const
        {
                return pose.translation().y();
        }

        inline double getHeading() const
        {
                return Eigen::Rotation2D<double>(pose.linear()).angle();
        }

        inline Eigen::Vector2d getPosition() const
        {
                return pose.translation();
        }

        inline double getSquaredDistance(const Pose2 &other) const
        {
                return (pose.translation() - other.pose.translation()).squaredNorm();
        }

        inline bool operator==(const Pose2 &other) const
        {
                return  pose.translation() == other.pose.translation() && this->getHeading() == other.getHeading();
        }

private:
        Eigen::Isometry2d pose;

}; // Pose2

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;


} // namespace myslam_types

#endif // MYSLAM_TYPES_HPP