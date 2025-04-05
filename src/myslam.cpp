#include "myslam/myslam.hpp"

namespace myslam
{

/*****************************************************************************/
MySlam::MySlam(rclcpp::NodeOptions options)
        : MySlamCommon(options)
/*****************************************************************************/
{
}

/*****************************************************************************/
void MySlam::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
        // get transform from odom to base
        scan_header_ = scan->header;
        Pose2 odom_pose;
        if (!pose_helper_->getPose(odom_pose, scan->header.stamp, odom_frame_, base_frame_)) {
                RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
                return;
        }

        // get details of laser
        if (!laser_->initialized()) {
                if (laser_->makeLaser(scan)) {
                        RCLCPP_WARN(get_logger(), "Failed to create laser device");
                        return;
                }
        } 

        if (shouldProcessScan(scan, odom_pose)) {
                addScan(scan, odom_pose);
        }
}

} // namespace myslam