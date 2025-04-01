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
        // store scan header
        scan_header_ = scan->header;

        // get transform from odom to base
        Pose2 odom_pose;
        if (!pose_helper_->getOdomPose(odom_pose, scan->header.stamp)) {
                RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
                return;
        }

        if (shouldProcessScan(scan, odom_pose)) {
        }
           


}

} // namespace myslam