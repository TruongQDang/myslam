#ifndef LASER_HELPER_HPP
#define LASER_HELPER_HPP

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "myslam/myslam_types.hpp"

namespace laser_utils
{

using namespace ::myslam_types;

/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized range scan relative to the robot.
 */
class LaserRangeFinder
{
public:
        template<class NodeT>
        LaserRangeFinder(NodeT node, tf2_ros::Buffer *tf, const std::string &base_frame);
        bool getLaserTransform(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);
private:
        geometry_msgs::msg::TransformStamped laser_pose_;
        std::string laser_frame_, base_frame_;
        rclcpp::Logger logger_;
        tf2_ros::Buffer *tf_;
};

class LocalizedRangeScan
{
public:
        LocalizedRangeScan()
        {
        }

        LocalizedRangeScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan, Pose2 &odom_pose);

        inline void setScanId(int32_t scan_id) 
        {
                scan_id_ = scan_id;
        } 

        inline int32_t getScanId()
        {
                return scan_id_;
        }
private:
        int32_t scan_id_;
}; 

} // namespace laser_utils


#endif // LASER_HELPER_HPP