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

        inline bool initialized() 
        {
                return intialized_;
        }

        bool makeLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);

private:
        geometry_msgs::msg::TransformStamped laser_pose_;
        std::string laser_frame_, base_frame_;
        rclcpp::Logger logger_;
        tf2_ros::Buffer *tf_;
        bool intialized_ = false;
};

class LocalizedRangeScan
{
public:
        LocalizedRangeScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);

        inline void setScanId(int32_t scan_id) 
        {
                scan_id_ = scan_id;
        } 

        inline int32_t getScanId()
        {
                return scan_id_;
        }

        /**
         * Gets the odometric pose of this scan
         * @return odometric pose of this scan
         */
        inline const Pose2 &getOdometricPose() const
        {
                return odom_pose_;
        }

        /**
         * Sets the odometric pose of this scan
         * @param pose
         */
        inline void setOdometricPose(const Pose2 &pose)
        {
                odom_pose_ = pose;
        }

        /**
         * Gets the (possibly corrected) robot pose at which this scan was taken.  The corrected robot pose of the scan
         * is usually set by an external module such as a localization or mapping module when it is determined
         * that the original pose was incorrect.  The external module will set the correct pose based on
         * additional sensor data and any context information it has.  If the pose has not been corrected,
         * a call to this method returns the same pose as GetOdometricPose().
         * @return corrected pose
         */
        inline const Pose2 &getCorrectedPose() const
        {
                return corrected_pose_;
        }

        /**
         * Moves the scan by moving the robot pose to the given location.
         * @param pose new pose of the robot of this scan
         */
        inline void setCorrectedPose(const Pose2 &pose)
        {
                corrected_pose_ = pose;
        }

private:
        int32_t scan_id_;
        Pose2 corrected_pose_;
        Pose2 odom_pose_;
}; 

} // namespace laser_utils


#endif // LASER_HELPER_HPP