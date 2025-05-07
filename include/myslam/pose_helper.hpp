#ifndef MYSLAM_POSE_HELPER_HPP
#define MYSLAM_POSE_HELPER_HPP

#include <tf2_ros/buffer.h>
#include <tf2/utils.hpp>

#include "karto_sdk/mapper.hpp"

namespace pose_utils
{

class PoseHelper
{
public:
        PoseHelper(tf2_ros::Buffer *tf)
            : tf_(tf)
        {
        }

        /**
         * Get transform between frames available in tf2 tree
         *
         * @param pose pose holder
         * @param t pose's time
         * @param target_frame T_a
         * @param source_frame T_b
         *
         * @return T_a_b, pose of frame b from frame a
         */
        bool getPose(
            karto::Pose2 &pose,
            const rclcpp::Time &t,
            const std::string &target_frame,
            const std::string &source_frame)
        {
                geometry_msgs::msg::TransformStamped tmp_pose;
                try
                {
                        tmp_pose = tf_->lookupTransform(
                            target_frame,
                            source_frame,
                            t);
                }
                catch (const tf2::TransformException &ex)
                {
                        return false;
                }

                const double yaw = tf2::getYaw(tmp_pose.transform.rotation);
                pose = karto::Pose2(
                        tmp_pose.transform.translation.x,
                        tmp_pose.transform.translation.y,
                        yaw);

                return true;
        }

private:
        tf2_ros::Buffer *tf_;
}; // PoseHelper

} //namespace pose_utils


#endif // MYSLAM_POSE_HELPER_HPP