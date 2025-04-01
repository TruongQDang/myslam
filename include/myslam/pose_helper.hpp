#ifndef POSE_HELPER_HPP
#define POSE_HELPER_HPP

#include <string>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "myslam/myslam_types.hpp"
#include "tf2/utils.hpp"

namespace pose_utils
{

using namespace ::myslam_types;
// helper to get the robots position
class GetPoseHelper
{
public:
	GetPoseHelper(
		tf2_ros::Buffer *tf,
		const std::string & base_frame,
		const std::string & odom_frame)
	: tf_(tf), base_frame_(base_frame), odom_frame_(odom_frame)
	{
	}

	bool getOdomPose(Pose2 &pose, const rclcpp::Time &t)
	{
		geometry_msgs::msg::TransformStamped base_ident, odom_pose;
		base_ident.header.stamp = t;
		base_ident.header.frame_id = base_frame_;
		base_ident.transform.rotation.w = 1.0;

		try
		{
			odom_pose = tf_->transform(base_ident, odom_frame_);
		}
		catch (tf2::TransformException &e)
		{
			return false;
		}

		const double yaw = tf2::getYaw(odom_pose.transform.rotation);
		pose = Pose2(odom_pose.transform.translation.x,
					  odom_pose.transform.translation.y, yaw);

		return true;
	}

private:
	tf2_ros::Buffer *tf_;
	std::string base_frame_, odom_frame_;
};

} // namespace pose_utils

#endif  // POSE_HELPER_HPP
