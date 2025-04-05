#ifndef POSE_HELPER_HPP
#define POSE_HELPER_HPP

#include <string>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "myslam/myslam_types.hpp"
#include "tf2/utils.hpp"
#include <cmath>

namespace pose_utils
{

using namespace ::myslam_types;

class PoseHelper
{
public:
	PoseHelper(tf2_ros::Buffer *tf)
	: tf_(tf)
	{
	}

	bool getPose(
		Pose2 &pose, 
		const rclcpp::Time &t, 
		const std::string &from_frame, 
		const std::string &to_frame)
	{
		geometry_msgs::msg::TransformStamped tmp_pose;
		try {
			tmp_pose = tf_->lookupTransform(
				to_frame,
				from_frame,
				t);
		} catch (const tf2::TransformException &ex) {
			return false;
		}

		const double yaw = tf2::getYaw(tmp_pose.transform.rotation);
		pose = Pose2(tmp_pose.transform.translation.x,
					  tmp_pose.transform.translation.y, yaw);

		return true;
	}
private:
	tf2_ros::Buffer *tf_;
}; // PoseHelper

/**
 * Implementation of a Pose2 transform
 */
class Transform
{
public:
	/**
	 * Constructs a transformation from the origin to the given pose
	 * @param rPose pose
	 */
	Transform(const Pose2 &rPose) // NOLINT
	{
		SetTransform(Pose2(), rPose);
	}

	/**
	 * Constructs a transformation from the first pose to the second pose
	 * @param rPose1 first pose
	 * @param rPose2 second pose
	 */
	Transform(const Pose2 &rPose1, const Pose2 &rPose2)
	{
		SetTransform(rPose1, rPose2);
	}

public:
	/**
	 * Transforms the pose according to this transform
	 * @param rSourcePose pose to transform from
	 * @return transformed pose
	 */
	inline Pose2 TransformPose(const Pose2 &rSourcePose)
	{

	}

	/**
	 * Inverse transformation of the pose according to this transform
	 * @param rSourcePose pose to transform from
	 * @return transformed pose
	 */
	inline Pose2 InverseTransformPose(const Pose2 &rSourcePose)
	{

	}

private:
	/**
	 * Sets this to be the transformation from the first pose to the second pose
	 * @param rPose1 first pose
	 * @param rPose2 second pose
	 */
	void SetTransform(const Pose2 &rPose1, const Pose2 &rPose2)
	{
		
	}

private:
	// pose transformation
	Pose2 m_Transform;

	Eigen::Matrix3d m_Rotation;
	Eigen::Matrix3d m_InverseRotation;
}; // Transform

} // namespace pose_utils

#endif  // POSE_HELPER_HPP
