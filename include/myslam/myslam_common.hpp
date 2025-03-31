#ifndef MYSLAM_COMMON_HPP
#define MYSLAM_COMMON_HPP

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "myslam/myslam_types.hpp"



namespace myslam 
{

using namespace ::myslam_types;

class MySlamCommon : public rclcpp_lifecycle::LifecycleNode
{
public:
        explicit MySlamCommon(rclcpp::NodeOptions);
        virtual ~MySlamCommon();

        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
protected:
        // threads
        void publishTransformLoop(const double &transform_publish_period);

        // callbacks
        // virtual void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) = 0;

        // ROS stuff
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
};

} // namespace myslam

#endif // MYSLAM_COMMON_HPP