#ifndef MYSLAM_COMMON_HPP
#define MYSLAM_COMMON_HPP

#include <boost/thread.hpp>
#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.h"

#include "myslam/myslam_types.hpp"
#include "myslam/pose_helper.hpp"
#include "myslam/laser_helper.hpp"
#include "myslam/mapper.hpp"



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
        void publishVisualizations();

        // setup
        void setParams();
        void setROSInterfaces();

        // functional bits
        bool shouldProcessScan(
                const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                const Pose2 &pose);

        // callbacks
        virtual void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) = 0;

        // helpers
        std::unique_ptr<pose_utils::GetPoseHelper> pose_helper_;
        std::unique_ptr<laser_utils::LaserRangeFinder> laser_;
        std::unique_ptr<mapper_utils::Mapper> mapper_;

        // internal state
        std::vector<std::unique_ptr<boost::thread>> threads_;

        // ROS stuff
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
                                                    rclcpp_lifecycle::LifecycleNode>>
                scan_filter_sub_;
        std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

        // storage for ROS parameters
        std::string odom_frame_, map_frame_, base_frame_, map_name_, scan_topic_;
        std_msgs::msg::Header scan_header_;
        rclcpp::Duration transform_timeout_, minimum_time_interval_;
        int throttle_scans_, scan_queue_size_;

        bool first_measurement_;
};

} // namespace myslam

#endif // MYSLAM_COMMON_HPP