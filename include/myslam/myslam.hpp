#ifndef MYSLAM_HPP
#define MYSLAM_HPP

#include <boost/thread.hpp>
#include <memory>
#include <sys/resource.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "myslam/myslam_types.hpp"
#include "myslam/mapper_utils.hpp"
#include "myslam/ceres_solver.hpp"



namespace myslam 
{

using namespace ::myslam_types;
using mapper_utils::LocalizedRangeScan;

class MySlam : public rclcpp_lifecycle::LifecycleNode
{
public:
        explicit MySlam(rclcpp::NodeOptions);
        ~MySlam();

        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
protected:
        // threads
        void publishTransformLoop(const double &transform_publish_period);
        void publishVisualizations();

        // // setup
        void setParams();
        void setROSInterfaces();
        void setSolver();

        // callbacks
        void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

        // // functional bits
        bool shouldProcessScan(
                const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                const Pose2 &pose);
        mapper_utils::LocalizedRangeScan *addScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                Pose2 &odom_pose);
        tf2::Stamped<tf2::Transform> setTransformFromPoses(
                const Pose2 &corrected_pose,
                const Pose2 &odom_pose, const rclcpp::Time &t,
                const bool &update_reprocessing_transform);
        void publishPose(
                const Pose2 &pose,
                const Eigen::Matrix3d &cov,
                const rclcpp::Time &t);
        LocalizedRangeScan *getLocalizedRangeScan(
                mapper_utils::LaserRangeFinder *laser,
                const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                Pose2 &odom_pose);
        void makeLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);
        bool updateMap();

        // ROS stuff
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>> pose_publisher_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>> map_publisher_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::MapMetaData>> map_metadata_publisher_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>> scan_filter_subscriber_;
        std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

        // storage for ROS parameters
        std::string odom_frame_, map_frame_, base_frame_, map_name_, scan_topic_;
        std_msgs::msg::Header scan_header_;
        rclcpp::Duration transform_timeout_, minimum_time_interval_;
        int throttle_scans_, scan_queue_size_;

        double resolution_;
        double position_covariance_scale_;
        double yaw_covariance_scale_;
        bool first_measurement_;

        // // helpers
        std::unique_ptr<mapper_utils::PoseHelper> pose_helper_;
        std::unique_ptr<mapper_utils::Mapper> mapper_;
        std::unique_ptr<mapper_utils::LaserRangeFinder> laser_;

        // // internal state
        std::vector<std::unique_ptr<boost::thread>> threads_;
        tf2::Transform map_to_odom_;
        boost::mutex map_to_odom_mutex_, mapper_mutex_, pose_mutex_;
        nav_msgs::srv::GetMap::Response map_;

};

} // namespace myslam

#endif // MYSLAM_HPP