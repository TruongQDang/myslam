#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "std_msgs/msg/string.hpp"
#include "myslam/mapper_utils.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using mapper_utils::LocalizedRangeScan;
using namespace ::myslam_types;

class OccMap : public rclcpp::Node
{
public:
        OccMap()
                : Node("occupancy_grid_map"), count_(0)
        {
                map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
                publisher_ = this->create_publisher<std_msgs::msg::String>("test_msg", 10);

                T_w_scan_ = Pose2();
                laser_scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
                laser_scan_msg_->ranges = {81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 81.91, 1.64, 1.64, 1.61, 1.61, 1.61, 1.62, 1.68, 1.62, 3.05, 3.03, 3.17, 3.15, 3.28, 3.26, 3.41, 3.4, 3.56, 3.55, 3.56, 3.57, 3.51, 3.52, 3.48, 3.49, 3.46, 3.46, 3.43, 3.43, 3.41, 3.41, 3.39, 3.4, 3.37, 3.37, 2.78, 3.01, 3.32, 3.09, 5.9, 5.85, 6.37, 6.37, 6.25, 6.25, 6.39, 6.28, 6.8, 6.89, 6.54, 6.85, 6.21, 6.21, 6.2, 6.2, 6.19, 6.19, 6.21, 6.21, 4.21, 4.3, 4.19, 4.19, 4.18, 4.19, 4.2, 4.19, 4.19, 4.19, 4.19, 4.2, 2.52, 2.56, 2.49, 2.49, 4.21, 4.2, 4.21, 4.21, 4.22, 4.23, 4.23, 4.23, 4.24, 4.24, 4.25, 4.25, 4.27, 4.27, 4.28, 4.29, 4.29, 4.3, 4.32, 4.32, 4.34, 4.34, 4.36, 4.37, 4.38, 4.39, 4.42, 4.42, 4.44, 4.45, 3.04, 3.04, 2.53, 2.53, 2.63, 2.51, 4.53, 2.79, 4.56, 4.56, 4.72, 4.72, 4.82, 4.79, 3.04, 3.04, 4.91, 4.91, 2.66, 4.96, 2.63, 2.64, 5.06, 5.06, 5.09, 5.08, 5.22, 5.19, 6.89, 6.91, 6.44, 6.47, 6.34, 6.36, 6.22, 6.23, 6.05, 6.08, 5.82, 5.83, 5.74, 5.75, 5.67, 5.68, 5.67, 5.66, 5.67, 5.68, 5.57, 5.58, 5.48, 5.48, 5.38, 5.39, 5.3, 5.31, 5.22, 5.23, 4.47, 4.48, 4.42, 4.44, 4.36, 4.36, 4.29, 4.3, 4.24, 4.25, 4.19, 4.2, 4.13, 4.14, 4.09, 4.09, 4.04, 4.05, 4, 4, 3.96, 3.96, 3.92, 3.92, 3.88, 3.89, 3.85, 3.86, 3.81, 3.82, 3.78, 3.78, 3.75, 3.75, 3.7, 3.7, 3.68, 3.68, 3.65, 3.66, 3.63, 3.63, 3.6, 3.61, 3.59, 3.59, 3.57, 3.57, 3.54, 3.55, 3.53, 3.53, 3.51, 3.52, 3.53, 3.53, 3.92, 3.91, 4, 4, 3.98, 3.99, 3.97, 3.97, 3.97, 3.97, 4.46, 4.46, 5.43, 5.43, 6.22, 6.19, 7.66, 7.63, 6.07, 6.07, 6.21, 6.2, 6.08, 6.07, 6.14, 6.14, 6.26, 6.34, 6.19, 6.24, 11.84, 11.84, 11.87, 11.88, 11.58, 11.57, 2.65, 2.65, 2.62, 2.62, 2.63, 2.63, 2.61, 2.61, 2.69, 2.7, 2.83, 2.83, 3.44, 3.45, 3.23, 3.23, 3.25, 3.25, 3.21, 3.22, 3.2, 3.21, 3.18, 3.18, 3.15, 3.15, 3.13, 3.13, 3.1, 3.11, 3.09, 3.08, 3.05, 3.05, 3.05, 3.05, 3.03, 3.03, 3.02, 3.02, 3, 3, 2.99, 2.99, 2.97, 2.97, 2.97, 2.96, 2.96, 2.96, 2.95, 2.94, 2.94, 2.94, 2.94, 2.94, 2.93, 2.93, 2.92, 2.92, 2.93, 2.93, 2.92, 2.92, 2.92, 2.92, 2.14, 2.12};

                laser_ = new mapper_utils::LaserRangeFinder();
                laser_->setMaximumRange(20.0);
                laser_->setMinimumRange(0.3);
                laser_->setRangeThreshold(15.0);
                laser_->setAngularResolution(0.0087270);
                laser_->setMinimumAngle(-1.5708);
                // laser_->setNumberOfRangeReadings(laser_scan_msg_->ranges.size());

                scan_ = new LocalizedRangeScan(laser_scan_msg_, laser_);
                scan_->setCorrectedPose(T_w_scan_);
                scan_->setOdometricPose(T_w_scan_);

                scans_.push_back(scan_);

                auto map_callback = [this]() -> void
                {
                        double resolution = 0.05;
                        uint32_t min_pass_through = 2;
                        double occupancy_threshold = 0.1;

                        mapper_utils::OccupancyGrid *occ_grid = mapper_utils::OccupancyGrid::createFromScans(scans_, resolution, min_pass_through, occupancy_threshold);

                        auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();

                        occupancy_grid_msg.info.resolution = resolution;
                        occupancy_grid_msg.info.origin.position.x = 0.0;
                        occupancy_grid_msg.info.origin.position.y = 0.0;
                        occupancy_grid_msg.info.origin.position.z = 0.0;
                        occupancy_grid_msg.info.origin.orientation.x = 0.0;
                        occupancy_grid_msg.info.origin.orientation.y = 0.0;
                        occupancy_grid_msg.info.origin.orientation.z = 0.0;
                        occupancy_grid_msg.info.origin.orientation.w = 1.0;
                        occupancy_grid_msg.header.frame_id = "map";

                        
                        mapper_utils::toNavMap(occ_grid, occupancy_grid_msg);
                        occupancy_grid_msg.header.stamp = rclcpp::Clock().now();

                        // occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
                        // occupancy_grid_msg.info.width = 3;
                        // occupancy_grid_msg.info.height = 3;
                        // occupancy_grid_msg.info.resolution = 1;
                        // occupancy_grid_msg.info.origin.position.x = -1;
                        // occupancy_grid_msg.info.origin.position.y = -1;
                        // occupancy_grid_msg.header.frame_id = "map";

                        // occupancy_grid_msg.data = {-1, 0, 0, 0, 100, 0, 0, 0, 0};

                        map_pub_->publish(occupancy_grid_msg);

                        auto message = std_msgs::msg::String();
                        message.data = "Hello, world! " + std::to_string(this->count_++);
                        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                        this->publisher_->publish(message);
                };

                timer_ = this->create_wall_timer(500ms, map_callback);
        }

private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;

        Pose2 T_w_scan_;
        std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_msg_;
        mapper_utils::LaserRangeFinder *laser_;
        LocalizedRangeScan *scan_;
        std::vector<LocalizedRangeScan *> scans_;
};

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OccMap>());
        rclcpp::shutdown();
        return 0;
}