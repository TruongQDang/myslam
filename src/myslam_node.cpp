#include <memory>
#include "myslam/myslam.hpp"

int main(int argc, char **argv)
{
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions options;
        auto myslam_node = std::make_shared<myslam::MySlam>(options);
        rclcpp::spin(myslam_node->get_node_base_interface());
        rclcpp::shutdown();
        return 0;
}
