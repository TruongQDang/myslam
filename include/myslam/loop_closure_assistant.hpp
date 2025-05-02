#ifndef LOOP_CLOSURE_ASSISTANT_HPP
#define LOOP_CLOSURE_ASSISTANT_HPP

#include "myslam/myslam_types.hpp"
#include "myslam/visualization_utils.hpp"
#include "myslam/mapper_utils.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace loop_closure_assistant
{

using namespace ::myslam_types;

class LoopClosureAssistant
{
public:
        template <class NodeT>
        LoopClosureAssistant(
                NodeT node, 
                mapper_utils::Mapper *mapper,
                mapper_utils::ScanManager *scan_holder);
        void publishGraph();
        void setMapper(mapper_utils::Mapper *mapper);

private:
        mapper_utils::ScanManager *scan_holder_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        mapper_utils::Mapper *mapper_;
        mapper_utils::ScanSolver *solver_;

        std::string map_frame_;

        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Logger logger_;
        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
};

} // namespace loop_closure_assistant

#endif // LOOP_CLOSURE_ASSISTANT_HPP