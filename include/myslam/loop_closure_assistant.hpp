#ifndef LOOP_CLOSURE_ASSISTANT_HPP
#define LOOP_CLOSURE_ASSISTANT_HPP

#include "myslam/visualization_utils.hpp"
#include "karto_sdk/mapper.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace loop_closure_assistant
{

class LoopClosureAssistant
{
public:
        template <class NodeT>
        LoopClosureAssistant(
                NodeT node, 
                karto::Mapper *mapper,
                karto::ScanManager *scan_holder);
        void publishGraph();
        void setMapper(karto::Mapper *mapper);

private:
        karto::ScanManager *scan_holder_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        karto::Mapper *mapper_;
        karto::ScanSolver *solver_;

        std::string map_frame_;

        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Logger logger_;
        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
};

} // namespace loop_closure_assistant

#endif // LOOP_CLOSURE_ASSISTANT_HPP