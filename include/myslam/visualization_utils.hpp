#ifndef VISUALIZATION_UTILS_HPP
#define VISUALIZATION_UTILS_HPP

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace vis_utils
{
        template <class NodeT>
        inline visualization_msgs::msg::Marker toMarker(
            const std::string &frame,
            const std::string &ns,
            const double &scale,
            NodeT node)
        {
                visualization_msgs::msg::Marker marker;

                marker.header.frame_id = frame;
                marker.header.stamp = node->now();
                marker.ns = ns;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.w = 1.;
                marker.scale.x = scale;
                marker.scale.y = scale;
                marker.scale.z = scale;
                marker.color.r = 1.0;
                marker.color.g = 0;
                marker.color.b = 0.0;
                marker.color.a = 1.;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.lifetime = rclcpp::Duration::from_seconds(0);

                return marker;
        }
} // namespace vis_utils

#endif // VISUALIZATION_UTILS_HPP