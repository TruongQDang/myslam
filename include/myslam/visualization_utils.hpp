#ifndef VISUALIZATION_UTILS_HPP
#define VISUALIZATION_UTILS_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "karto_sdk/mapper.hpp"

#include "myslam/myslam_types.hpp"

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

        inline void toNavMap(
            const karto::OccupancyGrid *occ_grid,
            nav_msgs::msg::OccupancyGrid &map)
        {
                // Translate to ROS format
                int32_t width = occ_grid->getWidth();
                int32_t height = occ_grid->getHeight();
                Eigen::Vector2d offset = occ_grid->getCoordinateConverter()->getOffset();

                if (map.info.width != (unsigned int)width ||
                    map.info.height != (unsigned int)height ||
                    map.info.origin.position.x != offset.x() ||
                    map.info.origin.position.y != offset.y())
                {
                        map.info.origin.position.x = offset.x();
                        map.info.origin.position.y = offset.y();
                        map.info.width = width;
                        map.info.height = height;
                        map.data.resize(map.info.width * map.info.height);
                }

                for (int32_t y = 0; y < height; y++)
                {
                        for (int32_t x = 0; x < width; x++)
                        {
                                uint8_t value = occ_grid->getValue(Eigen::Matrix<int32_t, 2, 1>(x, y));
                                switch (value)
                                {
                                case karto::GRIDSTATES_UNKNOWN:
                                        map.data[MAP_IDX(map.info.width, x, y)] = -1;
                                        break;
                                case karto::GRIDSTATES_OCCUPIED:
                                        map.data[MAP_IDX(map.info.width, x, y)] = 100;
                                        break;
                                case karto::GRIDSTATES_FREE:
                                        map.data[MAP_IDX(map.info.width, x, y)] = 0;
                                        break;
                                }
                        }
                }
        }
} // namespace vis_utils

#endif // VISUALIZATION_UTILS_HPP