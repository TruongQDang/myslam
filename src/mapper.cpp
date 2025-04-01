#include "myslam/mapper.hpp"

namespace mapper_utils
{

template <class NodeT>
Mapper::Mapper(const NodeT &node)
{
        double minimum_travel_distance = 0.5;
        if (!node->has_parameter("minimum_travel_distance")) {
                node->declare_parameter("minimum_travel_distance", minimum_travel_distance);
        }
        node->get_parameter("minimum_travel_distance", minimum_travel_distance);
        minimum_travel_distance_ = minimum_travel_distance;

        double minimum_travel_heading = 0.5;
        if (!node->has_parameter("minimum_travel_heading")) {
                node->declare_parameter("minimum_travel_heading", minimum_travel_heading);
        }
        node->get_parameter("minimum_travel_heading", minimum_travel_heading);
        minimum_travel_heading_ = minimum_travel_heading;
}

} // namespace mapper_utils