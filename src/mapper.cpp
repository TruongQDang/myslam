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

bool Mapper::process(laser_utils::LocalizedRangeScan *scan, Eigen::Matrix3d *covariance)
{

        if (scan == nullptr)
                return false;

        laser_utils::LocalizedRangeScan *last_scan = scan_manager_->getLastScan();

        // add scan to buffer and assign id
        scan_manager_->addScan(scan);

        scan_manager_->setLastScan(scan);

        return true;
}


///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////

OccupancyGrid::OccupancyGrid(
        int32_t width, int32_t height,
        const Eigen::Vector2d &offset,
        double resolution)
{
        
}

} // namespace mapper_utils