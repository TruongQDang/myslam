#include "myslam/mapper_utils.hpp"
#include <cmath>

namespace mapper_utils
{

void CellUpdater::operator()(uint32_t index)
{
        uint8_t *data_ptr = occupancy_grid_->getDataPointer();
        uint32_t *cell_pass_cnt_ptr = occupancy_grid_->cell_pass_cnt_->getDataPointer();
        uint32_t *cell_hit_cnt_ptr = occupancy_grid_->cell_hit_cnt_->getDataPointer();

        occupancy_grid_->updateCell(&data_ptr[index], cell_pass_cnt_ptr[index], cell_hit_cnt_ptr[index]);
}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

template <class NodeT>
void Mapper::configure(const NodeT &node)
{
        double minimum_travel_distance = 0.5;
        if (!node->has_parameter("minimum_travel_distance"))
        {
                node->declare_parameter("minimum_travel_distance", minimum_travel_distance);
        }
        node->get_parameter("minimum_travel_distance", minimum_travel_distance);
        minimum_travel_distance_ = minimum_travel_distance;

        double minimum_travel_heading = 0.5;
        if (!node->has_parameter("minimum_travel_heading"))
        {
                node->declare_parameter("minimum_travel_heading", minimum_travel_heading);
        }
        node->get_parameter("minimum_travel_heading", minimum_travel_heading);
        minimum_travel_heading_ = minimum_travel_heading;
}

// explicit instantiation for the supported template types
template void Mapper::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &);

bool Mapper::process(LocalizedRangeScan *scan, Eigen::Matrix3d *covariance)
{
        if (scan != nullptr) {
                LaserRangeFinder *laser = scan->getLaserRangeFinder();

                if (initialized_ == false) {
                        // initialize mapper's utilities
                        std::cout << "still fine" << std::endl;
                        initialize(laser->getRangeThreshold());
                }

                // add scan to buffer and assign id
                std::cout << "traced from 4::0" << std::endl;
                scan_manager_->addScan(scan);
                std::cout << "traced from 4::1" << std::endl;
                scan_manager_->setLastScan(scan);
                std::cout << "traced from 4::2" << std::endl;

                return true;
        }
        std::cout << "traced from 4::3" << std::endl;
        return false;
}



} // namespace mapper_utils