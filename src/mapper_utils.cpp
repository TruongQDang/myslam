#include "myslam/mapper_utils.hpp"

namespace mapper_utils
{

// template <class NodeT>
// Mapper::Mapper(const NodeT &node)
// {
//         double minimum_travel_distance = 0.5;
//         if (!node->has_parameter("minimum_travel_distance")) {
//                 node->declare_parameter("minimum_travel_distance", minimum_travel_distance);
//         }
//         node->get_parameter("minimum_travel_distance", minimum_travel_distance);
//         minimum_travel_distance_ = minimum_travel_distance;

//         double minimum_travel_heading = 0.5;
//         if (!node->has_parameter("minimum_travel_heading")) {
//                 node->declare_parameter("minimum_travel_heading", minimum_travel_heading);
//         }
//         node->get_parameter("minimum_travel_heading", minimum_travel_heading);
//         minimum_travel_heading_ = minimum_travel_heading;
// }

// bool Mapper::process(laser_utils::LocalizedRangeScan *scan, Eigen::Matrix3d *covariance)
// {

//         if (scan != nullptr) {
//                 laser_utils::LocalizedRangeScan *last_scan = scan_manager_->getLastScan();

//                 // if (m_Initialized == false)
//                 // {
//                 //         // initialize mapper with range threshold from device
//                 //         Initialize(pLaserRangeFinder->GetRangeThreshold());
//                 // }

//                 // Matrix3 cov;
//                 // cov.SetToIdentity();

//                 // // add scan to buffer and assign id
//                 // m_pMapperSensorManager->AddScan(pScan);

//                 // m_pMapperSensorManager->SetLastScan(pScan);

//                 return true;
//         }

//         return false;
// }

// OccupancyGrid *Mapper::getOccupancyGrid(const double &resolution)
// {
//         OccupancyGrid *occ_grid = nullptr;
//         return OccupancyGrid::createFromScans(
//                 getAllProcessedScans(),
//                 resolution, (uint32_t)getParamMinPassThrough(), (double)getParamOccupancyThreshold());
// }

// const std::vector<laser_utils::LocalizedRangeScan *> Mapper::getAllProcessedScans() const
// {
//         std::vector<laser_utils::LocalizedRangeScan *> all_scans;

//         if (scan_manager_ != nullptr)
//         {
//                 all_scans = scan_manager_->getAllScans();
//         }

//         return all_scans;
// }

// ///////////////////////////////////////////////////////////////////

// ///////////////////////////////////////////////////////////////////

// OccupancyGrid::OccupancyGrid(
//         int32_t width, int32_t height,
//         const Eigen::Vector2d &offset,
//         double resolution)
// {
        
// }


} // namespace mapper_utils