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
                        initialize(laser->getRangeThreshold());
                }

                // get last scan
                LocalizedRangeScan *last_scan = scan_manager_->getLastScan();

                // update scans corrected pose based on last correction
                if (last_scan != nullptr) {
                        Pose2 T_map_odom = Pose2::getRelativePose(last_scan->getCorrectedPose(), last_scan->getOdometricPose());
                        scan->setCorrectedPose(
                                Pose2::transformPose(T_map_odom, 
                                scan->getOdometricPose()));
                }

                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();

                // correct scan (if not first scan)
                if (last_scan != NULL) {
                        Pose2 best_pose;
                        scan_matcher_->matchScan(
                                scan,
                                scan_manager_->getRunningScans(),
                                best_pose,
                                cov);
                        scan->setSensorPose(best_pose);
                        if (covariance) {
                                *covariance = cov;
                        }
                }

                // add scan to buffer and assign id
                scan_manager_->addScan(scan);

                // add to graph
                graph_->addVertex(scan);
                graph_->addEdges(scan, cov);

                scan_manager_->addRunningScan(scan);

                // if (m_pDoLoopClosing->GetValue())
                // {
                //         std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
                //         const_forEach(std::vector<Name>, &deviceNames)
                //         {
                //                 m_pGraph->TryCloseLoop(pScan, *iter);
                //         }
                // }

                scan_manager_->setLastScan(scan);

                return true;
        }
        
        return false;
}



} // namespace mapper_utils