#include "myslam/mapper_utils.hpp"

namespace mapper_utils
{

template <class NodeT>
Mapper::Mapper(const NodeT &node)
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

template Mapper::Mapper(const rclcpp_lifecycle::LifecycleNode::SharedPtr &);

bool Mapper::process(LocalizedRangeScan *scan, Eigen::Matrix3d *covariance)
{

        if (scan != nullptr) {

                // add scan to buffer and assign id
                scan_manager_->addScan(scan);

                scan_manager_->setLastScan(scan);

                return true;
        }

        return false;
}

OccupancyGrid *Mapper::getOccupancyGrid(const double &resolution)
{
        OccupancyGrid *occ_grid = nullptr;
        return OccupancyGrid::createFromScans(
                getAllProcessedScans(),
                resolution, (uint32_t)getParamMinPassThrough(), (double)getParamOccupancyThreshold());
}

const std::vector<LocalizedRangeScan *> Mapper::getAllProcessedScans() const
{
        std::vector<LocalizedRangeScan *> all_scans;

        if (scan_manager_ != nullptr)
        {
                all_scans = scan_manager_->getAllScans();
        }

        return all_scans;
}


///////////////////////////////////////////////////////////////////

OccupancyGrid::OccupancyGrid(
        int32_t width, int32_t height,
        const Eigen::Vector2d &offset,
        double resolution)
{
        
}

OccupancyGrid *OccupancyGrid::createFromScans(
        const std::vector<LocalizedRangeScan *> &scans,
        double resolution,
        uint32_t min_pass_through,
        double occupancy_threshold)
{
        if (scans.empty())
        {
                return nullptr;
        }

        int32_t width, height;
        Eigen::Vector2d offset;
        ComputeDimensions(scans, resolution, width, height, offset);
        OccupancyGrid *pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
        pOccupancyGrid->setMinPassThrough(min_pass_through);
        pOccupancyGrid->setOccupancyThreshold(occupancy_threshold);
        pOccupancyGrid->createFromScans(scans);

        return pOccupancyGrid;
}

void OccupancyGrid::ComputeDimensions(
        const std::vector<LocalizedRangeScan *> &rScans,
        double resolution,
        int32_t &rWidth,
        int32_t &rHeight,
        Eigen::Vector2d &rOffset)
{
        
}

void OccupancyGrid::createFromScans(const std::vector<LocalizedRangeScan *> &rScans)
{
        // m_pCellPassCnt->Resize(GetWidth(), GetHeight());
        // m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

        // m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
        // m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

        // const_forEach(LocalizedRangeScanVector, &rScans)
        // {
        //         if (*iter == nullptr)
        //         {
        //                 continue;
        //         }

        //         LocalizedRangeScan *pScan = *iter;
        //         AddScan(pScan);
        // }

        // Update();
}

///////////////////////////////////////////////////////////////////

LocalizedRangeScan::LocalizedRangeScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
{
        range_readings_ = std::make_unique<double[]>(scan->ranges.size());
        std::copy(scan->ranges.begin(), scan->ranges.end(), range_readings_.get());
}

//////////////////////////////////////////////////////////////////

bool PoseHelper::getPose(Pose2 &pose,
        const rclcpp::Time &t,
        const std::string &from_frame,
        const std::string &to_frame)
{
        geometry_msgs::msg::TransformStamped tmp_pose;
        try
        {
                tmp_pose = tf_->lookupTransform(
                    to_frame,
                    from_frame,
                    t);
        }
        catch (const tf2::TransformException &ex)
        {
                return false;
        }

        const double yaw = tf2::getYaw(tmp_pose.transform.rotation);
        pose = Pose2(tmp_pose.transform.translation.x,
                     tmp_pose.transform.translation.y, yaw);

        return true;
}

} // namespace mapper_utils