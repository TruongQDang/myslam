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