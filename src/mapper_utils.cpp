#include "myslam/mapper_utils.hpp"
#include <cmath>

namespace mapper_utils
{

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

OccupancyGrid *Mapper::getOccupancyGrid(const double &resolution)
{
        OccupancyGrid *occ_grid = nullptr;
        return OccupancyGrid::createFromScans(
                getAllProcessedScans(),
                resolution, 
                (uint32_t)getParamMinPassThrough(), 
                (double)getParamOccupancyThreshold());
}

const std::vector<LocalizedRangeScan *> Mapper::getAllProcessedScans() const
{
        std::vector<LocalizedRangeScan *> all_scans;

        if (scan_manager_ != nullptr)
        {
                std::cout << "about to get scans" << std::endl;
                all_scans = scan_manager_->getAllScans();
        }

        std::cout << "got " << all_scans.size() << " scans" << std::endl;

        return all_scans;
}


///////////////////////////////////////////////////////////////////

OccupancyGrid::OccupancyGrid(
        int32_t width, int32_t height,
        const Eigen::Vector2d &offset,
        double resolution)
        : Grid<uint8_t>(width, height),
        cell_pass_cnt_(Grid<uint32_t>::createGrid(0, 0, resolution)),
        cell_hit_cnt_(Grid<uint32_t>::createGrid(0, 0, resolution)),
        cell_updater_(nullptr)
{
        cell_updater_ = new CellUpdater(this);

        // if (karto::math::DoubleEqual(resolution, 0.0))
        // {
        //         throw Exception("Resolution cannot be 0");
        // }

        min_pass_through_ = 2;
        occupancy_threshold_ = 0.1;

        getCoordinateConverter()->setScale(1.0 / resolution);
        getCoordinateConverter()->setOffset(offset);
}

OccupancyGrid *OccupancyGrid::createFromScans(
        const std::vector<LocalizedRangeScan *> &scans,
        double resolution,
        uint32_t min_pass_through,
        double occupancy_threshold)
{
        if (scans.empty()) {
                return nullptr;
        }

        int32_t width, height;
        Eigen::Vector2d offset;
        computeGridDimensions(scans, resolution, width, height, offset);
        OccupancyGrid *pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
        pOccupancyGrid->setMinPassThrough(min_pass_through);
        pOccupancyGrid->setOccupancyThreshold(occupancy_threshold);
        std::cout << "width of occgrid: " << pOccupancyGrid->getWidth() << std::endl;
        std::cout << "height of occgrid: " << pOccupancyGrid->getHeight() << std::endl;
        pOccupancyGrid->createFromScans(scans);

        return pOccupancyGrid;
}

void OccupancyGrid::computeGridDimensions(
        const std::vector<LocalizedRangeScan *> &scans,
        double resolution,
        int32_t &width,
        int32_t &height,
        Eigen::Vector2d &offset)
{
        BoundingBox2 bounding_box;
        
        for (const auto &scan : scans) {
                if (scan == nullptr) {
                        continue;
                }

                bounding_box.add(scan->getBoundingBox());
        }

        double scale = 1.0 / resolution;
        Size2<double> size = bounding_box.getSize();

        width = static_cast<int32_t>(std::round(size.getWidth() * scale));
        height = static_cast<int32_t>(std::round(size.getHeight() * scale));
        offset = bounding_box.getMinimum();
}

void OccupancyGrid::createFromScans(const std::vector<LocalizedRangeScan *> &scans)
{
        cell_pass_cnt_->resize(getWidth(), getHeight());
        cell_pass_cnt_->getCoordinateConverter()->setOffset(getCoordinateConverter()->getOffset());
        cell_hit_cnt_->resize(getWidth(), getHeight());
        cell_hit_cnt_->getCoordinateConverter()->setOffset(getCoordinateConverter()->getOffset());

        for (const auto &scan_iter : scans) {
                if (scan_iter == nullptr) {
                        continue;
                }

                LocalizedRangeScan *scan = scan_iter;
                addScan(scan);
        }

        update();
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

///////////////////////////////////////////////////////

void CellUpdater::operator()(uint32_t index)
{
        uint8_t *data_ptr = occupancy_grid_->getDataPointer();
        uint32_t *cell_pass_cnt_ptr = occupancy_grid_->cell_pass_cnt_->getDataPointer();
        uint32_t *cell_hit_cnt_ptr = occupancy_grid_->cell_hit_cnt_->getDataPointer();

        occupancy_grid_->updateCell(&data_ptr[index], cell_pass_cnt_ptr[index], cell_hit_cnt_ptr[index]);
}

////////////////////////////////////////////////////////



} // namespace mapper_utils