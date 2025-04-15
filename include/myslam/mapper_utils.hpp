#ifndef MAPPER_UTILS_HPP
#define MAPPER_UTILS_HPP

#include <unordered_map>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <cstring>
#include <iostream>
#include <cmath>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/time.hpp"

#include "myslam/myslam_types.hpp"
#include "myslam/math.hpp"


namespace mapper_utils
{

class Mapper;
class ScanManager;
class ScanMatcher;
template <typename T>
class Edge;
class LaserRangeFinder;
class OccupancyGrid;
class CellUpdater;
class CorrelationGrid;
class LocalizedRangeScan;
class PoseHelper;
class CoordinateConverter;
class LaserRangeFinder;

using namespace ::myslam_types;

////////////////////////////////////////////////////////////

class LaserRangeFinder
{
private:
        double range_threshold_;

        double minimum_range_;
        double maximum_range_;

        double minimum_angle_;
        double maximum_angle_;

        double angular_resolution_;

        uint32_t number_of_range_readings_;

        Pose2 offset_pose_;

        std::string frame_id_;

public:
        LaserRangeFinder()
        {
        }

        LaserRangeFinder(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan, const Pose2& offset_pose)
        {
                frame_id_ = scan->header.frame_id;
                offset_pose_ = offset_pose;
                minimum_range_ = scan->range_min;
                maximum_range_ = scan->range_max;
                minimum_angle_ = scan->angle_min;
                maximum_angle_ = scan->angle_max;
                angular_resolution_ = scan->angle_increment;
        }

        inline void setFrameId(std::string frame_id)
        {
                frame_id_ = frame_id;
        }

        inline void setMinimumRange(double minimum_range)
        {
                minimum_range_ = minimum_range;
        }

        inline double getMinimumRange() const
        {
                return minimum_range_;
        }

        inline void setMaximumRange(double maximum_range)
        {
                maximum_range_ = maximum_range;
        }

        inline double getMaximumRange() const
        {
                return maximum_range_;
        }

        inline double getRangeThreshold() const
        {
                return range_threshold_;
        }

        inline void setRangeThreshold(double range_threshold)
        {
                range_threshold_ = range_threshold;
        }

        inline void setMaximumAngle(double maximum_angle)
        {
                maximum_angle_ = maximum_angle;
                updateNumberOfRangeReadings();
        }

        inline double getMaximumAngle()
        {
                return maximum_angle_;
        }

        inline void setMinimumAngle(double minimum_angle)
        {
                minimum_angle_ = minimum_angle;
                updateNumberOfRangeReadings();
        }

        inline double getMinimumAngle() const
        {
                return minimum_angle_;
        }


        inline void setAngularResolution(double angular_resolution)
        {
                angular_resolution_ = angular_resolution;
                updateNumberOfRangeReadings();
        }

        inline double getAngularResolution() const
        {
                return angular_resolution_;
        }

        inline uint32_t getNumberOfRangeReadings() const
        {
                return number_of_range_readings_;
        }

        inline void setOffsetPose(const Pose2 &offset_pose)
        {
                offset_pose_ = offset_pose;
        }

        inline const Pose2 &getOffsetPose() const
        {
                return offset_pose_;
        }

        void updateNumberOfRangeReadings()
        {
                number_of_range_readings_ = static_cast<uint32_t>(std::round(
                        (getMaximumAngle() -  
                        getMinimumAngle()) /
                        getAngularResolution()));
        }
}; // LaserRangeFinder

////////////////////////////////////////////////////////////

typedef std::vector<double> RangeReadingsVector;

class LocalizedRangeScan
{
private:
        uint32_t scan_id_;
        Pose2 corrected_pose_;
        Pose2 odom_pose_;
        /**
         * Average of all the point readings
         */
        Pose2 barycenter_pose_;
        std::unique_ptr<double[]> range_readings_;
        uint32_t number_of_range_readings_;
        double time_;
        BoundingBox2 bounding_box_;
        bool is_dirty_;
        std::vector<Eigen::Vector2d> point_readings_;
        std::vector<Eigen::Vector2d> unfiltered_point_readings_;
        LaserRangeFinder *laser_;

        mutable boost::shared_mutex lock_;

public:
        LocalizedRangeScan()
        {
        }

        LocalizedRangeScan(LaserRangeFinder *laser, const RangeReadingsVector &range_readings)
        : is_dirty_(true), laser_(laser) 
        {
                number_of_range_readings_ = range_readings.size();

                range_readings_ = std::make_unique<double[]>(number_of_range_readings_);
                std::copy(range_readings.begin(), range_readings.end(), range_readings_.get());
        }


        inline void setScanId(uint32_t scan_id)
        {
                std::cout << "set scan id" << std::endl;
                scan_id_ = scan_id;
        }

        inline uint32_t getScanId()
        {
                return scan_id_;
        }

        inline void setOdometricPose(const Pose2 &pose)
        {
                odom_pose_ = pose;
        }

        inline const Pose2 &getOdometricPose() const
        {
                return odom_pose_;
        }

        inline void setCorrectedPose(const Pose2 &pose)
        {
                corrected_pose_ = pose;
        }

        inline const Pose2 &getCorrectedPose() const
        {
                return corrected_pose_;
        }

        void setSensorPose(const Pose2& scan_pose)
        {
                corrected_pose_ = getCorrectedAt(scan_pose);
                update();
        }

        /**
         * @brief Computes the pose of the robot if the sensor were at the given pose
         * @param sensor_pose sensor pose
         * @return robot pose
         */
        inline Pose2 getCorrectedAt(const Pose2 &sensor_pose) const
        {
                return Pose2::transformPose(
                        sensor_pose, 
                        getLaserRangeFinder()->getOffsetPose().inverse());
        }

        inline void setTime(double time)
        {
                time_ = time;
        }

        inline LaserRangeFinder *getLaserRangeFinder() const
        {
                return laser_;
        }

        /**
         * Get point readings in local coordinates
         */
        inline const std::vector<Eigen::Vector2d> &getPointReadings(bool want_filtered = false) const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);
                if (is_dirty_)
                {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> uniqueLock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                if (want_filtered == true) {
                        return point_readings_;
                } else {
                        return unfiltered_point_readings_;
                }
        }

        /**
         * Computes the position of the sensor
         * @return scan pose
         */
        inline Pose2 getSensorPose() const
        {
                return Pose2::transformPose(corrected_pose_, laser_->getOffsetPose());
        }

        /**
         * Gets the range readings of this scan
         * @return range readings of this scan
         */
        inline const double *getRangeReadings() const
        {
                return range_readings_.get();
        }

        /**
         * Gets the bounding box of this scan
         * @return bounding box of this scan
         */
        inline const BoundingBox2 &getBoundingBox() const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);

                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> unique_lock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                return bounding_box_;
        }

private:
        /**
         * Compute point readings based on range readings
         * Only range readings within [minimum range; range threshold] are returned
         */
        void update() 
        {
                if (laser_ != nullptr) {
                        point_readings_.clear();
                        unfiltered_point_readings_.clear();

                        double range_threshold = laser_->getRangeThreshold();
                        double minimum_angle = laser_->getMinimumAngle();
                        double angular_resolution = laser_->getAngularResolution();
                        Pose2 scan_pose = getSensorPose();
                        // compute point readings
                        Eigen::Vector2d range_points_sum(0,0);
                        uint32_t beam_num = 0;

                        std::cout << "minimum range: " << laser_->getMinimumRange() << std::endl;
                        std::cout << "range threshold: " << range_threshold << std::endl;
                        std::cout << "scan_pose x: " << scan_pose.getX() << std::endl;
                        std::cout << "heading of robot: " << scan_pose.getHeading() << std::endl;

                        for (uint32_t i = 0; i < laser_->getNumberOfRangeReadings(); i++, beam_num++) {
                                double range_reading = getRangeReadings()[i];
                                double angle = scan_pose.getHeading() + minimum_angle + beam_num * angular_resolution;
                                Eigen::Vector2d point;
                                point.x() = scan_pose.getX() + (range_reading * cos(angle));
                                point.y() = scan_pose.getY() + (range_reading * sin(angle));

                                std::cout << "x: " << point.x() << std::endl;
                                std::cout << "y: " << point.y() << std::endl;

                                if (!math::InRange(range_reading, laser_->getMinimumRange(), range_threshold)) {
                                        unfiltered_point_readings_.push_back(point);
                                        continue;
                                }

                                point_readings_.push_back(point);
                                unfiltered_point_readings_.push_back(point);
                                range_points_sum += point;
                        }


                        // compute barycenter
                        double n_points = static_cast<double>(point_readings_.size());
                        if (n_points != 0.0) {
                                Eigen::Vector2d average_position = Eigen::Vector2d(range_points_sum / n_points);
                                barycenter_pose_ = Pose2(average_position, 0.0);
                        } else {
                                barycenter_pose_ = scan_pose;
                        }

                        // calculate bounding box of scan
                        bounding_box_ = BoundingBox2();
                        bounding_box_.add(scan_pose.getPosition());

                        for (const auto &point_reading : point_readings_) {
                                bounding_box_.add(point_reading);
                        }
                }

                is_dirty_ = false;
                
        }

}; // LocalizedRangeScan

//////////////////////////////////////////////////////////////

class CellUpdater 
{
public:
        CellUpdater(OccupancyGrid *grid)
                : occupancy_grid_(grid)
        {
        }

        void operator()(uint32_t index);

private:
        OccupancyGrid *occupancy_grid_;
}; // CellUpdater

//////////////////////////////////////////////////////////////

class CoordinateConverter
{
public:
        CoordinateConverter()
                : scale_(20.0)
        {
        }

        /**
         * Sets the size
         * @param size
         */
        inline void setSize(const Size2<int32_t> &size)
        {
                size_ = size;
        }

        /**
         * Sets the scale
         * @param scale
         */
        inline void setScale(double scale)
        {
                scale_ = scale;
        }

        /**
         * Gets the size
         * @return size
         */
        inline const Size2<int32_t> &getSize() const
        {
                return size_;
        }

        /**
         * Gets the offset
         * @return offset
         */
        inline const Eigen::Vector2d &getOffset() const
        {
                return offset_;
        }

        /**
         * Sets the offset
         * @param rOffset
         */
        inline void setOffset(const Eigen::Vector2d &offset)
        {
                offset_ = offset;
        }

        /**
         * Converts the point from world coordinates to grid coordinates
         * @param world world coordinate
         * @param flip_y
         * @return grid coordinate
         */
        inline Eigen::Matrix<int32_t, 2, 1> convertWorldToGrid(
            const Eigen::Vector2d &world,
            bool flip_y = false) const
        {
                double grid_x = (world.x() - offset_.x()) * scale_;
                double grid_y = 0.0;

                if (flip_y == false) {
                        grid_y = (world.y() - offset_.y()) * scale_;
                } else {
                        grid_y = (size_.getHeight() / scale_ - world.y() + offset_.y()) * scale_;
                }

                return Eigen::Matrix<int32_t, 2, 1>(static_cast<int32_t>(std::round(grid_x)),
                                                    static_cast<int32_t>(std::round(grid_y)));
        }

private:
        double scale_;
        Size2<int32_t> size_;
        Eigen::Vector2d offset_;
};

/////////////////////////////////////////////////////////

template <typename T>
class Grid
{
public:
        Grid()
        {
        }

        static Grid *createGrid(int32_t width, int32_t height, double resolution)
        {
                Grid *grid = new Grid(width, height);

                grid->getCoordinateConverter()->setScale(1.0 / resolution);

                return grid;
        }

        /**
         * Gets the grid data pointer
         * @return data pointer
         */
        inline T *getDataPointer()
        {
        return data_;
        }

        /**
         * Gets the width step in bytes
         * @return width step
         */
        inline int32_t getWidthStep() const
        {
                return width_step_;
        }

        /**
         * Gets the width of the grid
         * @return width of the grid
         */
        inline int32_t getWidth() const
        {
                return width_;
        }

        /**
         * Gets the height of the grid
         * @return height of the grid
         */
        inline int32_t getHeight() const
        {
                return height_;
        }

        /**
         * Get value at given grid coordinate
         * @param grid_coordinate grid coordinate
         * @return value
         */
        inline T getValue(const Eigen::Matrix<int32_t, 2, 1> &grid_coordinate) const
        {
                int32_t index = getGridIndex(grid_coordinate);
                return data_[index];
        }

        /**
         * Increments all the grid cells from (x0, y0) to (x1, y1);
         * if applicable, apply f to each cell traced
         * @param x0
         * @param y0
         * @param x1
         * @param y1
         * @param cell_updater
         */
        void traceLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, CellUpdater *cell_updater = nullptr)
        {
                bool steep = abs(y1 - y0) > abs(x1 - x0);
                if (steep) {
                        std::swap(x0, y0);
                        std::swap(x1, y1);
                } if (x0 > x1) {
                        std::swap(x0, x1);
                        std::swap(y0, y1);
                }

                int32_t delta_x = x1 - x0;
                int32_t delta_y = abs(y1 - y0);
                int32_t error = 0;
                int32_t y_step;
                int32_t y = y0;

                if (y0 < y1) {
                        y_step = 1;
                } else {
                        y_step = -1;
                }

                int32_t point_x;
                int32_t point_y;
                for (int32_t x = x0; x <= x1; x++)
                {
                        if (steep) {
                                point_x = y;
                                point_y = x;
                        }
                        else
                        {
                                point_x = x;
                                point_y = y;
                        }

                        error += delta_y;

                        if (2 * error >= delta_x) {
                                y += y_step;
                                error -= delta_x;
                        }

                        Eigen::Matrix<int32_t, 2, 1> grid_index(point_x, point_y);
                        if (isValidGridIndex(grid_index))
                        {
                                int32_t index = getGridIndex(grid_index, false);
                                T *grid_pointer = getDataPointer();
                                grid_pointer[index]++;

                                if (cell_updater != nullptr) {
                                        (*cell_updater)(index);
                                }
                        }
                }
        }

        /**
         * Checks whether the given coordinates are valid grid indices
         * @param grid
         */
        inline bool isValidGridIndex(const Eigen::Matrix<int32_t, 2, 1> &grid) const
        {
                return (grid(0) < width_ && grid(1) < height_);
        }

        /**
         * Converts the point from world coordinates to grid coordinates
         * @param world world coordinate
         * @param flip_y
         * @return grid coordinate
         */
        inline Eigen::Matrix<int32_t, 2, 1> convertWorldToGrid(
                const Eigen::Vector2d &world,
                bool flip_y = false) const
        {
                return coordinate_converter_->convertWorldToGrid(world, flip_y);
        }

        /**
         * Gets the index into the data pointer of the given grid coordinate
         * @param grid
         * @param boundary_check default value is true
         * @return grid index
         */
        int32_t getGridIndex(const Eigen::Matrix<int32_t, 2, 1> &grid, bool boundary_check = true) const
        {
                if (boundary_check == true) {
                        if (isValidGridIndex(grid) == false)
                        {
                                std::stringstream error;
                                error << "Index " << grid << " out of range.  Index must be between [0; " << width_ << ") and [0; " << height_ << ")";
                        }
                }

                int32_t index = grid(0) + (grid(1) * width_step_);

                if (boundary_check == true) {
                        assert(math::IsUpTo(index, getDataSize()));
                }

                return index;
        }

        /**
         * Gets the allocated grid size in bytes
         * @return data size
         */
        inline int32_t getDataSize() const
        {
                return width_step_ * height_;
        }

        /**
         * Clear out the grid data
         */
        void clear()
        {
                memset(data_, 0, getDataSize() * sizeof(T));
        }

        /**
         * Gets the coordinate converter for this grid
         * @return coordinate converter
         */
        inline CoordinateConverter *getCoordinateConverter() const
        {
                return coordinate_converter_;
        }

        /**
         * Resizes the grid (deletes all old data)
         * @param width
         * @param height
         */
        virtual void resize(int32_t width, int32_t height)
        {
                width_ = width;
                height_ = height;
                width_step_ = math::AlignValue<int32_t>(width, 8);

                if (data_ != nullptr) {
                        delete[] data_;
                        data_ = nullptr;
                }

                try {
                        data_ = new T[getDataSize()];
                        

                        if (coordinate_converter_ == nullptr)
                        {
                                coordinate_converter_ = new CoordinateConverter();
                        }
                        coordinate_converter_->setSize(Size2<int32_t>(width, height));
                }
                catch (...) {
                        data_ = nullptr;
                        std::cout << "error in initing datapointer and coordinate converter" << std::endl;
                        width_ = 0;
                        height_ = 0;
                        width_step_ = 0;
                }

                clear();
        }

protected:
        /**
         * Constructs grid of given size
         * @param width
         * @param height
         */
        Grid(int32_t width, int32_t height)
            : data_(nullptr),
              coordinate_converter_(nullptr)
        {
                resize(width, height);
        }

private:
        int32_t width_;     // width of grid
        int32_t height_;    // height of grid
        int32_t width_step_; // 8 bit aligned width of grid
        T *data_;            // grid data
        CoordinateConverter *coordinate_converter_; // utility to convert between world coordinates and grid coordinates
}; // Grid

//////////////////////////////////////////////////////////////

class OccupancyGrid : public Grid<uint8_t>
{

        friend class CellUpdater;

public:
        /**
         * Constructs an occupancy grid of given size
         * @param width
         * @param height
         * @param offset
         * @param resolution
         */
        OccupancyGrid(
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

        /**
         * Create an occupancy grid from the given scans using the given resolution
         * @param scans
         * @param resolution
         */
        static OccupancyGrid *createFromScans(
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
                computeGridDimensions(scans, resolution, width, height, offset);
                OccupancyGrid *pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
                pOccupancyGrid->setMinPassThrough(min_pass_through);
                pOccupancyGrid->setOccupancyThreshold(occupancy_threshold);
                std::cout << "width of occgrid: " << pOccupancyGrid->getWidth() << std::endl;
                std::cout << "height of occgrid: " << pOccupancyGrid->getHeight() << std::endl;
                pOccupancyGrid->createFromScans(scans);

                return pOccupancyGrid;
        }

        /**
         * Calculate grid dimensions from localized range scans
         * @param rScans
         * @param resolution
         * @param rWidth
         * @param rHeight
         * @param rOffset
         */
        static void computeGridDimensions(
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

        /**
         * Traces a beam from the start position to the end position marking
         * the bookkeeping arrays accordingly.
         * @param world_from start position of beam
         * @param world_to end position of beam
         * @param is_endpoint_valid is the reading within the range threshold?
         * @param do_update whether to update the cells' occupancy status immediately
         * @return returns false if an endpoint fell off the grid, otherwise true
         */
        bool rayTrace(
            const Eigen::Vector2d &world_from,
            const Eigen::Vector2d &world_to,
            bool is_endpoint_valid,
            bool do_update = false)
        {
                assert(cell_pass_cnt_ != nullptr && cell_hit_cnt_ != nullptr);

                Eigen::Matrix<int32_t, 2, 1> grid_from = cell_pass_cnt_->convertWorldToGrid(world_from);
                Eigen::Matrix<int32_t, 2, 1> grid_to = cell_pass_cnt_->convertWorldToGrid(world_to);

                CellUpdater *cell_updater = do_update ? cell_updater_ : nullptr;
                cell_pass_cnt_->traceLine(grid_from(0), grid_from(1), grid_to(0),
                                          grid_to(1), cell_updater);

                // for the end point
                if (is_endpoint_valid) {
                        if (cell_pass_cnt_->isValidGridIndex(grid_to)) {
                                int32_t index = cell_pass_cnt_->getGridIndex(grid_to, false);

                                uint32_t *cell_pass_cnt_ptr = cell_pass_cnt_->getDataPointer();
                                uint32_t *cell_hit_cnt_ptr = cell_hit_cnt_->getDataPointer();

                                // increment cell pass through and hit count
                                cell_pass_cnt_ptr[index]++;
                                cell_hit_cnt_ptr[index]++;

                                if (do_update) {
                                        (*cell_updater)(index);
                                }
                        }
                }

                return cell_pass_cnt_->isValidGridIndex(grid_to);
        }

        /**
         * Updates a single cell's value based on the given counters
         * @param cell
         * @param cell_pass_cnt
         * @param cell_hit_cnt
         */
        virtual void updateCell(uint8_t *cell, uint32_t cell_pass_cnt, uint32_t cell_hit_cnt)
        {
                if (cell_pass_cnt > min_pass_through_)
                {
                        double hit_ratio = static_cast<double>(cell_hit_cnt) / static_cast<double>(cell_pass_cnt);

                        if (hit_ratio > occupancy_threshold_) {
                                *cell = static_cast<uint8_t>(GridStates::OCCUPIED);
                        } else {
                                *cell = static_cast<uint8_t>(GridStates::FREE);
                        }
                }
        }

        /**
         * Sets the minimum number of beams that must pass through a cell before it
         * will be considered to be occupied or unoccupied.
         * This prevents stray beams from messing up the map.
         */
        void setMinPassThrough(uint32_t count)
        {
                occupancy_threshold_ = count;
        }

        /**
         * Sets the minimum ratio of beams hitting cell to beams passing through
         * cell for cell to be marked as occupied.
         */
        void setOccupancyThreshold(double thresh)
        {
                occupancy_threshold_ = thresh;
        }

        /**
         * Create grid using scans
         * @param scans
         */
        void createFromScans(const std::vector<LocalizedRangeScan *> &scans)
        {
                cell_pass_cnt_->resize(getWidth(), getHeight());
                cell_pass_cnt_->getCoordinateConverter()->setOffset(getCoordinateConverter()->getOffset());
                cell_hit_cnt_->resize(getWidth(), getHeight());
                cell_hit_cnt_->getCoordinateConverter()->setOffset(getCoordinateConverter()->getOffset());

                for (const auto &scan_iter : scans)
                {
                        if (scan_iter == nullptr)
                        {
                                continue;
                        }

                        LocalizedRangeScan *scan = scan_iter;
                        addScan(scan);
                }

                update();
        }

        /**
         * Adds the scan's information to this grid's counters (optionally
         * update the grid's cells' occupancy status)
         * @param scan
         * @param do_update whether to update the grid's cell's occupancy status
         * @return returns false if an endpoint fell off the grid, otherwise true
         */
        bool addScan(LocalizedRangeScan *scan, bool do_update = false)
        {
                LaserRangeFinder *laser = scan->getLaserRangeFinder();
                double range_threshold = laser->getRangeThreshold();
                double max_range = laser->getMaximumRange();
                double min_range = laser->getMinimumRange();

                Eigen::Vector2d scan_position = scan->getSensorPose().getPosition();
                // get scan point readings
                const std::vector<Eigen::Vector2d> &point_readings = scan->getPointReadings(false);

                bool is_all_in_map = true;

                // draw lines from scan position to all point readings
                int point_index = 0;

                std::cout << "ok4" << std::endl;

                for (const auto &point_iter : point_readings)  {
                        Eigen::Vector2d point = point_iter;
                        double range_reading = scan->getRangeReadings()[point_index];
                        bool is_endpoint_valid = range_reading < range_threshold;

                        if (range_reading <= min_range || range_reading >= max_range || std::isnan(range_reading)) {
                                // ignore these readings 
                                point_index++;
                                continue;
                        } else if (range_reading >= range_threshold) {
                                // clip range reading to be within trusted region
                                double ratio = range_threshold / range_reading;
                                double dx = point.x() - scan_position.x();
                                double dy = point.y() - scan_position.y();
                                point.x() = scan_position.x() + ratio * dx;
                                point.y() = scan_position.y() + ratio * dy;
                        }

                        bool is_in_map = rayTrace(scan_position, point, is_endpoint_valid, do_update);

                        if (!is_in_map) {
                                is_all_in_map = false;
                        }

                        point_index++;
                }

                std::cout << "ok5" << std::endl;

                return is_all_in_map;
        }

        /**
         * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
         */
        virtual void update()
        {
                assert(cell_pass_cnt_ != nullptr && cell_pass_cnt_ != nullptr);

                // clear grid
                clear();

                std::cout << "ok6" << std::endl;

                // set occupancy status of cells
                uint8_t *data_ptr = getDataPointer();
                uint32_t *cell_pass_cnt_ptr = cell_pass_cnt_->getDataPointer();
                uint32_t *cell_hit_cnt_ptr = cell_hit_cnt_->getDataPointer();

                std::cout << "ok7" << std::endl;

                uint32_t n_bytes = getDataSize();
                for (uint32_t i = 0; i < n_bytes; i++, data_ptr++, cell_pass_cnt_ptr++, cell_hit_cnt_ptr++) {
                        updateCell(data_ptr, *cell_pass_cnt_ptr, *cell_hit_cnt_ptr);
                }

                
        }

private:
        double occupancy_threshold_;
        uint32_t min_pass_through_;
        CellUpdater *cell_updater_;

        /**
         * Restrict the copy constructor
         */
        OccupancyGrid(const OccupancyGrid &);

        /**
         * Restrict the assignment operator
         */
        const OccupancyGrid &operator=(const OccupancyGrid &);

protected:
        /**
         * Counters of number of times a beam passed through a cell
         */
        Grid<uint32_t> *cell_pass_cnt_;

        /**
         * Counters of number of times a beam ended at a cell
         */
        Grid<uint32_t> *cell_hit_cnt_;
}; // OccupancyGrid

//////////////////////////////////////////////////////////////////

class ScanManager
{
private:
        std::map<int, LocalizedRangeScan *> scans_;
        std::vector<LocalizedRangeScan *> running_scans_;
        LocalizedRangeScan *last_scan_;
        uint32_t next_scan_id_;

        uint32_t running_buffer_maximum_size_;
        double running_buffer_maximum_distance_;

public:
        ScanManager() 
        : last_scan_(nullptr),
        next_scan_id_(0)
        {
        }

        ScanManager(uint32_t running_buffer_maximum_size, double running_buffer_maximum_distance)
            : last_scan_(nullptr),
              next_scan_id_(0),
              running_buffer_maximum_size_(running_buffer_maximum_size),
              running_buffer_maximum_distance_(running_buffer_maximum_distance)
        {
        }

        /**
         * Gets all scans of all devices
         * @return all scans of all devices
         */
        std::vector<LocalizedRangeScan *> getAllScans()
        {
                std::vector<LocalizedRangeScan *> scans;
                scans.reserve(scans_.size());

                for (const auto &scan : scans_)
                {
                        scans.push_back(scan.second);
                }

                return scans;
        }


        inline void addRunningScan(LocalizedRangeScan *scan)
        {
                // running_scans_.push_back(scan);

                // // vector has at least one element (first line of this function), so this is valid
                // Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
                // Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

                // // cap vector size and remove all scans from front of vector that are too far from end of vector
                // kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(
                //     backScanPose.GetPosition());
                // while (m_RunningScans.size() > m_RunningBufferMaximumSize ||
                //        squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
                // {
                //         // remove front of running scans
                //         m_RunningScans.erase(m_RunningScans.begin());

                //         // recompute stats of running scans
                //         frontScanPose = m_RunningScans.front()->GetSensorPose();
                //         backScanPose = m_RunningScans.back()->GetSensorPose();
                //         squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
                // }
        }

        inline std::vector<LocalizedRangeScan *> getRunningScans()
        {
                return running_scans_;
        }

        inline void setLastScan(LocalizedRangeScan *scan)
        {
                last_scan_ = scan;
        }

        
        inline LocalizedRangeScan *getLastScan()
        {
                return last_scan_;
        }


        inline void addScan(LocalizedRangeScan *scan) 
        {
                std::cout << "traced from 4::1::0" << std::endl;
                if (scan == nullptr) {
                        std::cout << "scan is nullptr" << std::endl;
                }
                // assign unique scan id
                scan->setScanId(next_scan_id_);
                // add to scan buffer
                std::cout << "number of range reading before add: " << scan->getLaserRangeFinder()->getNumberOfRangeReadings() << std::endl;
                scans_.emplace(next_scan_id_, scan);
                next_scan_id_++;
                std::cout << "traced from 4::1::2" << std::endl;
        }

        void setRunningScanBufferSize(uint32_t scan_buffer_size)
        {
                running_buffer_maximum_size_ = scan_buffer_size;
        }

        void setRunningScanBufferMaximumDistance(double scan_buffer_max_distance)
        {
                running_buffer_maximum_distance_ = scan_buffer_max_distance;
        }



}; // ScanManager

///////////////////////////////////////////////////////////////////////

/**
 * Represents an object in a graph
 */
template<typename T>
class Vertex
{
private:
        T *object_;
        std::vector<Edge<T> *> edges_; 
        double score;
}; // Vertex

///////////////////////////////////////////////////////////////////////

class ScanMatcher
{
private:
public:
        ScanMatcher()
        {
        }

        /**
         * Parallelize scan matching
         */
        void operator()(const double &y) const;

        /**
         * Match given scan against set of scans
         * @param pScan scan being scan-matched
         * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
         * @param rMean output parameter of mean (best pose) of match
         * @param rCovariance output parameter of covariance of match
         * @param doPenalize whether to penalize matches further from the search center
         * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
         * @return strength of response
         */
        template <class T = std::vector<LocalizedRangeScan *>>
        double matchScan(
                LocalizedRangeScan *scan,
                const T &base_scans,
                Pose2 &mean, Eigen::Matrix3d &covariance,
                bool do_penalize = true,
                bool do_refine_match = true);
}; // ScanMatcher

///////////////////////////////////////////////////////////////////////

class MapperGraph
{

public:
        MapperGraph()
        {
        }

        /**
         * Adds a vertex representing the given scan to the graph
         * @param pScan
         */
        Vertex<LocalizedRangeScan> *addVertex(LocalizedRangeScan *scan);

        /**
         * Link scan to last scan and nearby chains of scans
         * @param pScan
         * @param rCovariance uncertainty of match
         */
        void addEdges(LocalizedRangeScan *scan, const Eigen::Matrix3d &covariance);


        bool tryCloseLoop(LocalizedRangeScan *scan);

}; // MapperGraph

///////////////////////////////////////////////////////////////////////

class ScanSolver
{
        ScanSolver()
        {
        }

}; // ScanSolver

///////////////////////////////////////////////////////////////////////

class Mapper 
{
private:
        // state
        bool initialized_;

        // parameters
        double minimum_travel_distance_;
        double minimum_travel_heading_;
        /**
         * Scan buffer size is the length of the scan chain stored for scan matching.
         * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
         * The idea is to get an area approximately 20 meters long for scan matching.
         * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
         * should be 20 / 0.3 = 67.)
         * Default value is 67.
         */
        uint32_t scan_buffer_size_;
        /**
         * Scan buffer maximum scan distance is the maximum distance between the first and last scans
         * in the scan chain stored for matching.
         * Default value is 20.0.
         */
        double scan_buffer_maximum_scan_distance_;

        ////////////////////////////////////////////////////////////
        // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
        // then not many beams will hit the cell!

        // Number of beams that must pass through a cell before it will be considered to be occupied
        // or unoccupied.  This prevents stray beams from messing up the map.
        uint32_t min_pass_through_;

        // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
        double occupancy_threshold_;

protected:
        std::unique_ptr<ScanManager> scan_manager_;
        std::unique_ptr<ScanMatcher> scan_matcher_;
        std::unique_ptr<MapperGraph> graph_;
        std::unique_ptr<ScanSolver> scan_solver_;

public:
        Mapper()
        : scan_manager_(nullptr),
        initialized_(false)
        {
        }

        void initialize(double range_threshold)
        {
                if (initialized_) {
                        return;
                }

                scan_manager_ = std::make_unique<ScanManager>(scan_buffer_size_, scan_buffer_maximum_scan_distance_);

                initialized_ = true;
        }

        template <class NodeT>
        void configure(const NodeT &node);

        inline double getMinimumTravelDistance() const {
                return minimum_travel_distance_;
        }

        inline double getMinimumTravelHeading() const {
                return minimum_travel_heading_;
        }

        /**
         * Process a localized range scan for incorporation into the map.  The scan must
         * be identified with a range finder device.  Once added to a map, the corrected pose information in the
         * localized scan will be updated to the correct pose as determined by the mapper.
         *
         * @param scan A localized range scan that has pose information associated directly with the scan data.  The pose
         * is that of the range device originating the scan.  Note that the mapper will set corrected pose
         * information in the scan object.
         *
         * @return true if the scan was added successfully, false otherwise
         */
        bool process(LocalizedRangeScan *scan, Eigen::Matrix3d *covariance = nullptr);

        /**
         * Returns all processed scans added to the mapper.
         * NOTE: The returned scans have their corrected pose updated.
         * @return list of scans received and processed by the mapper. If no scans have been processed,
         * return an empty list.
         */
        const std::vector<LocalizedRangeScan *> getAllProcessedScans() const
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

        // get occupancy grid from scans
        OccupancyGrid *getOccupancyGrid(const double &resolution)
        {
                OccupancyGrid *occ_grid = nullptr;
                return OccupancyGrid::createFromScans(
                    getAllProcessedScans(),
                    resolution,
                    (uint32_t)getParamMinPassThrough(),
                    (double)getParamOccupancyThreshold());
        }

        uint32_t getParamMinPassThrough()
        {
                return min_pass_through_;
        }

        double getParamOccupancyThreshold()
        {
                return occupancy_threshold_;
        }

}; // Mapper

//////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////



///////////////////////////////////////////////////////


/////////////////////////////////////////////////

class PoseHelper
{
public:
        PoseHelper(tf2_ros::Buffer *tf)
            : tf_(tf)
        {
        }

        /**
         * Get transform between frames available in tf2 tree
         * 
         * @param pose pose holder
         * @param t pose's time
         * @param target_frame T_a
         * @param source_frame T_b
         * 
         * @return T_a_b, pose of frame b from frame a
         */
        bool getPose(
                Pose2 &pose,
                const rclcpp::Time &t,
                const std::string &target_frame,
                const std::string &source_frame)
        {
                geometry_msgs::msg::TransformStamped tmp_pose;
                try {
                        tmp_pose = tf_->lookupTransform(
                            target_frame,
                            source_frame,
                            t);
                } catch (const tf2::TransformException &ex) {
                        return false;
                }

                const double yaw = tf2::getYaw(tmp_pose.transform.rotation);
                pose = Pose2(tmp_pose.transform.translation.x,
                             tmp_pose.transform.translation.y, yaw);

                return true;
        }

private:
        tf2_ros::Buffer *tf_;
}; // PoseHelper

inline void toNavMap(
        const OccupancyGrid *occ_grid,
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
                        case static_cast<uint8_t>(GridStates::UNKNOWN):
                                map.data[MAP_IDX(map.info.width, x, y)] = -1;
                                break;
                        case static_cast<uint8_t>(GridStates::OCCUPIED):
                                map.data[MAP_IDX(map.info.width, x, y)] = 100;
                                break;
                        case static_cast<uint8_t>(GridStates::FREE):
                                map.data[MAP_IDX(map.info.width, x, y)] = 0;
                                break;
                        }
                }
        }
}

} // namespace mapper_utils

#endif // MAPPER_UTILS_HPP