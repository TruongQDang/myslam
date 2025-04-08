#ifndef MAPPER_UTILS_HPP
#define MAPPER_UTILS_HPP

#include <unordered_map>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <cstring>
#include <iostream>

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
class LaserRangeFinder;
class OccupancyGrid;
class CellUpdater;
class CorrelationGrid;
class LocalizedRangeScan;
class PoseHelper;
class CoordinateConverter;

using namespace ::myslam_types;

//////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////

class LaserRangeFinder
{
public:
        LaserRangeFinder()
        {

        }

        /**
         * Sets this range finder sensor's minimum range
         * @param minimum_range
         */
        inline void setMinimumRange(double minimum_range)
        {
                minimum_range_ = minimum_range;

                // SetRangeThreshold(GetRangeThreshold());
        }

        /**
         * Sets this range finder sensor's maximum range
         * @param maximum_range
         */
        inline void setMaximumRange(double maximum_range)
        {
                maximum_range_ = maximum_range;

                // SetRangeThreshold(GetRangeThreshold());
        }

        /**
         * Sets the range threshold
         * @param range_threshold
         */
        inline void setRangeThreshold(double range_threshold)
        {
                range_threshold_ = range_threshold;
        }

        inline void setMinimumAngle(double minimum_angle)
        {
                minimum_angle_ = minimum_angle;
        }

        inline void setAngularResolution(double angular_resolution)
        {
                angular_resolution_ = angular_resolution;
        }

        inline void setNumberOfRangeReadings(uint32_t number_of_range_readings)
        {
                number_of_range_readings_ = number_of_range_readings;
        }

        /**
         * Gets the number of range readings each localized range scan must contain to be a valid scan.
         * @return number of range readings
         */
        inline uint32_t getNumberOfRangeReadings() const
        {
                return number_of_range_readings_;
        }

        /**
         * Gets this range finder sensor's minimum range
         * @return minimum range
         */
        inline double getMinimumRange() const
        {
                return minimum_range_;
        }


        /**
         * Gets the range threshold
         * @return range threshold
         */
        inline double getRangeThreshold() const
        {
                return range_threshold_;
        }

        /**
         * Gets this range finder sensor's maximum range
         * @return maximum range
         */
        inline double getMaximumRange() const
        {
                return maximum_range_;
        }

        /**
         * Gets this range finder sensor's minimum angle
         * @return minimum angle
         */
        inline double getMinimumAngle() const
        {
                return minimum_angle_;
        }

        /**
         * Gets this range finder sensor's angular resolution
         * @return angular resolution
         */
        inline double getAngularResolution() const
        {
                return angular_resolution_;
        }

        /**
         * Gets this range finder sensor's offset
         * @return offset pose
         */
        inline const Pose2 &getOffsetPose() const
        {
                return offset_pose_;
        }

private:
        double range_threshold_;
        double minimum_range_;
        double maximum_range_;
        double minimum_angle_;
        double angular_resolution_;
        uint32_t number_of_range_readings_;
        Pose2 offset_pose_;
};

////////////////////////////////////////////////////////////

class LocalizedRangeScan
{
public:
        LocalizedRangeScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan);

        LocalizedRangeScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan, LaserRangeFinder *laser)
                : LocalizedRangeScan(scan)
        {
                laser_ = laser;
                is_dirty_ = true;
        } 

        inline void setScanId(int32_t scan_id)
        {
                scan_id_ = scan_id;
        }

        inline int32_t getScanId()
        {
                return scan_id_;
        }

        /**
         * Gets the odometric pose of this scan
         * @return odometric pose of this scan
         */
        inline const Pose2 &getOdometricPose() const
        {
                return odom_pose_;
        }

        /**
         * Sets the odometric pose of this scan
         * @param pose
         */
        inline void setOdometricPose(const Pose2 &pose)
        {
                odom_pose_ = pose;
        }

        /**
         * Gets the (possibly corrected) robot pose at which this scan was taken.  The corrected robot pose of the scan
         * is usually set by an external module such as a localization or mapping module when it is determined
         * that the original pose was incorrect.  The external module will set the correct pose based on
         * additional sensor data and any context information it has.  If the pose has not been corrected,
         * a call to this method returns the same pose as GetOdometricPose().
         * @return corrected pose
         */
        inline const Pose2 &getCorrectedPose() const
        {
                return corrected_pose_;
        }

        /**
         * Moves the scan by moving the robot pose to the given location.
         * @param pose new pose of the robot of this scan
         */
        inline void setCorrectedPose(const Pose2 &pose)
        {
                corrected_pose_ = pose;
        }

        inline void setTime(rclcpp::Time time)
        {
                time_ = time;
        }

        /**
         * Gets the laser range finder sensor that generated this scan
         * @return laser range finder sensor of this scan
         */
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
                return Pose2::applyTransform(corrected_pose_, laser_->getOffsetPose());
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
                        Eigen::Vector2d range_points_sum;
                        uint32_t beam_num = 0;
                        for (uint32_t i = 0; i < laser_->getNumberOfRangeReadings(); i++, beam_num++) {
                                // iterate through all range readings
                                double range_reading = getRangeReadings()[i];
                                double angle = scan_pose.getHeading() + minimum_angle + beam_num * angular_resolution;
                                Eigen::Vector2d point;
                                point.x() = scan_pose.getX() + (range_reading * cos(angle));
                                point.y() = scan_pose.getY() + (range_reading * sin(angle));
        
                                if (range_reading < laser_->getMinimumRange() && range_reading > range_threshold) {
                                        // if not within valid range
                                        unfiltered_point_readings_.push_back(point);
                                } else {
                                        // if within valid range
                                        point_readings_.push_back(point);
                                        unfiltered_point_readings_.push_back(point);
                                        range_points_sum += point;
                                }
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

        int32_t scan_id_;
        Pose2 corrected_pose_;
        Pose2 odom_pose_;
        /**
         * Average of all the point readings
         */
        Pose2 barycenter_pose_;
        std::unique_ptr<double[]> range_readings_;
        rclcpp::Time time_;
        BoundingBox2 bounding_box_;
        bool is_dirty_;
        std::vector<Eigen::Vector2d> point_readings_;
        std::vector<Eigen::Vector2d> unfiltered_point_readings_;
        LaserRangeFinder *laser_;

        mutable boost::shared_mutex lock_;

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
                        assert(index < getDataSize());
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
                        std::cout << "allocate " << getDataSize() << std::endl;
                        data_ = new T[getDataSize()];
                        

                        if (coordinate_converter_ == nullptr)
                        {
                                coordinate_converter_ = new CoordinateConverter();
                        }
                        std::cout << "fine until here" << std::endl;
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
            double resolution);

        /**
         * Create an occupancy grid from the given scans using the given resolution
         * @param scans
         * @param resolution
         */
        static OccupancyGrid *createFromScans(
                const std::vector<LocalizedRangeScan *> &scans,
                double resolution,
                uint32_t min_pass_through,
                double occupancy_threshold);

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
                Eigen::Vector2d &offset);

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
                if (is_endpoint_valid)
                {
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
                        if (static_cast<double>(cell_pass_cnt) == 0.0)
                        {
                                std::cout << "invalid division" << std::endl;
                        }
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
        void createFromScans(const std::vector<LocalizedRangeScan *> &scans);

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

                if (data_ptr == nullptr) {
                        std::cout << "dat_ptr null" << std::endl;
                } 
                
                if (cell_pass_cnt_ptr == nullptr) {
                        std::cout << "cell_pass_ptr null" << std::endl;
                }
                if (cell_hit_cnt_ptr == nullptr) {
                        std::cout << "cell_hit_ptr null" << std::endl;
                }

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
public:
        ScanManager() 
        {
        }

        /**
         * Gets all scans of all devices
         * @return all scans of all devices
         */
        std::vector<LocalizedRangeScan *> getAllScans()
        {
                std::vector<LocalizedRangeScan *> scans;

                return scans;
        }

        /**
         * Gets last scan of given sensor
         * @return last localized range scan of sensor
         */
        inline LocalizedRangeScan *getLastScan()
        {
                return last_scan_;
        }

        inline void addScan(LocalizedRangeScan *scan) 
        {
                // assign unique scan id
                scan->setScanId(next_scan_id_);
                next_scan_id_++;
                // add to scan buffer
                scans_.emplace(scan->getScanId(), scan);
        }

        inline void setLastScan(LocalizedRangeScan *scan)
        {
                last_scan_ = scan;
        }


private:
        std::map<int, LocalizedRangeScan *> scans_;
        std::vector<LocalizedRangeScan *> running_scans_;
        LocalizedRangeScan *last_scan_;
        uint32_t next_scan_id_;
        
        uint32_t running_buffer_maximum_size_;
        double running_buffer_maximum_distance_;

}; // ScanManager

///////////////////////////////////////////////////////////////////////

class Mapper 
{
public:
        template <class NodeT>
        Mapper(const NodeT &node);

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
        const std::vector<LocalizedRangeScan *> getAllProcessedScans() const;

        // get occupancy grid from scans
        OccupancyGrid *getOccupancyGrid(const double &resolution);

        uint32_t getParamMinPassThrough()
        {
                return min_pass_through_;
        }

        double getParamOccupancyThreshold()
        {
                return occupancy_threshold_;
        }

private:
        ScanManager *scan_manager_;
        // parameters
        double minimum_travel_distance_;
        double minimum_travel_heading_;
        ////////////////////////////////////////////////////////////
        // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
        // then not many beams will hit the cell!

        // Number of beams that must pass through a cell before it will be considered to be occupied
        // or unoccupied.  This prevents stray beams from messing up the map.
        uint32_t min_pass_through_;

        // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
        double occupancy_threshold_;
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

        bool getPose(
            Pose2 &pose,
            const rclcpp::Time &t,
            const std::string &from_frame,
            const std::string &to_frame);

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