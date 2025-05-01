#ifndef MAPPER_UTILS_HPP
#define MAPPER_UTILS_HPP

#include <unordered_map>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <cstring>
#include <iostream>
#include <cmath>
#include <tbb/parallel_for_each.h>
#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"

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
template <typename T>
class Visitor;
class LaserRangeFinder;
class OccupancyGrid;
class CellUpdater;
class CorrelationGrid;
class LocalizedRangeScan;
class PoseHelper;
class CoordinateConverter;
class LaserRangeFinder;

using namespace ::myslam_types;

typedef std::vector<double> RangeReadingsVector;
typedef std::vector<LocalizedRangeScan *> LocalizedRangeScanVector;
typedef std::map<int, LocalizedRangeScan *> LocalizedRangeScanMap;
typedef std::vector<Eigen::Vector2d> PointVectorDouble;

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

class LocalizedRangeScan
{
private:
        int32_t scan_id_;
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


        inline void setScanId(int32_t scan_id)
        {
                scan_id_ = scan_id;
        }

        inline int32_t getScanId()
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

        /**
         * Gets sensor data time
         * @return time
         */
        inline double getTime() const
        {
                return time_;
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

        inline RangeReadingsVector getRangeReadingsVector() const
        {
                return RangeReadingsVector(range_readings_.get(), range_readings_.get() + number_of_range_readings_);
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

        inline uint32_t getNumberOfRangeReadings() const
        {
                return number_of_range_readings_;
        }

        /**
         * Moves the scan by moving the robot pose to the given location and update point readings.
         * @param rPose new pose of the robot of this scan
         */
        inline void setCorrectedPoseAndUpdate(const Pose2 &pose)
        {
                setCorrectedPose(pose);

                update();
        }

        /**
         * Gets barycenter of point readings
         */
        inline const Pose2 &getBarycenterPose() const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);
                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> unique_lock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                return barycenter_pose_;
        }

        /**
         * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
         * @param useBarycenter
         * @return barycenter if parameter is true, otherwise scanner pose
         */
        inline Pose2 getReferencePose(bool use_barycenter) const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);
                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> unique_lock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                return use_barycenter ? getBarycenterPose() : getSensorPose();
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
private:
        double scale_;
        Size2<int32_t> size_;
        Eigen::Vector2d offset_;

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

        inline double getResolution() const
        {
                return 1.0 / scale_;
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
        inline Vector2i convertWorldToGrid(
            const Vector2d &world,
            bool flip_y = false) const
        {
                double grid_x = (world.x() - offset_.x()) * scale_;
                double grid_y = 0.0;

                if (flip_y == false) {
                        grid_y = (world.y() - offset_.y()) * scale_;
                } else {
                        grid_y = (size_.getHeight() / scale_ - world.y() + offset_.y()) * scale_;
                }

                return Vector2i(
                        static_cast<int32_t>(std::round(grid_x)),
                        static_cast<int32_t>(std::round(grid_y)));
        }
};

/////////////////////////////////////////////////////////

template <typename T>
class Grid
{
private:
        int32_t width_;                             // width of grid
        int32_t height_;                            // height of grid
        int32_t width_step_;                        // 8 bit aligned width of grid
        T *data_;                                   // grid data
        CoordinateConverter *coordinate_converter_; // utility to convert between world coordinates and grid coordinates

public:
        Grid()
        {
        }

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

        static std::unique_ptr<Grid<T>> createGrid(int32_t width, int32_t height, double resolution)
        {
                std::unique_ptr<Grid<T>> grid = std::make_unique<Grid<T>>(width, height);

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
         * Gets pointer to data at given grid coordinate
         * @param rGrid grid coordinate
         * @return grid point
         */
        T *getDataPointer(const Eigen::Matrix<int32_t, 2, 1> &grid)
        {
                int32_t index = getGridIndex(grid, true);
                return data_ + index;
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

        inline double getResolution() const
        {
                return coordinate_converter_->getResolution();
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
        inline Vector2i convertWorldToGrid(
                const Vector2d &world,
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
        virtual int32_t getGridIndex(const Vector2i &grid, bool boundary_check = true) const
        {
                if (boundary_check == true) {
                        if (isValidGridIndex(grid) == false) {
                                std::stringstream error;
                                error << "Index " << grid << " out of range.  Index must be between [0; " << width_ << ") and [0; " << height_ << ")";
                        }
                }

                int32_t index = grid.x() + (grid.y() * width_step_);

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
}; // Grid

//////////////////////////////////////////////////////////////

/**
 * Create lookup tables for point readings at varying angles in grid.
 * For each angle, grid indexes are calculated for each range reading.
 * This is to speed up finding best angle/position for a localized range scan
 *
 * Used heavily in mapper and localizer.
 *
 * In the localizer, this is a huge speed up for calculating possible position.  For each particle,
 * a probability is calculated.  The range scan is the same, but all grid indexes at all possible angles are
 * calculated.  So when calculating the particle probability at a specific angle, the index table is used
 * to look up probability in probability grid!
 *
 */
template<typename T>
class GridIndexLookup
{
private:
        Grid<T> *grid_;

        uint32_t capacity_;
        uint32_t size_;

        std::vector<std::unique_ptr<LookupArray>> lookup_array_;

        // for sanity check
        std::vector<double> angles_;

public:
        GridIndexLookup()
        { 
        }

        GridIndexLookup(Grid<T> *grid) // NOLINT
                : grid_(grid),
                capacity_(0),
                size_(0)
        {
        }

        /**
         * Gets the lookup array for a particular angle index
         * @param index
         * @return lookup array
         */
        const LookupArray *getLookupArray(uint32_t index) const
        {
                assert(math::IsUpTo(index, size_));

                return lookup_array_[index].get();
        }

        /**
         * Compute lookup table of the points of the given scan for the given angular space
         * @param pScan the scan
         * @param angleCenter
         * @param angleOffset computes lookup arrays for the angles within this offset around angleStart
         * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
         */
        void computeOffsets(
                LocalizedRangeScan *scan,
                double angle_center,
                double angle_offset,
                double angle_resolution)
        {
                assert(angle_offset != 0.0);
                assert(angle_resolution != 0.0);

                uint32_t n_angles =
                        static_cast<uint32_t>(std::round(angle_offset * 2.0 / angle_resolution) + 1);
                setSize(n_angles);

                //////////////////////////////////////////////////////
                // convert points into local coordinates of scan pose

                const PointVectorDouble &point_readings = scan->getPointReadings();

                Pose2Vector local_points;
                for (const auto& point : point_readings) {
                        // get points in local coordinates
                        Pose2 vec = Pose2::transformPose(scan->getSensorPose().inverse(), Pose2(point, 0.0));
                        local_points.push_back(vec);
                }

                //////////////////////////////////////////////////////
                // create lookup array for different angles
                double angle = 0.0;
                double start_angle = angle_center - angle_offset;
                for (uint32_t angle_index = 0; angle_index < n_angles; angle_index++) {
                        angle = start_angle + angle_index * angle_resolution;
                        computeOffsets(angle_index, angle, local_points, scan);
                }
        }

private:
        /**
         * Compute lookup value of points for given angle
         * @param angleIndex
         * @param angle
         * @param rLocalPoints
         */
        void computeOffsets(
                uint32_t angle_index, double angle, const Pose2Vector &local_points,
                LocalizedRangeScan *scan)
        {
                lookup_array_[angle_index]->setSize(static_cast<uint32_t>(local_points.size()));
                angles_.at(angle_index) = angle;

                // set up point array by computing relative offsets to points readings
                // when rotated by given angle

                const Vector2d &grid_offset = grid_->getCoordinateConverter()->getOffset();

                double cosine = cos(angle);
                double sine = sin(angle);

                uint32_t reading_index = 0;

                int32_t *angle_index_pointer = lookup_array_[angle_index]->getArrayPointer();

                double max_range = scan->getLaserRangeFinder()->getMaximumRange();

                for (const auto& point : local_points) {
                        const Vector2d &position = point.getPosition();
                        if (std::isnan(scan->getRangeReadings()[reading_index]) ||
                            std::isinf(scan->getRangeReadings()[reading_index])) {
                                angle_index_pointer[reading_index] = math::INVALID_SCAN;
                                reading_index++;
                                continue;
                        }

                        // counterclockwise rotation and that rotation is about the origin (0, 0).
                        Vector2d offset;
                        offset.x() = cosine * position.x() - sine * position.y();
                        offset.y() = sine * position.x() + cosine * position.y();

                        // have to compensate for the grid offset when getting the grid index
                        Vector2i grid_point = grid_->convertWorldToGrid(offset + grid_offset);

                        // use base GridIndex to ignore ROI
                        int32_t lookup_index = grid_->Grid<T>::getGridIndex(grid_point, false);

                        angle_index_pointer[reading_index] = lookup_index;

                        reading_index++;
                }

                assert(reading_index == local_points.size());
        }

        /**
         * Sets size of lookup table (resize if not big enough)
         * @param size
         */
        void setSize(uint32_t size)
        {
                assert(size != 0);

                if (size > capacity_)
                {
                        lookup_array_.clear();

                        capacity_ = size;
                        lookup_array_.reserve(capacity_);

                        for (uint32_t i = 0; i < capacity_; i++) {
                                lookup_array_.push_back(std::make_unique<LookupArray>());
                        }
                }

                size_= size;

                angles_.resize(size);
        }

}; // GridIndexLookup


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
        std::unique_ptr<Grid<uint32_t>> cell_pass_cnt_;

        /**
         * Counters of number of times a beam ended at a cell
         */
        std::unique_ptr<Grid<uint32_t>> cell_hit_cnt_;
}; // OccupancyGrid

//////////////////////////////////////////////////////////////////

class ScanManager
{
private:
        LocalizedRangeScanMap scans_;
        LocalizedRangeScanVector running_scans_;
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
                running_scans_.push_back(scan);

                // vector has at least one element (first line of this function), so this is valid
                Pose2 front_scan_pose = running_scans_.front()->getSensorPose();
                Pose2 back_scan_pose = running_scans_.back()->getSensorPose();

                // cap vector size and remove all scans from front of vector that are too far from end of vector
                double squared_distance = front_scan_pose.getSquaredDistance(back_scan_pose);
                while (running_scans_.size() > running_buffer_maximum_size_ ||
                       squared_distance > math::Square(running_buffer_maximum_distance_) - math::TOLERANCE)
                {
                        // remove front of running scans
                        running_scans_.erase(running_scans_.begin());

                        // recompute stats of running scans
                        front_scan_pose = running_scans_.front()->getSensorPose();
                        back_scan_pose = running_scans_.back()->getSensorPose();
                        squared_distance = front_scan_pose.getSquaredDistance(back_scan_pose);
                }
        }

        inline LocalizedRangeScanVector getRunningScans()
        {
                return running_scans_;
        }

        /**
         * Gets scan from given device with given ID
         * @param rSensorName
         * @param scanNum
         * @return localized range scan
         */
        LocalizedRangeScan *getScan(int32_t scan_index)
        {
                LocalizedRangeScanMap::iterator it = scans_.find(scan_index);
                if (it != scans_.end()) {
                        return it->second;
                } else {
                        return nullptr;
                }
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
                // assign unique scan id
                scan->setScanId(next_scan_id_);
                // add to scan buffer
                scans_.emplace(next_scan_id_, scan);
                next_scan_id_++;
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

class LinkInfo
{
private:
        Pose2 pose1_;
        Pose2 pose2_;
        Pose2 pose_difference_;
        Matrix3d covariance_;

public:
        LinkInfo()
        {
        }

        LinkInfo(const Pose2 &rPose1, const Pose2 &rPose2, const Matrix3d &rCovariance)
        {
                Update(rPose1, rPose2, rCovariance);
        }
public:
        /**
         * Changes the link information to be the given parameters
         * @param rPose1
         * @param rPose2
         * @param rCovariance
         */
        void Update(const Pose2 &pose1, const Pose2 &pose2, const Matrix3d &covariance)
        {
                pose1_ = pose1;
                pose2_ = pose2;

                // transform second pose into the coordinate system of the first pose
                Pose2::transformPose(pose1.inverse(), pose2);

                // transform covariance into reference of first pose
                Matrix3d rotation_matrix = Eigen::AngleAxisd(
                        pose1.getHeading(), 
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();
                covariance_ = rotation_matrix * covariance * rotation_matrix.transpose();
        }

        /**
         * Gets the pose difference
         * @return pose difference
         */
        inline const Pose2 &getPoseDifference()
        {
                return pose_difference_;
        }

        /**
         * Gets the link covariance
         * @return link covariance
         */
        inline const Matrix3d &getCovariance()
        {
                return covariance_;
        }
}; // LinkInfo

///////////////////////////////////////////////////////////////////////

/**
 * Represents an object in a graph
 */
template<typename T>
class Vertex
{
        friend class Edge<T>;
private:
        T *object_;
        std::vector<Edge<T> *> edges_; 
        double score_;

public:
        explicit Vertex(T *object)
            : object_(object), score_(1.0)
        {
        }
public:
        /**
         * Gets the object associated with this vertex
         * @return the object
         */
        inline T *getObject() const
        {
                return object_;
        }

        /**
         * Gets edges adjacent to this vertex
         * @return adjacent edges
         */
        inline const std::vector<Edge<T> *> &getEdges() const
        {
                return edges_;
        }

        /**
         * Gets a vector of the vertices adjacent to this vertex
         * @return adjacent vertices
         */
        std::vector<Vertex<T> *> getAdjacentVertices() const
        {
                std::vector<Vertex<T> *> vertices;

                for (const auto &edge : edges_) {
                        if (edge == nullptr) {
                                continue;
                        }

                        // check both source and target because we have a undirected graph
                        if (edge->getSource() != this) {
                                vertices.push_back(edge->getSource());
                        }

                        if (edge->getTarget() != this) {
                                vertices.push_back(edge->getTarget());
                        }
                }

                return vertices;
        }
private:
        /**
         * Adds the given edge to this vertex's edge list
         * @param pEdge edge to add
         */
        inline void addEdge(Edge<T> *edge)
        {
                edges_.push_back(edge);
        }

}; // Vertex

///////////////////////////////////////////////////////////////////////

/**
 * Represents an edge in a graph
 */
template<typename T>
class Edge
{
private:
        Vertex<T> *source_;
        Vertex<T> *target_;
        LinkInfo *label_;
public:
        Edge(Vertex<T> *source, Vertex<T> *target)
                : source_(source),
                target_(target),
                label_(nullptr)
        {
                source_->addEdge(this);
                target_->addEdge(this);
        }

public:
        /**
         * Gets the target vertex
         * @return target vertex
         */
        inline Vertex<T> *getTarget() const
        {
                return target_;
        }

        /**
         * Sets the link payload
         * @param pLabel
         */
        inline void setLabel(LinkInfo *label)
        {
                label_ = label;
        }

        /**
         * Gets the link info
         * @return link info
         */
        inline LinkInfo *getLabel()
        {
                return label_;
        }

        /**
         * Gets the source vertex
         * @return source vertex
         */
        inline Vertex<T> *getSource() const
        {
                return source_;
        }

}; // Edge

///////////////////////////////////////////////////////////////////////

template<typename T>
class Graph
{        
public:
        typedef std::map<int, std::unique_ptr<Vertex<T>>> VertexMap;
protected:
        VertexMap vertices_;
        std::vector<std::unique_ptr<Edge<T>>> edges_;
public:
        Graph()
        {
        }
public:
        /**
         * Adds and indexes the given vertex into the map using the given name
         * @param rName
         * @param pVertex
         */
        inline void addVertex(std::unique_ptr<Vertex<T>> vertex)
        {
                int key = vertex->getObject()->getScanId();
                vertices_.emplace(key, std::move(vertex));
        }

        /**
         * Adds an edge to the graph
         * @param pEdge
         */
        inline void addEdge(std::unique_ptr<Edge<T>> edge)
        {
                edges_.push_back(std::move(edge));
        }

}; // Graph

///////////////////////////////////////////////////////////////////////

class CorrelationGrid : public Grid<uint8_t>
{
private:
        /**
         * The point readings are smeared by this value in X and Y to create a smoother response.
         * Default value is 0.03 meters.
         */
        double smear_deviation_;

        // Size of one side of the kernel
        int32_t kernel_size_;

        // Cached kernel for smearing
        std::unique_ptr<uint8_t[]> kernel_;

        // region of interest
        Rectangle2<int32_t> roi_;

public:
        CorrelationGrid()
        {
        }

        /**
         * Constructs a correlation grid of given size and parameters
         * @param width
         * @param height
         * @param borderSize
         * @param resolution
         * @param smearDeviation
         */
        CorrelationGrid(
            uint32_t width, uint32_t height, uint32_t borderSize,
            double resolution, double smearDeviation)
            : Grid<uint8_t>(width + borderSize * 2, height + borderSize * 2),
              smear_deviation_(smearDeviation),
              kernel_(nullptr)
        {
                getCoordinateConverter()->setScale(1.0 / resolution);

                // setup region of interest
                roi_ = Rectangle2<int32_t>(borderSize, borderSize, width, height);

                // calculate kernel
                calculateKernel();
        }

        static std::unique_ptr<CorrelationGrid> createGrid(
                int32_t width,
                int32_t height,
                double resolution,
                double smear_deviation)
        {
                assert(resolution != 0.0);

                // +1 in case of roundoff
                uint32_t border_size = getHalfKernelSize(smear_deviation, resolution) + 1;

                std::unique_ptr<CorrelationGrid> grid = std::make_unique<CorrelationGrid>(width, height, border_size, resolution,
                                                             smear_deviation);

                return grid;
        }

        inline const Rectangle2<int32_t> &getROI() const
        {
                return roi_;
        }

        /**
         * Gets the index into the data pointer of the given grid coordinate
         * @param rGrid
         * @param boundaryCheck
         * @return grid index
         */
        virtual int32_t getGridIndex(const Vector2i &grid, bool boundary_check = true) const
        {
                int32_t x = grid.x() + roi_.getX();
                int32_t y = grid.y() + roi_.getY();

                return Grid<uint8_t>::getGridIndex(Vector2i(x, y), boundary_check);
        }

        /**
         * Smear cell if the cell at the given point is marked as "occupied"
         * @param rGridPoint
         */
        inline void smearPoint(const Vector2i &grid_point)
        {
                assert(kernel_ != nullptr);

                int grid_index = getGridIndex(grid_point);
                if (getDataPointer()[grid_index] != static_cast<uint8_t>(GridStates::OCCUPIED)) {
                        return;
                }

                int32_t half_kernel = kernel_size_ / 2;

                // apply kernel
                for (int32_t j = -half_kernel; j <= half_kernel; j++) {
                        uint8_t *grid_adr = getDataPointer(
                                Vector2i(grid_point.x(), grid_point.y() + j));

                        int32_t kernel_constant = (half_kernel) + kernel_size_ * (j + half_kernel);

                        // if a point is on the edge of the grid, there is no problem
                        // with running over the edge of allowable memory, because
                        // the grid has margins to compensate for the kernel size
                        for (int32_t i = -half_kernel; i <= half_kernel; i++) {
                                int32_t kernel_array_index = i + kernel_constant;

                                uint8_t kernel_value = kernel_[kernel_array_index];
                                if (kernel_value > grid_adr[i]) {
                                        // kernel value is greater, so set it to kernel value
                                        grid_adr[i] = kernel_value;
                                }
                        }
                }
        }

protected:

        /**
         * Computes the kernel half-size based on the smear distance and the grid resolution.
         * Computes to two standard deviations to get 95% region and to reduce aliasing.
         * @param smearDeviation
         * @param resolution
         * @return kernel half-size based on the parameters
         */
        static int32_t getHalfKernelSize(double smear_deviation, double resolution)
        {
                assert(resolution != 0.0);

                return static_cast<int32_t>(std::round(2.0 * smear_deviation / resolution));
        }

        /**
         * Sets up the kernel for grid smearing.
         */
        virtual void calculateKernel()
        {
                double resolution = getResolution();

                assert(resolution != 0.0);
                assert(smear_deviation_ != 0.0);

                // min and max distance deviation for smearing;
                // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
                const double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
                const double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

                // check if given value too small or too big
                if (!math::InRange(smear_deviation_, MIN_SMEAR_DISTANCE_DEVIATION,
                                   MAX_SMEAR_DISTANCE_DEVIATION))
                {
                        std::stringstream error;
                        error << "Mapper Error:  Smear deviation too small:  Must be between " << MIN_SMEAR_DISTANCE_DEVIATION << " and " << MAX_SMEAR_DISTANCE_DEVIATION;
                        throw std::runtime_error(error.str());
                }

                // NOTE:  Currently assumes a two-dimensional kernel

                // +1 for center
                kernel_size_ = 2 * getHalfKernelSize(smear_deviation_, resolution) + 1;

                // allocate kernel
                kernel_ = std::make_unique<uint8_t[]>(kernel_size_ * kernel_size_);
                if (kernel_ == nullptr) {
                        throw std::runtime_error("Unable to allocate memory for kernel!");
                }

                // calculate kernel
                int32_t half_kernel = kernel_size_ / 2;
                for (int32_t i = -half_kernel; i <= half_kernel; i++)
                {
                        for (int32_t j = -half_kernel; j <= half_kernel; j++) {
                                #ifdef WIN32
                                double distance_from_mean = _hypot(i * resolution, j * resolution);
                                #else
                                double distance_from_mean = hypot(i * resolution, j * resolution);
                                #endif
                                double z = exp(-0.5 * pow(distance_from_mean / smear_deviation_, 2));

                                uint32_t kernel_value = static_cast<uint32_t>(std::round(z * static_cast<uint8_t>(GridStates::OCCUPIED)));
                                assert(math::IsUpTo(kernel_value, static_cast<uint32_t>(255)));

                                int kernelArrayIndex = (i + half_kernel) + kernel_size_ * (j + half_kernel);
                                kernel_[kernelArrayIndex] = static_cast<uint8_t>(kernel_value);
                        }
                }
        }

}; // CorrelationGrid


///////////////////////////////////////////////////////////////////////

class ScanMatcher
{
private:
        Mapper *mapper_;
        std::unique_ptr<CorrelationGrid> correlation_grid_;
        std::unique_ptr<Grid<double>> search_space_probs_;
        std::unique_ptr<GridIndexLookup<uint8_t>> grid_lookup_;
        std::unique_ptr<std::pair<double, Pose2>[]> pose_response_;
        std::vector<double> x_poses_;
        std::vector<double> y_poses_;
        Pose2 search_center_;
        double search_angle_offset_;
        uint32_t n_angles_;
        double search_angle_resolution_;
        bool do_penalize_;

        /**
         * Marks cells where scans' points hit as being occupied
         * @param scan scans whose points will mark cells in grid as being occupied
         * @param view_point do not add points that belong to scans "opposite" the view point
         */
        void addScans(const LocalizedRangeScanVector &scan, Eigen::Vector2d view_point);
        void addScans(const LocalizedRangeScanMap &scans, Eigen::Vector2d view_point);

        /**
         * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
         * @param pScan scan whose points will mark cells in grid as being occupied
         * @param viewPoint do not add points that belong to scans "opposite" the view point
         * @param doSmear whether the points will be smeared
         */
        void addScan(
            LocalizedRangeScan *scan, const Eigen::Vector2d &viewpoint,
            bool do_smear = true);

        /**
         * Compute which points in a scan are on the same side as the given viewpoint
         * @param pScan
         * @param rViewPoint
         * @return points on the same side
         */
        PointVectorDouble findValidPoints(
                LocalizedRangeScan *scan,
                const Eigen::Vector2d &viewpoint) const;

        /**
         * Get response at given position for given rotation (only look up valid points)
         * @param angleIndex
         * @param gridPositionIndex
         * @return response
         */
        double getResponse(uint32_t angle_index, int32_t grid_position_index) const;

public:
        ScanMatcher()
        {
        }

        /**
         * Default constructor
         */
        explicit ScanMatcher(Mapper *mapper)
            : mapper_(mapper),
              correlation_grid_(nullptr),
              search_space_probs_(nullptr),
              grid_lookup_(nullptr),
              pose_response_(nullptr),
              do_penalize_(false)
        {
        }

        /**
         * Parallelize scan matching
         */
        void operator()(const double &y) const;

        /**
         * Create a scan matcher with the given parameters
         */
        static std::unique_ptr<ScanMatcher> create(
                Mapper *mapper,
                double search_size,
                double resolution,
                double smear_deviation,
                double range_threshold);

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

        /**
         * Finds the best pose for the scan centering the search in the correlation grid
         * at the given pose and search in the space by the vector and angular offsets
         * in increments of the given resolutions
         * @param pScan scan to match against correlation grid
         * @param rSearchCenter the center of the search space
         * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
         * @param rSearchSpaceResolution how fine a granularity to search in the search space
         * @param searchAngleOffset searches poses in the angles offset by this angle around search center
         * @param searchAngleResolution how fine a granularity to search in the angular search space
         * @param doPenalize whether to penalize matches further from the search center
         * @param rMean output parameter of mean (best pose) of match
         * @param rCovariance output parameter of covariance of match
         * @param doingFineMatch whether to do a finer search after coarse search
         * @return strength of response
         */
        double correlateScan(
                LocalizedRangeScan *scan,
                const Pose2 &search_center,
                const Vector2d &search_space_offset,
                const Vector2d &search_space_resolution,
                double search_angle_offset,
                double search_angle_resolution,
                bool do_penalize,
                Pose2 &mean,
                Matrix3d &covariance,
                bool doing_fine_match);

        /**
         * Computes the positional covariance of the best pose
         * @param rBestPose
         * @param bestResponse
         * @param rSearchCenter
         * @param rSearchSpaceOffset
         * @param rSearchSpaceResolution
         * @param searchAngleResolution
         * @param rCovariance
         */
        void computePositionalCovariance(
            const Pose2 &best_pose,
            double best_response,
            const Pose2 &search_center,
            const Vector2d &search_space_offset,
            const Vector2d &search_space_resolution,
            double search_angle_resolution,
            Matrix3d &covariance);

        /**
         * Computes the angular covariance of the best pose
         * @param rBestPose
         * @param bestResponse
         * @param rSearchCenter
         * @param searchAngleOffset
         * @param searchAngleResolution
         * @param rCovariance
         */
        void computeAngularCovariance(
            const Pose2 &best_pose,
            double best_response,
            const Pose2 &search_center,
            double search_angle_offset,
            double search_angle_resolution,
            Matrix3d &covariance);
}; // ScanMatcher

///////////////////////////////////////////////////////////////////////

/**
* Graph traversal algorithm
*/
template<typename T>
class GraphTraversal
{
protected:
        Graph<T> *graph_;

public:
        GraphTraversal()
        {
        }

        explicit GraphTraversal(Graph<T> *pGraph)
            : graph_(pGraph)
        {
        }

public:
        virtual std::vector<T *> traverseForScans(Vertex<T> *start_vertex, Visitor<T> *visitor) = 0;
        virtual std::vector<Vertex<T> *> traverseForVertices(
            Vertex<T> *start_vertex,
            Visitor<T> *visitor) = 0;

}; // GraphTraversal<T>

///////////////////////////////////////////////////////////////////////

template<typename T>
class BreadthFirstTraversal : public GraphTraversal<T>
{
public:
        /**
         * Constructs a breadth-first traverser for the given graph
         */
        BreadthFirstTraversal()
        {
        }
        explicit BreadthFirstTraversal(Graph<T> *graph)
                : GraphTraversal<T>(graph)
        {
        }

public:
        /**
         * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
         * @param pStartVertex
         * @param pVisitor
         * @return visited vertice scans
         */
        virtual std::vector<T *> traverseForScans(Vertex<T> *start_vertex, Visitor<T> *visitor)
        {
                std::vector<Vertex<T> *> valid_vertices = traverseForVertices(start_vertex, visitor);

                std::vector<T *> objects;
                for (auto& vertex : valid_vertices) {
                        objects.push_back(vertex->getObject());
                }

                return objects;
        }

        /**
         * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
         * @param pStartVertex
         * @param pVisitor
         * @return visited vertices
         */
        virtual std::vector<Vertex<T> *> traverseForVertices(
                Vertex<T> *start_vertex,
                Visitor<T> *visitor)
        {
                std::queue<Vertex<T> *> to_visit;
                std::set<Vertex<T> *> seen_vertices;
                std::vector<Vertex<T> *> valid_vertices;

                to_visit.push(start_vertex);
                seen_vertices.insert(start_vertex);

                do {
                        Vertex<T> *next = to_visit.front();
                        to_visit.pop();

                        if (next != nullptr && visitor->visit(next)) {
                                // vertex is valid, explore neighbors
                                valid_vertices.push_back(next);

                                std::vector<Vertex<T> *> adjacent_vertices = next->getAdjacentVertices();
                                for (auto &vertex : adjacent_vertices) {
                                        if (seen_vertices.find(vertex) == seen_vertices.end()) {
                                                to_visit.push(vertex);
                                                seen_vertices.insert(vertex);
                                        }
                                }
                        }
                } while (to_visit.empty() == false);

                return valid_vertices;
        }


}; // BreadthFirstTraversal<T>

///////////////////////////////////////////////////////////////////////

/**
 * Visitor class
 */
template <typename T>
class Visitor
{
public:
        /**
         * Applies the visitor to the vertex
         * @param pVertex
         * @return true if the visitor accepted the vertex, false otherwise
         */
        virtual bool visit(Vertex<T> *vertex) = 0;
}; // Visitor<T>

///////////////////////////////////////////////////////////////////////
class NearScanVisitor : public Visitor<LocalizedRangeScan>
{
protected:
        Pose2 center_pose_;
        double max_distance_squared_;
        bool use_scan_barycenter_;
public:
        NearScanVisitor(LocalizedRangeScan *scan, double max_distance, bool use_scan_barycenter)
            : max_distance_squared_(math::Square(max_distance)),
              use_scan_barycenter_(use_scan_barycenter)
        {
                center_pose_ = scan->getReferencePose(use_scan_barycenter);
        }

        virtual bool visit(Vertex<LocalizedRangeScan> *vertex)
        {
                try {
                        LocalizedRangeScan *scan = vertex->getObject();
                        Pose2 pose = scan->getReferencePose(use_scan_barycenter_);
                        double squared_distance = pose.getSquaredDistance(center_pose_);
                        return squared_distance <= max_distance_squared_ - math::TOLERANCE;
                } catch (...) {
                        // relocalization vertex elements missing
                        std::cout << "Unable to visit valid vertex elements!" << std::endl;
                        return false;
                }
        }
}; // NearScanVisitor

///////////////////////////////////////////////////////////////////////

class MapperGraph : public Graph<LocalizedRangeScan>
{
private:
        /**
         * Mapper of this graph
         */
        Mapper *mapper_;

        /**
         * Scan matcher for loop closures
         */
        std::unique_ptr<ScanMatcher> loop_scan_matcher_;

        /**
         * Traversal algorithm to find near linked scans
         */
        std::unique_ptr<GraphTraversal<LocalizedRangeScan>> traversal_;

public:
        MapperGraph()
        {
        }

        /**
         * Graph for graph SLAM
         * @param pMapper
         * @param rangeThreshold
         */
        MapperGraph(Mapper *mapper, double range_threshold);

        /**
         * Adds a vertex representing the given scan to the graph
         * @param pScan
         */
        Vertex<LocalizedRangeScan> *addVertex(LocalizedRangeScan *scan);

        /**
         * Creates an edge between the source scan vertex and the target scan vertex if it
         * does not already exist; otherwise return the existing edge
         * @param pSourceScan
         * @param pTargetScan
         * @param rIsNewEdge set to true if the edge is new
         * @return edge between source and target scan vertices
         */
        Edge<LocalizedRangeScan> *addEdge(
                LocalizedRangeScan *source_scan,
                LocalizedRangeScan *target_scan,
                bool &is_new_edge);

        /**
         * Link scan to last scan and nearby chains of scans
         * @param pScan
         * @param rCovariance uncertainty of match
         */
        void addEdges(LocalizedRangeScan *scan, const Matrix3d &covariance);


        bool tryCloseLoop(LocalizedRangeScan *scan);

        /**
         * Optimizes scan poses
         */
        void correctPoses();

        /**
         * Find "nearby" (no further than given distance away) scans through graph links
         * @param pScan
         * @param maxDistance
         */
        LocalizedRangeScanVector findNearLinkedScans(LocalizedRangeScan *scan, double max_distance);

private:
        /**
         * Adds an edge between the two scans and labels the edge with the given mean and covariance
         * @param pFromScan
         * @param pToScan
         * @param rMean
         * @param rCovariance
         */
        void linkScans(
            LocalizedRangeScan *from_scan,
            LocalizedRangeScan *to_scan,
            const Pose2 &mean,
            const Matrix3d &covariance);

        /**
         * Finds the closest scan in the vector to the given pose
         * @param rScans
         * @param rPose
         */
        LocalizedRangeScan *getClosestScanToPose(
            const LocalizedRangeScanVector &scans,
            const Pose2 &pose) const;

        /**
         * Tries to find a chain of scan from the given device starting at the
         * given scan index that could possibly close a loop with the given scan
         * @param pScan
         * @param rSensorName
         * @param rStartNum
         * @return chain that can possibly close a loop with given scan
         */
        LocalizedRangeScanVector FindPossibleLoopClosure(
            LocalizedRangeScan *pScan,
            uint32_t &rStartNum);

        /**
         * Link the chain of scans to the given scan by finding the closest scan in the chain to the given scan
         * @param rChain
         * @param pScan
         * @param rMean
         * @param rCovariance
         */
        void linkChainToScan(
                const LocalizedRangeScanVector &chain,
                LocalizedRangeScan *scan,
                const Pose2 &mean,
                const Matrix3d &covariance);

        /**
         * Find nearby chains of scans and link them to scan if response is high enough
         * @param pScan
         * @param rMeans
         * @param rCovariances
         */
        void linkNearChains(
            LocalizedRangeScan *scan, Pose2Vector &means,
            std::vector<Matrix3d> &covariances);

        /**
         * Find chains of scans that are close to given scan
         * @param pScan
         * @return chains of scans
         */
        std::vector<LocalizedRangeScanVector> findNearChains(LocalizedRangeScan *scan);

        /**
         * Compute mean of poses weighted by covariances
         * @param rMeans
         * @param rCovariances
         * @return weighted mean
         */
        Pose2 computeWeightedMean(
            const Pose2Vector &means,
            const std::vector<Matrix3d> &covariances) const;

        /**
         * Gets the vertex associated with the given scan
         * @param pScan
         * @return vertex of scan
         */
        inline Vertex<LocalizedRangeScan> *getVertex(LocalizedRangeScan *scan)
        {
                std::map<int, std::unique_ptr<Vertex<LocalizedRangeScan>>>::iterator it = vertices_.find(
                    scan->getScanId());
                if (it != vertices_.end())
                {
                        return it->second.get();
                }
                else
                {
                        std::cout << "GetVertex: Failed to get vertex, idx " << scan->getScanId() << " is not in m_Vertices." << std::endl;
                        return nullptr;
                }
        }

}; // MapperGraph

///////////////////////////////////////////////////////////////////////

class ScanSolver
{
public:
        /**
         * Vector of id-pose pairs
         */
        typedef std::vector<std::pair<int32_t, Pose2>> IdPoseVector;

        ScanSolver()
        {
        }

        /**
         * Solve!
         */
        virtual void compute() = 0;

        /**
         * Adds a node to the solver
         */
        virtual void addNode(Vertex<LocalizedRangeScan> * /*vertex*/)
        {
        }

        /**
         * Adds a constraint to the solver
         */
        virtual void addConstraint(Edge<LocalizedRangeScan> * /*pEdge*/)
        {
        }

        /**
         * Resets the solver
         */
        virtual void clear()
        {
        }

        /**
         * Get corrected poses after optimization
         * @return optimized poses
         */
        virtual const IdPoseVector &getCorrections() const = 0;

}; // ScanSolver

///////////////////////////////////////////////////////////////////////

class Mapper 
{
        friend class ScanMatcher;
        friend class MapperGraph;
protected:
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

        // The range of angles to search during a coarse search and a finer search
        double fine_search_angle_offset_;
        double coarse_search_angle_offset_;

        // whether to increase the search space if no good matches are initially found
        bool use_response_expansion;

        // Resolution of angles to search during a coarse search
        double coarse_angle_resolution_;

        // Variance of penalty for deviating from odometry when scan-matching.
        // The penalty is a multiplier (less than 1.0) is a function of the
        // delta of the scan position being tested and the odometric pose
        double distance_variance_penalty_;
        double angle_variance_penalty_;

        // Minimum value of the penalty multiplier so scores do not
        // become too small
        double minimum_angle_penalty_;
        double minimum_distance_penalty_;

        /**
         * The size of the search grid used by the matcher.
         * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
         */
        double correlation_search_space_dimension_;

        /**
         * The resolution (size of a grid cell) of the correlation grid.
         * Default value is 0.01 meters.
         */
        double correlation_search_space_resolution_;

        /**
         * The point readings are smeared by this value in X and Y to create a smoother response.
         * Default value is 0.03 meters.
         */
        double correlation_search_space_smear_deviation_;

        /**
         * Default value is true.
         */
        bool use_scan_barycenter_;

        /**
         * Scans are linked only if the correlation response value is greater than this value.
         * Default value is 0.4
         */
        double link_match_minimum_response_fine_;

        /**
         * Maximum distance between linked scans.  Scans that are farther apart will not be linked
         * regardless of the correlation response value.
         * Default value is 6.0 meters.
         */
        double link_scan_maximum_distance_;

        /**
         * Scans less than this distance from the current position will be considered for a match
         * in loop closure.
         * Default value is 4.0 meters.
         */
        double loop_search_maximum_distance_;

        /**
         * When the loop closure detection finds a candidate it must be part of a large
         * set of linked scans. If the chain of scans is less than this value we do not attempt
         * to close the loop.
         * Default value is 10.
         */
        uint32_t loop_match_minimum_chain_size_;

        /**
         * The co-variance values for a possible loop closure have to be less than this value
         * to consider a viable solution. This applies to the coarse search.
         * Default value is 0.16.
         */
        double loop_match_maximum_variance_coarse_;

        /**
         * If response is larger then this, then initiate loop closure search at the coarse resolution.
         * Default value is 0.7.
         */
        double loop_match_minimum_response_coarse_;

        /**
         * If response is larger then this, then initiate loop closure search at the fine resolution.
         * Default value is 0.7.
         */
        double loop_match_minimum_response_fine_;

        /**
         * The size of the search grid used by the matcher.
         * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
         */
        double loop_search_space_dimension_;

        /**
         * The resolution (size of a grid cell) of the correlation grid.
         * Default value is 0.01 meters.
         */
        double loop_search_space_resolution_;

        /**
         * The point readings are smeared by this value in X and Y to create a smoother response.
         * Default value is 0.03 meters.
         */
        double loop_search_space_smear_deviation_;

        std::unique_ptr<ScanManager> scan_manager_;
        std::unique_ptr<ScanMatcher> scan_matcher_;
        std::unique_ptr<MapperGraph> graph_;
        std::unique_ptr<ScanSolver> scan_optimizer_;
protected:
        bool hasMovedEnough(LocalizedRangeScan *scan, LocalizedRangeScan *last_scan) const;

public:
        Mapper()
        : scan_manager_(nullptr),
        initialized_(false)
        {
        }

        void initialize(double range_threshold)
        {
                if (initialized_ == true) {
                        return;
                }

                // create sequential scan and loop matcher

                scan_matcher_ = ScanMatcher::create(
                        this,
                        correlation_search_space_dimension_,
                        correlation_search_space_resolution_,
                        correlation_search_space_smear_deviation_,
                        range_threshold);
                assert(scan_matcher_);

                scan_manager_ = std::make_unique<ScanManager>(scan_buffer_size_, scan_buffer_maximum_scan_distance_);

                graph_ = std::make_unique<MapperGraph>(this, range_threshold);

                initialized_ = true;
        }

        template <class NodeT>
        void configure(const NodeT &node);

        inline double getMinimumTravelDistance() const {
                return minimum_travel_distance_;
        }

        /**
         * @return Minimum travel heading in radians 
         */
        inline double getMinimumTravelHeading() const {
                return minimum_travel_heading_;
        }

        void setScanSolver(std::unique_ptr<ScanSolver> scan_optimizer)
        {
                scan_optimizer_ = std::move(scan_optimizer);
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

public:
        void setUseScanBarycenter(bool b) 
        {
                use_scan_barycenter_ = b;
        }

        void setScanBufferSize(int i)
        {
                scan_buffer_size_ = (uint32_t)i;
        }

        void setScanBufferMaximumScanDistance(double d)
        {
                scan_buffer_maximum_scan_distance_ = d;
        }

        void setLinkMatchMinimumResponseFine(double d)
        {
                link_match_minimum_response_fine_ = d;
        }

        void setLinkScanMaximumDistance(double d)
        {
                link_scan_maximum_distance_ = d;
        }

        void setLoopSearchMaximumDistance(double d)
        {
                loop_search_maximum_distance_ = d;
                
        }

        void setLoopMatchMinimumChainSize(int i)
        {
                loop_match_minimum_chain_size_ = (uint32_t)i;
        }

        void setLoopMatchMaximumVarianceCoarse(double d)
        {
               loop_match_maximum_variance_coarse_ = d;
        }

        void setLoopMatchMinimumResponseCoarse(double d)
        {
               loop_match_minimum_response_coarse_ = d; 
        }

        void setLoopMatchMinimumResponseFine(double d)
        {
                loop_match_minimum_response_fine_ = d;
        }

        // Correlation Parameters - Correlation Parameters
        void setCorrelationSearchSpaceDimension(double d)
        {
               correlation_search_space_dimension_ = d;
        }

        void setCorrelationSearchSpaceResolution(double d)
        {
                correlation_search_space_resolution_ = d;
        }

        void setCorrelationSearchSpaceSmearDeviation(double d)
        {
                correlation_search_space_smear_deviation_ = d;
        }

        // Correlation Parameters - Loop Closure Parameters
        void setLoopSearchSpaceDimension(double d)
        {
                loop_search_space_dimension_ = d;
        }

        void setLoopSearchSpaceResolution(double d)
        {
               loop_search_space_resolution_ = d;
        }

        void setLoopSearchSpaceSmearDeviation(double d)
        {
                loop_search_space_smear_deviation_ = d;
        }

        // Scan Matcher Parameters
        void setDistanceVariancePenalty(double d)
        {
                distance_variance_penalty_ = math::Square(d);
        }

        void setAngleVariancePenalty(double d)
        {
                angle_variance_penalty_ = math::Square(d);
        }

        void setFineSearchAngleOffset(double d)
        {
                fine_search_angle_offset_ = d;
        }

        void setCoarseSearchAngleOffset(double d)
        {
                coarse_search_angle_offset_ = d;
        }

        void setCoarseAngleResolution(double d)
        {
                coarse_angle_resolution_ = d;
        }

        void setMinimumAnglePenalty(double d)
        {
                minimum_angle_penalty_ = d;
        }

        void setMinimumDistancePenalty(double d)
        {
                minimum_distance_penalty_ = d;
        }

        void setUseResponseExpansion(bool b)
        {
                use_response_expansion = b;
        }

        void setMinPassThrough(int i)
        {
                min_pass_through_ = (uint32_t)i;
        }

        void setOccupancyThreshold(double d)
        {
                occupancy_threshold_ = d;
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
                             tmp_pose.transform.translation.y, 
                             yaw);
                std::cout << "x: " << tmp_pose.transform.translation.x
                        << " y: " << tmp_pose.transform.translation.y 
                        << " heading: " << yaw << std::endl;

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