#ifndef MAPPER_UTILS_HPP
#define MAPPER_UTILS_HPP

#include <unordered_map>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/time.hpp"

#include "myslam/myslam_types.hpp"


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
// inline void toNavMap(
//         const OccupancyGrid *occ_grid,
//         nav_msgs::msg::OccupancyGrid &map);

////////////////////////////////////////////////////////////

class LaserRangeFinder
{
public:
        LaserRangeFinder()
        {

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
                        point_readings_.clear();

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
                        }
                        else
                        {
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

template<typename T>
class Grid 
{
public:
        Grid()
        {

        }

};

//////////////////////////////////////////////////////////////

class CoordinateConverter
{
public:
        CoordinateConverter()
        {
        }
};

//////////////////////////////////////////////////////////////

class OccupancyGrid
{
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
         * Adds the scan's information to this grid's counters (optionally
         * update the grid's cells' occupancy status)
         * @param scan
         * @param doUpdate whether to update the grid's cell's occupancy status
         * @return returns false if an endpoint fell off the grid, otherwise true
         */
        bool addScan(
            LocalizedRangeScan *scan,
            bool doUpdate = false);

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
            bool do_update = false);

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
         * @param rScans
         */
        void createFromScans(const std::vector<LocalizedRangeScan *> &rScans);

private:
        int32_t width_;  // width of grid
        int32_t height_; // height of grid
        double occupancy_threshold_;
        uint32_t min_pass_through_;
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


} // namespace mapper_utils

#endif // MAPPER_UTILS_HPP