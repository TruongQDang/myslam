#ifndef MAPPER_UTILS_HPP
#define MAPPER_UTILS_HPP

#include <unordered_map>

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
class Grid;
class OccupancyGrid;
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

private:
        int32_t scan_id_;
        Pose2 corrected_pose_;
        Pose2 odom_pose_;
        std::unique_ptr<double[]> range_readings_;
        rclcpp::Time time_;
}; // LocalizedRangeScan

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

        static void ComputeDimensions(
                const std::vector<LocalizedRangeScan *> &rScans,
                double resolution,
                int32_t &rWidth,
                int32_t &rHeight,
                Eigen::Vector2d &rOffset);

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