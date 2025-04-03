#ifndef MYSLAM_MAPPER_HPP
#define MYSLAM_MAPPER_HPP

#include <unordered_map>

#include "myslam/laser_helper.hpp"

namespace mapper_utils
{

class Mapper;
class ScanManager;
class OccupancyGrid;

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
        bool process(laser_utils::LocalizedRangeScan *scan, Eigen::Matrix3d *covariance = nullptr);

        /**
         * Returns all processed scans added to the mapper.
         * NOTE: The returned scans have their corrected pose updated.
         * @return list of scans received and processed by the mapper. If no scans have been processed,
         * return an empty list.
         */
        const std::vector<laser_utils::LocalizedRangeScan *> getAllProcessedScans() const;

private:
        ScanManager *scan_manager_;
        // parameters
        double minimum_travel_distance_;
        double minimum_travel_heading_;
};

class ScanManager
{
public:
        ScanManager() 
        {
        }

        /**
         * Gets last scan of given sensor
         * @return last localized range scan of sensor
         */
        inline laser_utils::LocalizedRangeScan * getLastScan()
        {
                return last_scan_;
        }

        inline void addScan(laser_utils::LocalizedRangeScan *scan) 
        {
                // assign unique scan id
                scan->setScanId(next_scan_id_);
                next_scan_id_++;
                // add to scan buffer
                scans_.emplace(scan->getScanId(), scan);
        }

        inline void setLastScan(laser_utils::LocalizedRangeScan *scan)
        {
                last_scan_ = scan;
        }


private:
        std::map<int, laser_utils::LocalizedRangeScan *> scans_;
        std::vector<laser_utils::LocalizedRangeScan *> running_scans_;
        laser_utils::LocalizedRangeScan *last_scan_;
        uint32_t next_scan_id_;
        
        uint32_t running_buffer_maximum_size_;
        double running_buffer_maximum_distance_;

}; 

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
        static OccupancyGrid * createFromScans(
                const std::vector<laser_utils::LocalizedRangeScan *> &scans,
                double resolution, 
                uint32_t min_pass_through,
                double occupancy_threshold);

        /**
         * Adds the scan's information to this grid's counters (optionally
         * update the grid's cells' occupancy status)
         * @param scan
         * @param doUpdate whether to update the grid's cell's occupancy status
         * @return returns false if an endpoint fell off the grid, otherwise true
         */
        bool addScan(
                laser_utils::LocalizedRangeScan *scan, 
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
                const Eigen::Vector2d & world_to,
                bool is_endpoint_valid,
                bool do_update = false);

private:
        ////////////////////////////////////////////////////////////
        // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
        // then not many beams will hit the cell!

        // Number of beams that must pass through a cell before it will be considered to be occupied
        // or unoccupied.  This prevents stray beams from messing up the map.
        uint32_t *min_pass_through_;

        // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
        double *occupancy_threshold_;
};

} // namespace mapper_utils

#endif // MYSLAM_MAPPER_HPP