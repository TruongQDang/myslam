#include "myslam/mapper_utils.hpp"
#include <cmath>

namespace mapper_utils
{

#define MAX_VARIANCE 500.0
#define DISTANCE_PENALTY_GAIN 0.2
#define ANGLE_PENALTY_GAIN 0.2

void CellUpdater::operator()(uint32_t index)
{
        uint8_t *data_ptr = occupancy_grid_->getDataPointer();
        uint32_t *cell_pass_cnt_ptr = occupancy_grid_->cell_pass_cnt_->getDataPointer();
        uint32_t *cell_hit_cnt_ptr = occupancy_grid_->cell_hit_cnt_->getDataPointer();

        occupancy_grid_->updateCell(&data_ptr[index], cell_pass_cnt_ptr[index], cell_hit_cnt_ptr[index]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Create a scan matcher with the given parameters
 */
std::unique_ptr<ScanMatcher> ScanMatcher::create(
        Mapper *mapper,
        double search_size,
        double resolution,
        double smear_deviation,
        double range_threshold)
{
        // invalid parameters
        if (resolution <= 0)
        {
                return nullptr;
        }
        if (search_size <= 0)
        {
                return nullptr;
        }
        if (smear_deviation < 0)
        {
                return nullptr;
        }
        if (range_threshold <= 0)
        {
                return nullptr;
        }

        assert(math::DoubleEqual(std::round(search_size / resolution), (search_size / resolution)));

        // calculate search space in grid coordinates
        uint32_t search_space_side_size = static_cast<uint32_t>(std::round(search_size / resolution) + 1);

        // compute requisite size of correlation grid (pad grid so that scan
        // points can't fall off the grid
        // if a scan is on the border of the search space)
        uint32_t point_reading_margin = static_cast<uint32_t>(ceil(range_threshold / resolution));

        int32_t grid_size = search_space_side_size + 2 * point_reading_margin;
        
        assert(grid_size % 2 == 1);

        std::unique_ptr<ScanMatcher> scan_matcher = std::make_unique<ScanMatcher>(mapper);
        scan_matcher->correlation_grid_ = CorrelationGrid::createGrid(
                grid_size, 
                grid_size, 
                resolution,
                smear_deviation);
        scan_matcher->search_space_probs_ = Grid<double>::createGrid(
                search_space_side_size,
                search_space_side_size,
                resolution);
        scan_matcher->grid_lookup_ = std::make_unique<GridIndexLookup<uint8_t>>(scan_matcher->correlation_grid_.get());

        return scan_matcher;
}

template<class T>
double ScanMatcher::matchScan(
        LocalizedRangeScan *scan,
        const T &base_scans,
        Pose2 &mean, Eigen::Matrix3d &covariance,
        bool do_penalize,
        bool do_refine_match)
{
        ///////////////////////////////////////
        // set scan pose to be center of grid

        // 1. get scan position
        Pose2 scan_pose = scan->getSensorPose();

        // scan has no readings; cannot do scan matching
        // best guess of pose is based off of adjusted odometer reading
        if (scan->getNumberOfRangeReadings() == 0) {
                mean = scan_pose;

                // maximum covariance
                covariance(0, 0) = MAX_VARIANCE; // XX
                covariance(1, 1) = MAX_VARIANCE; // YY
                covariance(2, 2) =
                        4 * math::Square(mapper_->coarse_angle_resolution_); // TH*TH

                return 0.0;
        }

        // 2. get size of grid
        Rectangle2<int32_t> roi = correlation_grid_->getROI();

        // 3. compute offset (in meters - lower left corner)
        Vector2d offset;
        offset.x() = scan_pose.getX() - 0.5 * (roi.getWidth() - 1) * correlation_grid_->getResolution();
        offset.y() = scan_pose.getY() - 0.5 * (roi.getHeight() - 1) * correlation_grid_->getResolution();

        // 4. set offset
        correlation_grid_->getCoordinateConverter()->setOffset(offset);

        ///////////////////////////////////////

        // set up correlation grid
        addScans(base_scans, scan_pose.getPosition());

        // compute how far to search in each direction
        Vector2d search_dimensions(
                search_space_probs_->getWidth(), 
                search_space_probs_->getHeight());
        Vector2d coarse_search_offset(
                0.5 * (search_dimensions.x() - 1) * correlation_grid_->getResolution(),
                0.5 * (search_dimensions.y() - 1) * correlation_grid_->getResolution());

        // a coarse search only checks half the cells in each dimension
        Vector2d coarse_search_resolution(
                2 * correlation_grid_->getResolution(),
                2 * correlation_grid_->getResolution());

        // actual scan-matching
        double best_response = correlateScan(
                scan, 
                scan_pose, 
                coarse_search_offset,
                coarse_search_resolution,
                mapper_->coarse_search_angle_offset_,
                mapper_->coarse_angle_resolution_,
                do_penalize, 
                mean, 
                covariance, 
                false);
        
        if (mapper_->use_response_expansion == true) {
                if (math::DoubleEqual(best_response, 0.0)) {
                        #ifdef MYSLAM_DEBUG
                        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
                        #endif
                        // try and increase search angle offset with 20 degrees and do another match
                        double new_search_angle_offset = mapper_->coarse_search_angle_offset_;
                        for (uint32_t i = 0; i < 3; i++) {
                                new_search_angle_offset += math::DegreesToRadians(20);

                                best_response = correlateScan(
                                        scan,
                                        scan_pose,
                                        coarse_search_offset,
                                        coarse_search_resolution,
                                        new_search_angle_offset,
                                        mapper_->coarse_angle_resolution_,
                                        do_penalize,
                                        mean,
                                        covariance,
                                        false);

                                if (math::DoubleEqual(best_response, 0.0) == false) {
                                        break;
                                }
                        }

                        #ifdef MYSLAM_DEBUG
                        if (math::DoubleEqual(best_response, 0.0)) {
                                std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
                        }
                        #endif
                }
        }

        if (do_refine_match) {
                Vector2d fine_search_offset(coarse_search_resolution * 0.5);
                Vector2d fine_search_resolution(
                        correlation_grid_->getResolution(),
                        correlation_grid_->getResolution());
                best_response = correlateScan(
                        scan,
                        mean,
                        fine_search_offset,
                        fine_search_resolution,
                        0.5 * mapper_->coarse_angle_resolution_,
                        mapper_->fine_search_angle_offset_,
                        do_penalize,
                        mean,
                        covariance,
                        true);
        }

        #ifdef MYSLAM_DEBUG
        std::cout << "  BEST POSE = " << mean << " BEST RESPONSE = " << best_response << ",  VARIANCE = " << covariance(0, 0) << ", " << covariance(1, 1) << std::endl;
        #endif

        assert(math::InRange(mean.getHeading(), -math::PI, math::PI));

        return best_response;
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::addScans(
        const LocalizedRangeScanVector &scans, 
        Vector2d viewpoint)
{
        correlation_grid_->clear();

        // add all scans to grid
        for (const auto& scan : scans) {
                if (scan == nullptr) {
                        continue;
                }

                addScan(scan, viewpoint);
        }
}

/**
 * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
 * @param pScan scan whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 * @param doSmear whether the points will be smeared
 */
void ScanMatcher::addScan(
        LocalizedRangeScan *scan, 
        const Vector2d &viewpoint,
        bool do_smear)
{
        PointVectorDouble valid_points = findValidPoints(scan, viewpoint);

        // put in all valid points
        for (const auto& point : valid_points) {
                Vector2i grid_point = correlation_grid_->convertWorldToGrid(point);
                if (!math::IsUpTo(grid_point.x(), correlation_grid_->getROI().getWidth()) ||
                    !math::IsUpTo(grid_point.y(), correlation_grid_->getROI().getHeight())) {
                        // point not in grid
                        continue;
                }

                int grid_index = correlation_grid_->getGridIndex(grid_point);

                // set grid cell as occupied
                if (correlation_grid_->getDataPointer()[grid_index] 
                        == static_cast<uint8_t>(GridStates::OCCUPIED)) {
                                // value already set
                                continue;
                }

                correlation_grid_->getDataPointer()[grid_index] = static_cast<uint8_t>(GridStates::OCCUPIED);

                // smear_grid
                if (do_smear == true) {
                        correlation_grid_->smearPoint(grid_point); 
                }
        }
}

/**
 * Compute which points in a scan are on the same side as the given viewpoint
 * @param pScan
 * @param rViewPoint
 * @return points on the same side
 */
PointVectorDouble ScanMatcher::findValidPoints(
        LocalizedRangeScan *scan,
        const Vector2d &viewpoint) const
{
        const PointVectorDouble &point_readings = scan->getPointReadings();

        // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
        const double min_square_distance = math::Square(0.1); // in m^2

        // this iterator lags from the main iterator adding points only when the points are on
        // the same side as the viewpoint
        PointVectorDouble::const_iterator trailing_point_iter = point_readings.begin();
        PointVectorDouble valid_points;

        Vector2d first_point;
        bool first_time = true;
        for (PointVectorDouble::const_iterator iter = point_readings.begin(); iter != point_readings.end(); ++iter) {
                Vector2d current_point = *iter;

                if (first_time && !std::isnan(current_point.x()) && !std::isnan(current_point.y())) {
                        first_point = current_point;
                        first_time = false;
                }

                Vector2d delta = first_point - current_point;
                if (delta.squaredNorm() > min_square_distance) {
                        // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
                        // Which computes the direction of rotation, if the rotation is counterclock
                        // wise then we are looking at data we should keep. If it's negative rotation
                        // we should not included in in the matching
                        // have enough distance, check viewpoint
                        double a = viewpoint.y() - first_point.y();
                        double b = first_point.x() - viewpoint.x();
                        double c = first_point.y() * viewpoint.x() - first_point.x() * viewpoint.y();
                        double ss = current_point.x() * a + current_point.y() * b + c;

                        first_point = current_point;

                        if (ss < 0.0) { 
                                // wrong side, skip and keep going
                                trailing_point_iter = iter;
                        } else {
                                for (; trailing_point_iter != iter; ++trailing_point_iter) {
                                        valid_points.push_back(*trailing_point_iter);
                                }
                        }
                }
        }

        return valid_points;
}

/**
 * Finds the best pose for the scan centering the search in the correlation grid
 * at the given pose and search in the space by the vector and angular offsets
 * in increments of the given resolutions
 * @param rScan scan to match against correlation grid
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
double ScanMatcher::correlateScan(
        LocalizedRangeScan *scan,
        const Pose2 &search_center,
        const Vector2d &search_space_offset,
        const Vector2d &search_space_resolution,
        double search_angle_offset,
        double search_angle_resolution,
        bool do_penalize,
        Pose2 &mean,
        Matrix3d &covariance,
        bool doing_fine_match)
{
        assert(search_angle_resolution != 0.0);

        // setup lookup arrays
        grid_lookup_->computeOffsets(
                scan,
                search_center.getHeading(),
                search_angle_offset, 
                search_angle_resolution);

        // only initialize probability grid if computing positional covariance (during coarse match)
        if (!doing_fine_match) {
                search_space_probs_->clear();

                // position search grid - finds lower left corner of search grid
                Vector2d offset(search_center.getPosition() - search_space_offset);
                search_space_probs_->getCoordinateConverter()->setOffset(offset);
        }

        // calculate position arrays
        x_poses_.clear();
        uint32_t n_x = static_cast<uint32_t>(
                std::round(search_space_offset.x() * 2.0 / search_space_resolution.x()) + 1);
        double start_x = -search_space_offset.x();
        for (uint32_t x_index = 0; x_index < n_x; x_index++) {
                x_poses_.push_back(start_x + x_index * search_space_resolution.x());
        }
        assert(math::DoubleEqual(x_poses_.back(), -start_x));

        y_poses_.clear();
        uint32_t n_y = static_cast<uint32_t>(
            std::round(search_space_offset.y() * 2.0 / search_space_resolution.y()) + 1);
        double start_y = -search_space_offset.y();
        for (uint32_t y_index = 0; y_index < n_y; y_index++) {
                y_poses_.push_back(start_y + y_index * search_space_resolution.y());
        }
        assert(math::DoubleEqual(y_poses_.back(), -start_y));

        // calculate pose response array size
        uint32_t n_angles = 
                static_cast<uint32_t>(std::round(search_angle_offset * 2.0 / search_angle_resolution) + 1);

        uint32_t pose_response_size = static_cast<uint32_t>(x_poses_.size() * y_poses_.size() * n_angles);

        // allocate array
        pose_response_ = std::make_unique<std::pair<double, Pose2>[]>(pose_response_size);

        Vector2i start_grid_point = correlation_grid_->convertWorldToGrid(
                Vector2d(
                        search_center.getX() + start_x,
                        search_center.getY() + start_y));

        // this isn't good but its the fastest way to iterate. Should clean up later.
        search_center_ = search_center;
        search_angle_offset_ = search_angle_offset;
        n_angles_ = n_angles;
        search_angle_resolution_ = search_angle_resolution;
        do_penalize_ = do_penalize;
        tbb::parallel_for_each(y_poses_, (*this));

        // find value of best response (in [0; 1])
        double best_response = -1;
        for (uint32_t i = 0; i < pose_response_size; i++) {
                best_response = std::max(best_response, pose_response_[i].first);

                // will compute positional covariance, save best relative probability for each cell
                if (!doing_fine_match) {
                        const Pose2 &pose = pose_response_[i].second;
                        Vector2i grid = search_space_probs_->convertWorldToGrid(pose.getPosition());
                        double *ptr;

                        try {
                                ptr = (double *)(search_space_probs_->getDataPointer(grid)); // NOLINT
                        } catch (...) {
                                throw std::runtime_error("Mapper FATAL ERROR - "
                                                         "unable to get pointer in probability search!");
                        }

                        if (ptr == nullptr) {
                                throw std::runtime_error("Mapper FATAL ERROR - "
                                                         "Index out of range in probability search!");
                        }

                        *ptr = std::max(pose_response_[i].first, *ptr);
                }
        }

        // average all poses with same highest response
        Vector2d average_position;
        double theta_x = 0.0;
        double theta_y = 0.0;
        int32_t average_pose_count = 0;
        for (uint32_t i = 0; i < pose_response_size; i++) {
                if (math::DoubleEqual(pose_response_[i].first, best_response)) {
                        average_position += pose_response_[i].second.getPosition();

                        double heading = pose_response_[i].second.getHeading();
                        theta_x += cos(heading);
                        theta_y += sin(heading);

                        average_pose_count++;
                }
        }

        Pose2 average_pose;
        if (average_pose_count > 0) {
                average_position /= average_pose_count;

                theta_x /= average_pose_count;
                theta_y /= average_pose_count;

                average_pose = Pose2(average_position, atan2(theta_y, theta_x));
        } else {
                throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
        }

        // delete pose response array
        pose_response_.reset();

        #ifdef MYSLAM_DEBUG
                std::cout << "best_pose: " << average_pose << std::endl;
                std::cout << "best_response: " << best_response << std::endl;
        #endif

        if (!doing_fine_match) {
                computePositionalCovariance(average_pose, best_response, search_center, search_space_offset,
                                                search_space_resolution, search_angle_resolution, covariance);
        } else {
                computeAngularCovariance(average_pose, best_response, search_center,
                                                search_angle_offset, search_angle_resolution, covariance);
        }

        mean = average_pose;

        #ifdef MYSLAM_DEBUG
                std::cout << "best_pose: " << average_pose << std::endl;
        #endif

        if (best_response > 1.0) {
                best_response = 1.0;
        }

        assert(math::InRange(best_response, 0.0, 1.0));
        assert(math::InRange(mean.getHeading(), -math::PI, math::PI));

        return best_response;
}

void ScanMatcher::operator()(const double &y) const
{
        uint32_t poseResponseCounter;
        uint32_t x_pose;
        uint32_t y_pose = std::find(y_poses_.begin(), y_poses_.end(), y) - y_poses_.begin();

        const uint32_t size_x = x_poses_.size();

        double newPositionY = search_center_.getY() + y;
        double squareY = math::Square(y);

        for (std::vector<double>::const_iterator xIter = x_poses_.begin(); xIter != x_poses_.end();
             ++xIter) {
                x_pose = std::distance(x_poses_.begin(), xIter);
                double x = *xIter;
                double newPositionX = search_center_.getX() + x;
                double squareX = math::Square(x);

                Vector2i gridPoint =
                        correlation_grid_->convertWorldToGrid(Vector2d(newPositionX, newPositionY));
                int32_t gridIndex = correlation_grid_->getGridIndex(gridPoint);
                assert(gridIndex >= 0);

                double angle = 0.0;
                double startAngle = search_center_.getHeading() - search_angle_offset_;
                for (uint32_t angleIndex = 0; angleIndex < n_angles_; angleIndex++)
                {
                        angle = startAngle + angleIndex * search_angle_resolution_;

                        double response = getResponse(angleIndex, gridIndex);
                        if (do_penalize_ && (math::DoubleEqual(response, 0.0) == false)) {
                                // simple model (approximate Gaussian) to take odometry into account
                                double squaredDistance = squareX + squareY;
                                double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN *
                                                                squaredDistance / mapper_->distance_variance_penalty_);
                                distancePenalty = std::max(distancePenalty,
                                                           mapper_->minimum_distance_penalty_);

                                double squaredAngleDistance = math::Square(angle - search_center_.getHeading());
                                double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN *
                                                             squaredAngleDistance / mapper_->angle_variance_penalty_);
                                anglePenalty = std::max(anglePenalty, mapper_->minimum_angle_penalty_);

                                response *= (distancePenalty * anglePenalty);
                        }

                        // store response and pose
                        poseResponseCounter = (y_pose * size_x + x_pose) * (n_angles_) + angleIndex;
                        pose_response_[poseResponseCounter] =
                            std::pair<double, Pose2>(response, Pose2(newPositionX, newPositionY,
                                                                     math::NormalizeAngle(angle)));
                }
        }
}

double ScanMatcher::getResponse(uint32_t angle_index, int32_t grid_position_index) const
{
        double response = 0.0;

        // add up value for each point
        uint8_t *pByte = correlation_grid_->getDataPointer() + grid_position_index;

        const LookupArray *pOffsets = grid_lookup_->getLookupArray(angle_index);
        assert(pOffsets != NULL);

        // get number of points in offset list
        uint32_t nPoints = pOffsets->getSize();
        if (nPoints == 0) {
                return response;
        }

        // calculate response
        int32_t *pAngleIndexPointer = pOffsets->getArrayPointer();
        for (uint32_t i = 0; i < nPoints; i++)
        {
                // ignore points that fall off the grid
                int32_t pointGridIndex = grid_position_index + pAngleIndexPointer[i];
                if (!math::IsUpTo(pointGridIndex, correlation_grid_->getDataSize()) ||
                        pAngleIndexPointer[i] == math::INVALID_SCAN) {
                        continue;
                }

                // uses index offsets to efficiently find location of point in the grid
                response += pByte[pAngleIndexPointer[i]];
        }

        // normalize response
        response /= (nPoints * static_cast<uint8_t>(GridStates::OCCUPIED));
        assert(fabs(response) <= 1.0);

        return response;
}

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
void ScanMatcher::computePositionalCovariance(
        const Pose2 &best_pose,
        double best_response,
        const Pose2 &search_center,
        const Vector2d &search_space_offset,
        const Vector2d &search_space_resolution,
        double search_angle_resolution,
        Matrix3d &covariance)
{
        // reset covariance to identity matrix
        covariance.Identity();

        // if best response is vary small return max variance
        if (best_response < math::TOLERANCE)
        {
                covariance(0, 0) = MAX_VARIANCE;                            // XX
                covariance(1, 1) = MAX_VARIANCE;                            // YY
                covariance(2, 2) = 4 * math::Square(search_angle_resolution); // TH*TH

                return;
        }

        double accumulated_variance_xx = 0;
        double accumulated_variance_xy = 0;
        double accumulated_variance_yy = 0;
        double norm = 0;

        double dx = best_pose.getX() - search_center.getX();
        double dy = best_pose.getY() - search_center.getY();

        double offset_x = search_space_offset.x();
        double offset_y = search_space_offset.y();

        uint32_t n_x =
                static_cast<uint32_t>(std::round(offset_x * 2.0 / search_space_resolution.x()) + 1);
        double start_x = -offset_x;
        assert(math::DoubleEqual(start_x + (n_x - 1) * search_space_resolution.x(), -start_x));

        uint32_t n_y =
            static_cast<uint32_t>(std::round(offset_y * 2.0 / search_space_resolution.y()) + 1);
        double start_y = -offset_y;
        assert(math::DoubleEqual(start_y + (n_y - 1) * search_space_resolution.y(), -start_y));

        for (uint32_t y_index = 0; y_index < n_y; y_index++) {
                double y = start_y + y_index * search_space_resolution.y();

                for (uint32_t x_index = 0; x_index < n_x; x_index++) {
                        double x = start_x + x_index * search_space_resolution.x();

                        Vector2i grid_point =
                                search_space_probs_->convertWorldToGrid(Vector2d(search_center.getX() + x,
                                                                                search_center.getY() + y));
                        double response = *(search_space_probs_->getDataPointer(grid_point));

                        // response is not a low response
                        if (response >= (best_response - 0.1)) {
                                norm += response;
                                accumulated_variance_xx += (math::Square(x - dx) * response);
                                accumulated_variance_xy += ((x - dx) * (y - dy) * response);
                                accumulated_variance_yy += (math::Square(y - dy) * response);
                        }
                }
        }

        if (norm > math::TOLERANCE) {
                double variance_xx = accumulated_variance_xx / norm;
                double variance_xy = accumulated_variance_xy / norm;
                double variance_yy = accumulated_variance_yy / norm;
                double variance_thth = 4 * math::Square(search_angle_resolution);

                // lower-bound variances so that they are not too small;
                // ensures that links are not too tight
                double min_variance_xx = 0.1 * math::Square(search_space_resolution.x());
                double min_variance_yy = 0.1 * math::Square(search_space_resolution.y());
                variance_xx = std::max(variance_xx, min_variance_xx);
                variance_yy = std::max(variance_yy, min_variance_yy);

                // increase variance for poorer responses
                double multiplier = 1.0 / best_response;
                covariance(0, 0) = variance_xx * multiplier;
                covariance(0, 1) = variance_xy * multiplier;
                covariance(1, 0) = variance_xy * multiplier;
                covariance(1, 1) = variance_yy * multiplier;
                covariance(2, 2) = variance_thth; // this value will be set in ComputeAngularCovariance
        }

        // if values are 0, set to MAX_VARIANCE
        // values might be 0 if points are too sparse and thus don't hit other points
        if (math::DoubleEqual(covariance(0, 0), 0.0)) {
                covariance(0, 0) = MAX_VARIANCE;
        }

        if (math::DoubleEqual(covariance(1, 1), 0.0)) {
                covariance(1, 1) = MAX_VARIANCE;
        }
}

/**
 * Computes the angular covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param searchAngleOffset
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::computeAngularCovariance(
        const Pose2 &best_pose,
        double best_response,
        const Pose2 &search_center,
        double search_angle_offset,
        double search_angle_resolution,
        Matrix3d &covariance)
{
        // NOTE: do not reset covariance matrix

        // normalize angle difference
        double best_angle = math::NormalizeAngleDifference(
                best_pose.getHeading(), search_center.getHeading());

        Vector2i grid_point = correlation_grid_->convertWorldToGrid(best_pose.getPosition());
        int32_t grid_index = correlation_grid_->getGridIndex(grid_point);

        uint32_t n_angles =
            static_cast<uint32_t>(std::round(search_angle_offset * 2 / search_angle_resolution) + 1);

        double angle = 0.0;
        double startAngle = search_center.getHeading() - search_angle_offset;

        double norm = 0.0;
        double accumulated_variance_thth = 0.0;
        for (uint32_t angle_index = 0; angle_index < n_angles; angle_index++)
        {
                angle = startAngle + angle_index * search_angle_resolution;
                double response = getResponse(angle_index, grid_index);

                // response is not a low response
                if (response >= (best_response - 0.1))
                {
                        norm += response;
                        accumulated_variance_thth += (math::Square(angle - best_angle) * response);
                }
        }
        assert(math::DoubleEqual(angle, search_center.getHeading() + search_angle_offset));

        if (norm > math::TOLERANCE) {
                if (accumulated_variance_thth < math::TOLERANCE)
                {
                        accumulated_variance_thth = math::Square(search_angle_resolution);
                }

                accumulated_variance_thth /= norm;
        } else {
                accumulated_variance_thth = 1000 * math::Square(search_angle_resolution);
        }

        covariance(2, 2) = accumulated_variance_thth;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MapperGraph::MapperGraph(Mapper *mapper, double range_threshold)
        : mapper_(mapper)
{
        loop_scan_matcher_ = ScanMatcher::create(
                mapper,
                mapper_->loop_search_space_dimension_,
                mapper_->loop_search_space_resolution_,
                mapper_->loop_search_space_smear_deviation_, 
                range_threshold);
        assert(loop_scan_matcher_);

        traversal_ = std::make_unique<BreadthFirstTraversal<LocalizedRangeScan>>(this);
}

Vertex<LocalizedRangeScan> *MapperGraph::addVertex(LocalizedRangeScan * scan)
{
        assert(scan);

        if (scan != nullptr) {
                std::unique_ptr<Vertex<LocalizedRangeScan>> vertex = std::make_unique<Vertex<LocalizedRangeScan>>(scan);
                Vertex<LocalizedRangeScan> *vertex_ptr = vertex.get();
                Graph<LocalizedRangeScan>::addVertex(std::move(vertex));
                if (mapper_->scan_optimizer_ != nullptr) {
                        mapper_->scan_optimizer_->addNode(vertex_ptr);
                }
                return vertex_ptr;
        }

        return nullptr;
}

Edge<LocalizedRangeScan> *MapperGraph::addEdge(
        LocalizedRangeScan *source_scan,
        LocalizedRangeScan *target_scan,
        bool &is_new_edge)
{
        std::map<int, 
                std::unique_ptr<Vertex<LocalizedRangeScan>>>::iterator v1 = vertices_.find(source_scan->getScanId());
        std::map<int, 
                std::unique_ptr<Vertex<LocalizedRangeScan>>>::iterator v2 = vertices_.find(target_scan->getScanId());

        if (v1 == vertices_.end() || v2 == vertices_.end()) {
                std::cout << "addEdge: At least one vertex is invalid." << std::endl;
                return nullptr;
        }

        // see if edge already exists
        for (const auto& edge : v1->second->getEdges()) {
                if (edge->getTarget() == v2->second.get()) {
                        is_new_edge= false;
                        return edge;
                }
        }

        std::unique_ptr<Edge<LocalizedRangeScan>> edge = std::make_unique<Edge<LocalizedRangeScan>>(v1->second.get(), v2->second.get());
        Edge<LocalizedRangeScan> *edge_ptr = edge.get();
        Graph<LocalizedRangeScan>::addEdge(std::move(edge));
        is_new_edge = true;
        return edge_ptr;
}

void MapperGraph::addEdges(LocalizedRangeScan * scan, const Matrix3d &covariance)
{
        ScanManager *scan_manager = mapper_->scan_manager_.get();

        // link to previous scan
        LocalizedRangeScan *prev_scan = scan_manager->getLastScan();
        if (!prev_scan) {
                return;
        }
        linkScans(prev_scan, scan, scan->getSensorPose(), covariance);

        Pose2Vector means;
        std::vector<Matrix3d> covariances;

        // link to running scans
        Pose2 scan_pose = scan->getSensorPose();
        means.push_back(scan_pose);
        covariances.push_back(covariance);
        linkChainToScan(scan_manager->getRunningScans(), scan, scan_pose, covariance);

        // link to other near chains (chains that include new scan are invalid)
        linkNearChains(scan, means, covariances);

        if (!means.empty()) {
                scan->setSensorPose(computeWeightedMean(means, covariances));
        }
}

bool MapperGraph::tryCloseLoop(LocalizedRangeScan * scan)
{
        bool loopClosed = false;

        uint32_t scanIndex = 0;

        LocalizedRangeScanVector candidateChain = FindPossibleLoopClosure(scan, scanIndex);

        while (!candidateChain.empty())
        {
                Pose2 bestPose;
                Matrix3d covariance;
                double coarseResponse = loop_scan_matcher_->matchScan(scan, candidateChain,
                                                                         bestPose, covariance, false, false);

                std::stringstream stream;
                stream << "COARSE RESPONSE: " << coarseResponse << " (> " << mapper_->loop_match_minimum_response_coarse_ << ")" << std::endl;
                stream << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) << " (< " << mapper_->loop_match_maximum_variance_coarse_ << ")";

                if ((coarseResponse > mapper_->loop_match_minimum_response_coarse_) &&
                    (covariance(0, 0) < mapper_->loop_match_maximum_variance_coarse_) &&
                    (covariance(1, 1) < mapper_->loop_match_maximum_variance_coarse_))
                {
                        LocalizedRangeScan tmpScan(scan->getLaserRangeFinder(), scan->getRangeReadingsVector());
                        tmpScan.setScanId(scan->getScanId());
                        tmpScan.setTime(scan->getTime());
                        tmpScan.setCorrectedPose(scan->getCorrectedPose());
                        tmpScan.setSensorPose(bestPose); // This also updates OdometricPose.
                        double fineResponse = mapper_->scan_matcher_->matchScan(&tmpScan,
                                                                                                candidateChain,
                                                                                                bestPose, covariance, false);

                        std::stringstream stream1;
                        stream1 << "FINE RESPONSE: " << fineResponse << " (>" << mapper_->loop_match_minimum_response_fine_ << ")" << std::endl;

                        if (fineResponse < mapper_->loop_match_minimum_response_fine_)
                        {
                                // mapper_->FireLoopClosureCheck("REJECTED!");
                        }
                        else
                        {
                                // mapper_->FireBeginLoopClosure("Closing loop...");

                                scan->setSensorPose(bestPose);
                                linkChainToScan(candidateChain, scan, bestPose, covariance);
                                correctPoses();

                                // mapper_->FireEndLoopClosure("Loop closed!");

                                loopClosed = true;
                        }
                }

                candidateChain = FindPossibleLoopClosure(scan, scanIndex);
        }

        return loopClosed;
}

void MapperGraph::correctPoses()
{
        // optimize scans!
        ScanSolver *solver = mapper_->scan_optimizer_.get();
        if (solver != nullptr) {
                solver->compute();

                for (const auto &pose_vec : solver->getCorrections()) {
                        LocalizedRangeScan *scan = mapper_->scan_manager_->getScan(pose_vec.first);
                        if (scan == nullptr) {
                                continue;
                        }
                        scan->setCorrectedPoseAndUpdate(pose_vec.second);
                }

                solver->clear();
        }
}

LocalizedRangeScanVector MapperGraph::FindPossibleLoopClosure(
        LocalizedRangeScan *scan,
        uint32_t &rStartNum)
{
        LocalizedRangeScanVector chain; // return value

        Pose2 pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

        // possible loop closure chain should not include close scans that have a
        // path of links to the scan of interest
        const LocalizedRangeScanVector nearLinkedScans =
                findNearLinkedScans(scan, mapper_->loop_search_maximum_distance_);

        uint32_t nScans =
            static_cast<uint32_t>(mapper_->scan_manager_->getAllScans().size());
        for (; rStartNum < nScans; rStartNum++) {
                LocalizedRangeScan *pCandidateScan = mapper_->scan_manager_->getScan(rStartNum);

                if (pCandidateScan == nullptr) {
                        continue;
                }

                Pose2 candidateScanPose = pCandidateScan->getReferencePose(
                    mapper_->use_scan_barycenter_);

                double squaredDistance = candidateScanPose.getSquaredDistance(pose);
                if (squaredDistance <
                    math::Square(mapper_->loop_search_maximum_distance_) + math::TOLERANCE) {
                        // a linked scan cannot be in the chain
                        if (find(nearLinkedScans.begin(), nearLinkedScans.end(),
                                 pCandidateScan) != nearLinkedScans.end())
                        {
                                chain.clear();
                        }
                        else
                        {
                                chain.push_back(pCandidateScan);
                        }
                }
                else
                {
                        // return chain if it is long "enough"
                        if (chain.size() >= mapper_->loop_match_minimum_chain_size_)
                        {
                                return chain;
                        }
                        else
                        {
                                chain.clear();
                        }
                }
        }

        return chain;
}

void MapperGraph::linkScans(
        LocalizedRangeScan *from_scan,
        LocalizedRangeScan *to_scan,
        const Pose2 &mean,
        const Matrix3d &covariance)
{
        bool is_new_edge = true;
        Edge<LocalizedRangeScan> *edge = addEdge(from_scan, to_scan, is_new_edge);

        if (edge == nullptr) {
                return;
        }

        // only attach link information if the edge is new
        if (is_new_edge == true) {
                edge->setLabel(new LinkInfo(from_scan->getCorrectedPose(), to_scan->getCorrectedAt(mean), covariance));
                if (mapper_->scan_optimizer_ != nullptr) {
                        mapper_->scan_optimizer_->addConstraint(edge);
                }
        }
}

LocalizedRangeScanVector MapperGraph::findNearLinkedScans(LocalizedRangeScan *scan, double max_distance)
{
        NearScanVisitor *visitor = new NearScanVisitor(
                scan, 
                max_distance,
                mapper_->use_scan_barycenter_);
        LocalizedRangeScanVector nearLinkedScans = traversal_->traverseForScans(
                getVertex(scan),
                visitor);
        delete visitor;

        return nearLinkedScans;
}

void MapperGraph::linkChainToScan(
        const LocalizedRangeScanVector &chain,
        LocalizedRangeScan *scan,
        const Pose2 &mean,
        const Matrix3d &covariance)
{
        Pose2 pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

        LocalizedRangeScan *closest_scan = getClosestScanToPose(chain, pose);
        assert(closest_scan != nullptr);

        Pose2 closest_scan_pose =
                closest_scan->getReferencePose(mapper_->use_scan_barycenter_);

        double squared_distance = pose.getSquaredDistance(closest_scan_pose);
        if (squared_distance <
                math::Square(mapper_->link_scan_maximum_distance_) + math::TOLERANCE)
        {
                linkScans(closest_scan, scan, mean, covariance);
        }
}

void MapperGraph::linkNearChains(
        LocalizedRangeScan *scan, Pose2Vector &means,
        std::vector<Matrix3d> &covariances)
{
        const std::vector<LocalizedRangeScanVector> near_chains = findNearChains(scan);
        for (const auto& scan_chain : near_chains) {
                if (scan_chain.size() < mapper_->loop_match_minimum_chain_size_) {
                        continue;
                }

                Pose2 mean;
                Matrix3d covariance;
                // match scan against "near" chain
                double response = mapper_->scan_matcher_->matchScan(
                        scan, 
                        scan_chain, 
                        mean,
                        covariance, 
                        false);
                if (response > mapper_->link_match_minimum_response_fine_ - math::TOLERANCE) {
                        means.push_back(mean);
                        covariances.push_back(covariance);
                        linkChainToScan(scan_chain, scan, mean, covariance);
                }
        }
}

std::vector<LocalizedRangeScanVector> MapperGraph::findNearChains(LocalizedRangeScan *scan)
{
        std::vector<LocalizedRangeScanVector> near_chains;

        Pose2 scan_pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

        // to keep track of which scans have been added to a chain
        LocalizedRangeScanVector processed;

        const LocalizedRangeScanVector near_linked_scans = findNearLinkedScans(
                scan,
                mapper_->link_scan_maximum_distance_);
        
        for (const auto& near_scan : near_linked_scans) {
                if (near_scan == scan) {
                        continue;
                }

                // scan has already been processed, skip
                if (find(processed.begin(), processed.end(), near_scan) != processed.end()) {
                        continue;
                }

                processed.push_back(near_scan);

                // build up chain
                bool is_valid_chain = true;
                std::list<LocalizedRangeScan *> chain;

                // add scans before current scan being processed
                for (int32_t candidateScanNum = near_scan->getScanId() - 1; candidateScanNum >= 0;
                     candidateScanNum--)
                {
                        LocalizedRangeScan *pCandidateScan = mapper_->scan_manager_->getScan(candidateScanNum);

                        // chain is invalid--contains scan being added
                        if (pCandidateScan == scan)
                        {
                                is_valid_chain = false;
                        }

                        // probably removed in localization mode
                        if (pCandidateScan == nullptr)
                        {
                                continue;
                        }

                        Pose2 candidatePose = pCandidateScan->getReferencePose(mapper_->use_scan_barycenter_);
                        double squaredDistance = scan_pose.getSquaredDistance(candidatePose);

                        if (squaredDistance <
                            math::Square(mapper_->link_scan_maximum_distance_) + math::TOLERANCE)
                        {
                                chain.push_front(pCandidateScan);
                                processed.push_back(pCandidateScan);
                        }
                        else
                        {
                                break;
                        }
                }

                chain.push_back(near_scan);

                // add scans after current scan being processed
                uint32_t end =
                        static_cast<uint32_t>(mapper_->scan_manager_->getAllScans().size());
                for (uint32_t candidateScanNum = near_scan->getScanId() + 1; candidateScanNum < end;
                     candidateScanNum++)
                {
                        LocalizedRangeScan *pCandidateScan = mapper_->scan_manager_->getScan(candidateScanNum);

                        if (pCandidateScan == scan)
                        {
                                is_valid_chain = false;
                        }

                        // probably removed in localization mode
                        if (pCandidateScan == nullptr)
                        {
                                continue;
                        }

                        Pose2 candidatePose = pCandidateScan->getReferencePose(mapper_->use_scan_barycenter_);
                        double squaredDistance =
                            scan_pose.getSquaredDistance(candidatePose);

                        if (squaredDistance <
                            math::Square(mapper_->link_scan_maximum_distance_) + math::TOLERANCE)
                        {
                                chain.push_back(pCandidateScan);
                                processed.push_back(pCandidateScan);
                        }
                        else
                        {
                                break;
                        }
                }

                if (is_valid_chain)
                {
                        // change list to vector
                        LocalizedRangeScanVector tempChain;
                        std::copy(chain.begin(), chain.end(), std::inserter(tempChain, tempChain.begin()));
                        // add chain to collection
                        near_chains.push_back(tempChain);
                }
        }

        return near_chains;
}

Pose2 MapperGraph::computeWeightedMean(
        const Pose2Vector &means,
        const std::vector<Matrix3d> &covariances) const
{
        assert(means.size() == covariances.size());

        // compute sum of inverses and create inverse list
        std::vector<Matrix3d> inverses;
        inverses.reserve(covariances.size());

        Matrix3d sumOfInverses;
        for (const auto& cov : covariances) {
                Matrix3d inverse = cov.inverse();
                inverses.push_back(inverse);

                sumOfInverses += inverse;
        }
        Matrix3d inverseOfSumOfInverses = sumOfInverses.inverse();

        // compute weighted mean
        Pose2 accumulatedPose;
        double thetaX = 0.0;
        double thetaY = 0.0;

        Pose2Vector::const_iterator meansIter = means.begin();
        for (const auto& inverse_iter : inverses) {
                Pose2 pose = *meansIter;
                double angle = pose.getHeading();
                thetaX += cos(angle);
                thetaY += sin(angle);

                Matrix3d weight = inverseOfSumOfInverses * (inverse_iter);
                accumulatedPose += pose.multiplyLeftMatrix(weight);

                ++meansIter;
        }

        thetaX /= means.size();
        thetaY /= means.size();
        accumulatedPose.setHeading(atan2(thetaY, thetaX));

        return accumulatedPose;
}

LocalizedRangeScan *MapperGraph::getClosestScanToPose(
        const LocalizedRangeScanVector &scans,
        const Pose2 &pose) const
{
        LocalizedRangeScan *closest_scan = nullptr;
        double best_squared_distance = DBL_MAX;

        for (const auto& scan : scans) {
                Pose2 scan_pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

                double squared_distance = pose.getSquaredDistance(scan_pose);
                if (squared_distance < best_squared_distance) {
                        best_squared_distance = squared_distance;
                        closest_scan = scan;
                }
        }

        return closest_scan;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

bool Mapper::process(LocalizedRangeScan * scan, Eigen::Matrix3d * covariance)
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
                        Pose2 T_map_odom = Pose2::getRelativePose(
                                last_scan->getCorrectedPose(), 
                                last_scan->getOdometricPose());
                        scan->setCorrectedPose(
                                Pose2::transformPose(
                                        T_map_odom,
                                        scan->getOdometricPose()));
                }

                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();

                // correct scan (if not first scan)
                if (last_scan != nullptr) {
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

                graph_->tryCloseLoop(scan);

                scan_manager_->setLastScan(scan);

                return true;
        }
        
        return false;
}

} // namespace mapper_utils