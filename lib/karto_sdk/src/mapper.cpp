#include "karto_sdk/mapper.hpp"

namespace karto
{
// enable for verbose debug
// #define MYSLAM_DEBUG
#define MAX_VARIANCE 500.0
#define DISTANCE_PENALTY_GAIN 0.2
#define ANGLE_PENALTY_GAIN 0.2


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/**
 * Create a scan matcher with the given parameters
 */
std::unique_ptr<ScanMatcher> ScanMatcher::create(
        Mapper *mapper,
        double search_size,
        double resolution,
        double smear_deviation,
        double range_threshold)
/*****************************************************************************/
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

        assert(math::DoubleEqual(math::Round(search_size / resolution), (search_size / resolution)));

        // calculate search space in grid coordinates
        uint32_t search_space_side_size = static_cast<uint32_t>(math::Round(search_size / resolution) + 1);

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
        scan_matcher->search_space_probs_.reset(Grid<double>::createGrid(
                search_space_side_size,
                search_space_side_size,
                resolution)); // take ownership
        scan_matcher->grid_lookup_ = std::make_unique<GridIndexLookup<uint8_t>>(scan_matcher->correlation_grid_.get());

        return scan_matcher;
}

/*****************************************************************************/
template<class T>
double ScanMatcher::matchScan(
        LocalizedRangeScan *scan,
        const T &base_scans,
        Pose2 &mean, 
        Matrix3 &covariance,
        bool do_penalize,
        bool do_refine_match)
/*****************************************************************************/
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
        Vector2<double> offset;
        offset.setX(scan_pose.getX() - 0.5 * (roi.getWidth() - 1) * correlation_grid_->getResolution());
        offset.setY(scan_pose.getY() - 0.5 * (roi.getHeight() - 1) * correlation_grid_->getResolution());

        // 4. set offset
        correlation_grid_->getCoordinateConverter()->setOffset(offset);

        ///////////////////////////////////////
        // set up correlation grid
        addScans(base_scans, scan_pose.getPosition());

        // compute how far to search in each direction
        Vector2<double> search_dimensions(
                search_space_probs_->getWidth(), 
                search_space_probs_->getHeight());
        Vector2<double> coarse_search_offset(
                0.5 * (search_dimensions.getX() - 1) * correlation_grid_->getResolution(),
                0.5 * (search_dimensions.getY() - 1) * correlation_grid_->getResolution());

        // a coarse search only checks half the cells in each dimension
        Vector2<double> coarse_search_resolution(
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
        
        if (mapper_->use_response_expansion_ == true) {
                if (math::DoubleEqual(best_response, 0.0)) {
                        #ifdef MYSLAM_DEBUG
                        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
                        #endif
                        // try and increase search angle offset with 20 degrees and do another match
                        double new_search_angle_offset = mapper_->coarse_search_angle_offset_;
                        for (uint32_t i = 0; i < 3; i++) {
                                new_search_angle_offset += math::DegreesToRadians(20);
                                std::cout << "about to enter expansion correlateScan " << i << std::endl;
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
                Vector2<double> fine_search_offset(coarse_search_resolution * 0.5);
                Vector2<double> fine_search_resolution(
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

        assert(math::InRange(mean.getHeading(), -KT_PI, KT_PI));

        return best_response;
}

/*****************************************************************************/
/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::addScans(
        const LocalizedRangeScanVector &scans, 
        Vector2<double> viewpoint)
/*****************************************************************************/
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

/*****************************************************************************/
/**
 * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
 * @param pScan scan whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 * @param doSmear whether the points will be smeared
 */
void ScanMatcher::addScan(
        LocalizedRangeScan *scan, 
        const Vector2<double> &viewpoint,
        bool do_smear)
/*****************************************************************************/
{
        PointVectorDouble valid_points = findValidPoints(scan, viewpoint);

        // put in all valid points
        for (const auto& point : valid_points) {
                Vector2<int32_t> grid_point = correlation_grid_->convertWorldToGrid(point);
                if (!math::IsUpTo(grid_point.getX(), correlation_grid_->getROI().getWidth()) ||
                    !math::IsUpTo(grid_point.getY(), correlation_grid_->getROI().getHeight())) {
                        // point not in grid
                        continue;
                }

                int grid_index = correlation_grid_->getGridIndex(grid_point);

                // set grid cell as occupied
                if (correlation_grid_->getDataPointer()[grid_index] 
                        == GRIDSTATES_OCCUPIED) {
                                // value already set
                                continue;
                }

                correlation_grid_->getDataPointer()[grid_index] = GRIDSTATES_OCCUPIED;

                // smear_grid
                if (do_smear == true) {
                        correlation_grid_->smearPoint(grid_point);
                }
        }
}

/*****************************************************************************/
/**
 * Compute which points in a scan are on the same side as the given viewpoint
 * @param scan
 * @param rViewPoint
 * @return points on the same side
 */
PointVectorDouble ScanMatcher::findValidPoints(
        LocalizedRangeScan *scan,
        const Vector2<double> &viewpoint) const
/*****************************************************************************/
{
        const PointVectorDouble &point_readings = scan->getPointReadings();

        // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
        const double min_square_distance = math::Square(0.1); // in m^2

        // this iterator lags from the main iterator adding points only when the points are on
        // the same side as the viewpoint
        PointVectorDouble::const_iterator trailing_point_iter = point_readings.begin();
        PointVectorDouble valid_points;

        Vector2<double> first_point;
        bool first_time = true;
        for (PointVectorDouble::const_iterator iter = point_readings.begin(); iter != point_readings.end(); ++iter) {
                Vector2<double> current_point = *iter;

                if (first_time && !std::isnan(current_point.getX()) && !std::isnan(current_point.getY())) {
                        first_point = current_point;
                        first_time = false;
                }

                Vector2<double> delta = first_point - current_point;
                if (delta.computeSquaredLength() > min_square_distance) {
                        // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
                        // Which computes the direction of rotation, if the rotation is counterclock
                        // wise then we are looking at data we should keep. If it's negative rotation
                        // we should not included in in the matching
                        // have enough distance, check viewpoint
                        double a = viewpoint.getY() - first_point.getY();
                        double b = first_point.getX() - viewpoint.getX();
                        double c = first_point.getY() * viewpoint.getX() - first_point.getX() * viewpoint.getY();
                        double ss = current_point.getX() * a + current_point.getY() * b + c;

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

/*****************************************************************************/
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
        const Vector2<double> &search_space_offset,
        const Vector2<double> &search_space_resolution,
        double search_angle_offset,
        double search_angle_resolution,
        bool do_penalize,
        Pose2 &mean,
        Matrix3 &covariance,
        bool doing_fine_match)
/*****************************************************************************/
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
                Vector2<double> offset(search_center.getPosition() - search_space_offset);
                search_space_probs_->getCoordinateConverter()->setOffset(offset);
        }

        // calculate position arrays
        x_poses_.clear();
        uint32_t n_x = static_cast<uint32_t>(
                math::Round(search_space_offset.getX() * 2.0 / search_space_resolution.getX()) + 1);
        double start_x = -search_space_offset.getX();
        for (uint32_t x_index = 0; x_index < n_x; x_index++) {
                x_poses_.push_back(start_x + x_index * search_space_resolution.getX());
        }
        assert(math::DoubleEqual(x_poses_.back(), -start_x));

        y_poses_.clear();
        uint32_t n_y = static_cast<uint32_t>(
            math::Round(search_space_offset.getY() * 2.0 / search_space_resolution.getY()) + 1);
        double start_y = -search_space_offset.getY();
        for (uint32_t y_index = 0; y_index < n_y; y_index++) {
                y_poses_.push_back(start_y + y_index * search_space_resolution.getY());
        }
        assert(math::DoubleEqual(y_poses_.back(), -start_y));

        // calculate pose response array size
        uint32_t n_angles = 
                static_cast<uint32_t>(math::Round(search_angle_offset * 2.0 / search_angle_resolution) + 1);

        uint32_t pose_response_size = static_cast<uint32_t>(x_poses_.size() * y_poses_.size() * n_angles);

        // allocate array
        pose_response_ = std::make_unique<std::pair<double, Pose2>[]>(pose_response_size);

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
                        Vector2<int32_t> grid = search_space_probs_->convertWorldToGrid(pose.getPosition());
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
        Vector2<double> average_position;
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
        assert(math::InRange(mean.getHeading(), -KT_PI, KT_PI));

        return best_response;
}

/*****************************************************************************/
void ScanMatcher::operator()(const double &y) const
/*****************************************************************************/
{
        uint32_t pose_response_counter;
        uint32_t x_pose;
        uint32_t y_pose = std::find(y_poses_.begin(), y_poses_.end(), y) - y_poses_.begin();

        const uint32_t size_x = x_poses_.size();

        double new_position_y = search_center_.getY() + y;
        double square_y = math::Square(y);

        for (std::vector<double>::const_iterator x_iter = x_poses_.begin(); x_iter != x_poses_.end();
             ++x_iter) {
                x_pose = std::distance(x_poses_.begin(), x_iter);
                double x = *x_iter;
                double new_position_x = search_center_.getX() + x;
                double square_x = math::Square(x);

                Vector2<int32_t> gridPoint =
                        correlation_grid_->convertWorldToGrid(Vector2<double>(new_position_x, new_position_y));
                int32_t grid_index = correlation_grid_->getGridIndex(gridPoint);
                assert(grid_index >= 0);

                double angle = 0.0;
                double start_angle = search_center_.getHeading() - search_angle_offset_;
                for (uint32_t angle_index = 0; angle_index < n_angles_; angle_index++) {
                        angle = start_angle + angle_index * search_angle_resolution_;

                        double response = getResponse(angle_index, grid_index);
                        if (do_penalize_ && (math::DoubleEqual(response, 0.0) == false)) {
                                // simple model (approximate Gaussian) to take odometry into account
                                double squared_distance = square_x + square_y;
                                double distance_penalty = 1.0 - (DISTANCE_PENALTY_GAIN *
                                                                squared_distance / mapper_->distance_variance_penalty_);
                                distance_penalty = std::max(distance_penalty,
                                                           mapper_->minimum_distance_penalty_);

                                double squared_angle_distance = math::Square(angle - search_center_.getHeading());
                                double angle_penalty = 1.0 - (ANGLE_PENALTY_GAIN *
                                                             squared_angle_distance / mapper_->angle_variance_penalty_);
                                angle_penalty = std::max(angle_penalty, mapper_->minimum_angle_penalty_);

                                response *= (distance_penalty * angle_penalty);
                        }

                        // store response and pose
                        pose_response_counter = (y_pose * size_x + x_pose) * (n_angles_) + angle_index;
                        pose_response_[pose_response_counter] =
                            std::pair<double, Pose2>(response, Pose2(new_position_x, new_position_y,
                                                                     math::NormalizeAngle(angle)));
                }
        }
}

/*****************************************************************************/
double ScanMatcher::getResponse(uint32_t angle_index, int32_t grid_position_index) const
/*****************************************************************************/
{
        double response = 0.0;

        // add up value for each point
        uint8_t *byte = correlation_grid_->getDataPointer() + grid_position_index;

        const LookupArray *offsets = grid_lookup_->getLookupArray(angle_index);
        assert(offsets != NULL);

        // get number of points in offset list
        uint32_t points = offsets->getSize();
        if (points == 0) {
                return response;
        }

        // calculate response
        int32_t *angle_index_pointer = offsets->getArrayPointer();
        for (uint32_t i = 0; i < points; i++)
        {
                // ignore points that fall off the grid
                int32_t point_grid_index = grid_position_index + angle_index_pointer[i];
                if (!math::IsUpTo(point_grid_index, correlation_grid_->getDataSize()) ||
                        angle_index_pointer[i] == INVALID_SCAN) {
                        continue;
                }

                // uses index offsets to efficiently find location of point in the grid
                response += byte[angle_index_pointer[i]];
        }

        // normalize response
        response /= (points * GRIDSTATES_OCCUPIED);
        assert(fabs(response) <= 1.0);

        return response;
}

/*****************************************************************************/
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
        const Vector2<double> &search_space_offset,
        const Vector2<double> &search_space_resolution,
        double search_angle_resolution,
        Matrix3 &covariance)
/*****************************************************************************/
{
        // reset covariance to identity matrix
        covariance.setToIdentity();

        // if best response is vary small return max variance
        if (best_response < KT_TOLERANCE)
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

        double offset_x = search_space_offset.getX();
        double offset_y = search_space_offset.getY();

        uint32_t n_x =
                static_cast<uint32_t>(std::round(offset_x * 2.0 / search_space_resolution.getX()) + 1);
        double start_x = -offset_x;
        assert(math::DoubleEqual(start_x + (n_x - 1) * search_space_resolution.getX(), -start_x));

        uint32_t n_y =
            static_cast<uint32_t>(std::round(offset_y * 2.0 / search_space_resolution.getY()) + 1);
        double start_y = -offset_y;
        assert(math::DoubleEqual(start_y + (n_y - 1) * search_space_resolution.getY(), -start_y));

        for (uint32_t y_index = 0; y_index < n_y; y_index++) {
                double y = start_y + y_index * search_space_resolution.getY();

                for (uint32_t x_index = 0; x_index < n_x; x_index++) {
                        double x = start_x + x_index * search_space_resolution.getX();

                        Vector2<int32_t> grid_point =
                                search_space_probs_->convertWorldToGrid(Vector2<double>(search_center.getX() + x,
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

        if (norm > KT_TOLERANCE) {
                double variance_xx = accumulated_variance_xx / norm;
                double variance_xy = accumulated_variance_xy / norm;
                double variance_yy = accumulated_variance_yy / norm;
                double variance_thth = 4 * math::Square(search_angle_resolution);

                // lower-bound variances so that they are not too small;
                // ensures that links are not too tight
                double min_variance_xx = 0.1 * math::Square(search_space_resolution.getX());
                double min_variance_yy = 0.1 * math::Square(search_space_resolution.getY());
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

/*****************************************************************************/
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
        Matrix3 &covariance)
/*****************************************************************************/
{
        // NOTE: do not reset covariance matrix

        // normalize angle difference
        double best_angle = math::NormalizeAngleDifference(
                best_pose.getHeading(), search_center.getHeading());

        Vector2<int32_t> grid_point = correlation_grid_->convertWorldToGrid(best_pose.getPosition());
        int32_t grid_index = correlation_grid_->getGridIndex(grid_point);

        uint32_t n_angles =
            static_cast<uint32_t>(std::round(search_angle_offset * 2 / search_angle_resolution) + 1);

        double angle = 0.0;
        double start_anle = search_center.getHeading() - search_angle_offset;

        double norm = 0.0;
        double accumulated_variance_thth = 0.0;
        for (uint32_t angle_index = 0; angle_index < n_angles; angle_index++)
        {
                angle = start_anle + angle_index * search_angle_resolution;
                double response = getResponse(angle_index, grid_index);

                // response is not a low response
                if (response >= (best_response - 0.1))
                {
                        norm += response;
                        accumulated_variance_thth += (math::Square(angle - best_angle) * response);
                }
        }
        assert(math::DoubleEqual(angle, search_center.getHeading() + search_angle_offset));

        if (norm > KT_TOLERANCE) {
                if (accumulated_variance_thth < KT_TOLERANCE)
                {
                        accumulated_variance_thth = math::Square(search_angle_resolution);
                }

                accumulated_variance_thth /= norm;
        } else {
                accumulated_variance_thth = 1000 * math::Square(search_angle_resolution);
        }

        covariance(2, 2) = accumulated_variance_thth;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
MapperGraph::MapperGraph(Mapper *mapper, double range_threshold)
/*****************************************************************************/
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

/*****************************************************************************/
Vertex<LocalizedRangeScan> *MapperGraph::addVertex(LocalizedRangeScan *scan)
/*****************************************************************************/
{
        assert(scan);

        if (scan != nullptr) {
                std::unique_ptr<Vertex<LocalizedRangeScan>> vertex = 
                        std::make_unique<Vertex<LocalizedRangeScan>>(scan);
                Vertex<LocalizedRangeScan> *vertex_ptr = vertex.get();
                Graph<LocalizedRangeScan>::addVertex(std::move(vertex));
                if (mapper_->scan_optimizer_ != nullptr) {
                        mapper_->scan_optimizer_->addNode(vertex_ptr);
                }
                return vertex_ptr;
        }

        return nullptr;
}

/*****************************************************************************/
Edge<LocalizedRangeScan> *MapperGraph::addEdge(
        LocalizedRangeScan *source_scan,
        LocalizedRangeScan *target_scan,
        bool &is_new_edge)
/*****************************************************************************/
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

/*****************************************************************************/
void MapperGraph::addEdges(LocalizedRangeScan *scan, const Matrix3 &covariance)
/*****************************************************************************/
{
        ScanManager *scan_manager = mapper_->scan_manager_.get();

        // link to previous scan
        LocalizedRangeScan *prev_scan = scan_manager->getLastScan();
        if (!prev_scan) {
                return;
        }
        linkScans(prev_scan, scan, scan->getSensorPose(), covariance);

        Pose2Vector means;
        std::vector<Matrix3> covariances;

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

/*****************************************************************************/
bool MapperGraph::tryCloseLoop(LocalizedRangeScan *scan)
/*****************************************************************************/
{
        bool loopClosed = false;

        uint32_t scanIndex = 0;

        LocalizedRangeScanVector candidate_chain = findPossibleLoopClosure(scan, scanIndex);

        while (!candidate_chain.empty()) {
                Pose2 bestPose;
                Matrix3 covariance;
                double coarseResponse = loop_scan_matcher_->matchScan(
                        scan, 
                        candidate_chain,                                                 
                        bestPose, 
                        covariance, 
                        false, 
                        false);

                if ((coarseResponse > mapper_->loop_match_minimum_response_coarse_) &&
                    (covariance(0, 0) < mapper_->loop_match_maximum_variance_coarse_) &&
                    (covariance(1, 1) < mapper_->loop_match_maximum_variance_coarse_)) {
                        LocalizedRangeScan tmpScan(scan->getLaserRangeFinder(), scan->getRangeReadingsVector());
                        tmpScan.setScanId(scan->getScanId());
                        tmpScan.setTime(scan->getTime());
                        tmpScan.setCorrectedPose(scan->getCorrectedPose());
                        tmpScan.setSensorPose(bestPose); // This also updates OdometricPose.
                        double fineResponse = mapper_->sequential_scan_matcher_->matchScan(
                                &tmpScan,
                                candidate_chain,
                                bestPose, 
                                covariance, 
                                false);

                        if (fineResponse < mapper_->loop_match_minimum_response_fine_) {
                                // mapper_->FireLoopClosureCheck("REJECTED!");
                        } else {
                                // mapper_->FireBeginLoopClosure("Closing loop...");
                                scan->setSensorPose(bestPose);
                                linkChainToScan(candidate_chain, scan, bestPose, covariance);
                                correctPoses();
                                // mapper_->FireEndLoopClosure("Loop closed!");
                                loopClosed = true;
                        }
                }

                candidate_chain = findPossibleLoopClosure(scan, scanIndex);
        }

        return loopClosed;
}

/*****************************************************************************/
void MapperGraph::correctPoses()
/*****************************************************************************/
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

/*****************************************************************************/
LocalizedRangeScanVector MapperGraph::findPossibleLoopClosure(
        LocalizedRangeScan *scan,
        uint32_t &start_num)
/*****************************************************************************/
{
        LocalizedRangeScanVector chain; // return value

        Pose2 pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

        // possible loop closure chain should not include close scans that have a
        // path of links to the scan of interest
        const LocalizedRangeScanVector near_linked_scans =
                findNearLinkedScans(scan, mapper_->loop_search_maximum_distance_);

        uint32_t n_scans =
                static_cast<uint32_t>(mapper_->scan_manager_->getAllScans().size());
        for (; start_num < n_scans; start_num++) {
                LocalizedRangeScan *candidate_scan = mapper_->scan_manager_->getScan(start_num);
                if (candidate_scan == nullptr) {
                        continue;
                }

                Pose2 candidate_scan_pose = candidate_scan->getReferencePose(
                    mapper_->use_scan_barycenter_);

                double squared_distance = candidate_scan_pose.computeSquaredDistance(pose);
                if (squared_distance <
                    math::Square(mapper_->loop_search_maximum_distance_) + KT_TOLERANCE) {
                        // a linked scan cannot be in the chain
                        if (std::find(near_linked_scans.begin(), 
                            near_linked_scans.end(),
                            candidate_scan) != near_linked_scans.end()) {
                                chain.clear();
                        } else {
                                chain.push_back(candidate_scan);
                        }
                } else {
                        // return chain if it is long "enough"
                        if (chain.size() >= mapper_->loop_match_minimum_chain_size_) {
                                return chain;
                        } else {
                                chain.clear();
                        }
                }
        }

        return chain;
}

/*****************************************************************************/
void MapperGraph::linkScans(
        LocalizedRangeScan *from_scan,
        LocalizedRangeScan *to_scan,
        const Pose2 &mean,
        const Matrix3 &covariance)
/*****************************************************************************/
{
        bool is_new_edge = true;
        Edge<LocalizedRangeScan> *edge = addEdge(from_scan, to_scan, is_new_edge);

        if (edge == nullptr) {
                return;
        }

        // only attach link information if the edge is new
        if (is_new_edge == true) {
                edge->setLabel(std::make_unique<LinkInfo>(from_scan->getCorrectedPose(), to_scan->getCorrectedAt(mean), covariance));
                if (mapper_->scan_optimizer_ != nullptr) {
                        mapper_->scan_optimizer_->addConstraint(edge);
                }
        }
}

/*****************************************************************************/
LocalizedRangeScanVector MapperGraph::findNearLinkedScans(LocalizedRangeScan *scan, double max_distance)
/*****************************************************************************/
{
        std::unique_ptr<NearScanVisitor> visitor = std::make_unique<NearScanVisitor>(
                scan, 
                max_distance,
                mapper_->use_scan_barycenter_
        );
        LocalizedRangeScanVector nearLinkedScans = traversal_->traverseForScans(
                getVertex(scan),
                visitor.get());
        visitor.reset();

        return nearLinkedScans;
}

/*****************************************************************************/
void MapperGraph::linkChainToScan(
        const LocalizedRangeScanVector &chain,
        LocalizedRangeScan *scan,
        const Pose2 &mean,
        const Matrix3 &covariance)
/*****************************************************************************/
{
        Pose2 pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

        LocalizedRangeScan *closest_scan = getClosestScanToPose(chain, pose);
        assert(closest_scan != nullptr);

        Pose2 closest_scan_pose =
                closest_scan->getReferencePose(mapper_->use_scan_barycenter_);

        double squared_distance = pose.computeSquaredDistance(closest_scan_pose);
        if (squared_distance <
            math::Square(mapper_->link_scan_maximum_distance_) + KT_TOLERANCE) {
                linkScans(closest_scan, scan, mean, covariance);
        }
}

/*****************************************************************************/
void MapperGraph::linkNearChains(
        LocalizedRangeScan *scan, Pose2Vector &means,
        std::vector<Matrix3> &covariances)
/*****************************************************************************/
{
        const std::vector<LocalizedRangeScanVector> near_chains = findNearChains(scan);
        for (const auto& scan_chain : near_chains) {
                if (scan_chain.size() < mapper_->loop_match_minimum_chain_size_) {
                        continue;
                }

                Pose2 mean;
                Matrix3 covariance;
                // match scan against "near" chain
                double response = mapper_->sequential_scan_matcher_->matchScan(
                        scan, 
                        scan_chain, 
                        mean,
                        covariance, 
                        false);
                if (response > mapper_->link_match_minimum_response_fine_ - KT_TOLERANCE) {
                        means.push_back(mean);
                        covariances.push_back(covariance);
                        linkChainToScan(scan_chain, scan, mean, covariance);
                }
        }
}

/*****************************************************************************/
std::vector<LocalizedRangeScanVector> MapperGraph::findNearChains(LocalizedRangeScan *scan)
/*****************************************************************************/
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
                for (int32_t candidate_scan_num = near_scan->getScanId() - 1; candidate_scan_num >= 0;
                     candidate_scan_num--)
                {
                        LocalizedRangeScan *candidate_scan = mapper_->scan_manager_->getScan(candidate_scan_num);

                        // chain is invalid--contains scan being added
                        if (candidate_scan == scan)
                        {
                                is_valid_chain = false;
                        }

                        // probably removed in localization mode
                        if (candidate_scan == nullptr)
                        {
                                continue;
                        }

                        Pose2 candidate_pose = candidate_scan->getReferencePose(mapper_->use_scan_barycenter_);
                        double squaredDistance = scan_pose.computeSquaredDistance(candidate_pose);

                        if (squaredDistance <
                            math::Square(mapper_->link_scan_maximum_distance_) + KT_TOLERANCE)
                        {
                                chain.push_front(candidate_scan);
                                processed.push_back(candidate_scan);
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
                for (uint32_t candidate_scan_num = near_scan->getScanId() + 1; candidate_scan_num < end;
                     candidate_scan_num++)
                {
                        LocalizedRangeScan *candidate_scan = mapper_->scan_manager_->getScan(candidate_scan_num);

                        if (candidate_scan == scan)
                        {
                                is_valid_chain = false;
                        }

                        // probably removed in localization mode
                        if (candidate_scan == nullptr)
                        {
                                continue;
                        }

                        Pose2 candidate_pose = candidate_scan->getReferencePose(mapper_->use_scan_barycenter_);
                        double squaredDistance =
                            scan_pose.computeSquaredDistance(candidate_pose);

                        if (squaredDistance <
                            math::Square(mapper_->link_scan_maximum_distance_) + KT_TOLERANCE)
                        {
                                chain.push_back(candidate_scan);
                                processed.push_back(candidate_scan);
                        }
                        else
                        {
                                break;
                        }
                }

                if (is_valid_chain)
                {
                        // change list to vector
                        LocalizedRangeScanVector temp_chain;
                        std::copy(chain.begin(), chain.end(), std::inserter(temp_chain, temp_chain.begin()));
                        // add chain to collection
                        near_chains.push_back(temp_chain);
                }
        }

        return near_chains;
}

/*****************************************************************************/
Pose2 MapperGraph::computeWeightedMean(
        const Pose2Vector &means,
        const std::vector<Matrix3> &covariances) const
/*****************************************************************************/
{
        assert(means.size() == covariances.size());

        // compute sum of inverses and create inverse list
        std::vector<Matrix3> inverses;
        inverses.reserve(covariances.size());

        Matrix3 sum_of_inverses;
        for (const auto& cov : covariances) {
                Matrix3 inverse = cov.inverse();
                inverses.push_back(inverse);

                sum_of_inverses += inverse;
        }
        Matrix3 inverse_of_sum_of_inverses = sum_of_inverses.inverse();

        // compute weighted mean
        Pose2 accumulated_pose;
        double theta_x = 0.0;
        double theta_y = 0.0;

        Pose2Vector::const_iterator means_iter = means.begin();
        for (const auto& inverse_iter : inverses) {
                Pose2 pose = *means_iter;
                double angle = pose.getHeading();
                theta_x += cos(angle);
                theta_y += sin(angle);

                Matrix3 weight = inverse_of_sum_of_inverses * (inverse_iter);
                accumulated_pose += weight * pose;

                ++means_iter;
        }

        theta_x /= means.size();
        theta_y /= means.size();
        accumulated_pose.setHeading(atan2(theta_y, theta_x));

        return accumulated_pose;
}

/*****************************************************************************/
LocalizedRangeScan *MapperGraph::getClosestScanToPose(
        const LocalizedRangeScanVector &scans,
        const Pose2 &pose) const
/*****************************************************************************/
{
        LocalizedRangeScan *closest_scan = nullptr;
        double best_squared_distance = DBL_MAX;

        for (const auto& scan : scans) {
                Pose2 scan_pose = scan->getReferencePose(mapper_->use_scan_barycenter_);

                double squared_distance = pose.computeSquaredDistance(scan_pose);
                if (squared_distance < best_squared_distance) {
                        best_squared_distance = squared_distance;
                        closest_scan = scan;
                }
        }

        return closest_scan;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
bool Mapper::process(LocalizedRangeScan * scan, Matrix3 * covariance)
/*****************************************************************************/
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
                        Transform lastTransform(
                                last_scan->getOdometricPose(),
                                last_scan->getCorrectedPose());
                        scan->setCorrectedPose(lastTransform.transformPose(scan->getOdometricPose()));
                }

                Matrix3 cov;
                cov.setToIdentity();

                // correct scan (if not first scan)
                if (use_scan_matching_ && last_scan != nullptr) {
                        Pose2 best_pose;
                        sequential_scan_matcher_->matchScan(
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
                if (use_scan_matching_) {
                        // add to graph
                        graph_->addVertex(scan);
                        graph_->addEdges(scan, cov);

                        scan_manager_->addRunningScan(scan);

                        if (do_loop_closing_) {
                                graph_->tryCloseLoop(scan);
                        }
                }
                
                scan_manager_->setLastScan(scan);

                return true;
        }
        
        return false;
}

/*****************************************************************************/
void Mapper::setParamUseScanMatching(bool b)
/*****************************************************************************/
{
        use_scan_matching_ = b;
}

/*****************************************************************************/
void Mapper::setParamUseScanBarycenter(bool b)
/*****************************************************************************/
{
        use_scan_barycenter_ = b;
}

/*****************************************************************************/
void Mapper::setParamMinimumTravelDistance(double d)
/*****************************************************************************/
{
        minimum_travel_distance_ = d;
}

/*****************************************************************************/
/**
 * @param d in radians
 */
void Mapper::setParamMinimumTravelHeading(double d)
/*****************************************************************************/
{
        minimum_travel_heading_ = d;
}

/*****************************************************************************/
void Mapper::setParamScanBufferSize(int i)
/*****************************************************************************/
{
        scan_buffer_size_ = (uint32_t)i;
}

/*****************************************************************************/
void Mapper::setParamScanBufferMaximumScanDistance(double d)
/*****************************************************************************/
{
        scan_buffer_maximum_scan_distance_ = d;
}

/*****************************************************************************/
void Mapper::setParamLinkMatchMinimumResponseFine(double d)
/*****************************************************************************/
{
        link_match_minimum_response_fine_ = d;
}

/*****************************************************************************/
void Mapper::setParamLinkScanMaximumDistance(double d)
/*****************************************************************************/
{
        link_scan_maximum_distance_ = d;
}

/*****************************************************************************/
void Mapper::setParamLoopSearchMaximumDistance(double d)
/*****************************************************************************/
{
        loop_search_maximum_distance_ = d;
}

/*****************************************************************************/
void Mapper::setParamDoLoopClosing(bool b)
/*****************************************************************************/
{
        do_loop_closing_ = b;
}

/*****************************************************************************/
void Mapper::setParamLoopMatchMinimumChainSize(int i)
/*****************************************************************************/
{
        loop_match_minimum_chain_size_ = (uint32_t)i;
}

/*****************************************************************************/
void Mapper::setParamLoopMatchMaximumVarianceCoarse(double d)
/*****************************************************************************/
{
        loop_match_maximum_variance_coarse_ = d;
}

/*****************************************************************************/
void Mapper::setParamLoopMatchMinimumResponseCoarse(double d)
/*****************************************************************************/
{
        loop_match_minimum_response_coarse_ = d;
}

/*****************************************************************************/
void Mapper::setParamLoopMatchMinimumResponseFine(double d)
/*****************************************************************************/
{
        loop_match_minimum_response_fine_ = d;
}

/*****************************************************************************/
void Mapper::setParamCorrelationSearchSpaceDimension(double d)
/*****************************************************************************/
{
        correlation_search_space_dimension_ = d;
}

/*****************************************************************************/
void Mapper::setParamCorrelationSearchSpaceResolution(double d)
/*****************************************************************************/
{
        correlation_search_space_resolution_ = d;
}

/*****************************************************************************/
void Mapper::setParamCorrelationSearchSpaceSmearDeviation(double d)
/*****************************************************************************/
{
        correlation_search_space_smear_deviation_ = d;
}

/*****************************************************************************/
void Mapper::setParamLoopSearchSpaceDimension(double d)
/*****************************************************************************/
{
        loop_search_space_dimension_ = d;
}

/*****************************************************************************/
void Mapper::setParamLoopSearchSpaceResolution(double d)
/*****************************************************************************/
{
        loop_search_space_resolution_ = d;
}

/*****************************************************************************/
void Mapper::setParamLoopSearchSpaceSmearDeviation(double d)
/*****************************************************************************/
{
        loop_search_space_smear_deviation_ = d;
}

/*****************************************************************************/
void Mapper::setParamDistanceVariancePenalty(double d)
/*****************************************************************************/
{
        distance_variance_penalty_ = math::Square(d);
}

/*****************************************************************************/
void Mapper::setParamAngleVariancePenalty(double d)
/*****************************************************************************/
{
        angle_variance_penalty_ = math::Square(d);
}

/*****************************************************************************/
void Mapper::setParamFineSearchAngleOffset(double d)
/*****************************************************************************/
{
        fine_search_angle_offset_ = d;
}

/*****************************************************************************/
void Mapper::setParamCoarseSearchAngleOffset(double d)
/*****************************************************************************/
{
        coarse_search_angle_offset_ = d;
}

/*****************************************************************************/
void Mapper::setParamCoarseAngleResolution(double d)
/*****************************************************************************/
{
        coarse_angle_resolution_ = d;
}

/*****************************************************************************/
void Mapper::setParamMinimumAnglePenalty(double d)
/*****************************************************************************/
{
        minimum_angle_penalty_ = d;
}

/*****************************************************************************/
void Mapper::setParamMinimumDistancePenalty(double d)
/*****************************************************************************/
{
        minimum_distance_penalty_ = d;
}

/*****************************************************************************/
void Mapper::setParamUseResponseExpansion(bool b)
/*****************************************************************************/
{
        use_response_expansion_ = b;
}

/*****************************************************************************/
void Mapper::setParamMinPassThrough(int i)
/*****************************************************************************/
{
        min_pass_through_ = (uint32_t)i;
}

/*****************************************************************************/
void Mapper::setParamOccupancyThreshold(double d)
/*****************************************************************************/
{
        occupancy_threshold_ = d;
}

/*****************************************************************************/
bool Mapper::getParamUseScanMatching()
/*****************************************************************************/
{
        return static_cast<bool>(use_scan_matching_);
}

/*****************************************************************************/
bool Mapper::getParamUseScanBarycenter()
/*****************************************************************************/
{
        return static_cast<bool>(use_scan_barycenter_);
}

/*****************************************************************************/
double Mapper::getParamMinimumTravelDistance()
/*****************************************************************************/
{
        return static_cast<double>(minimum_travel_distance_);
}

/*****************************************************************************/
/**
 * @return Minimum travel heading in radian
 */
double Mapper::getParamMinimumTravelHeading()
/*****************************************************************************/
{
        return static_cast<double>(minimum_travel_heading_);
}

/*****************************************************************************/
int Mapper::getParamScanBufferSize()
/*****************************************************************************/
{
        return static_cast<int>(scan_buffer_size_);
}

/*****************************************************************************/
double Mapper::getParamScanBufferMaximumScanDistance()
/*****************************************************************************/
{
        return static_cast<double>(scan_buffer_maximum_scan_distance_);
}

/*****************************************************************************/
double Mapper::getParamLinkMatchMinimumResponseFine()
/*****************************************************************************/
{
        return static_cast<double>(link_match_minimum_response_fine_);
}

/*****************************************************************************/
double Mapper::getParamLinkScanMaximumDistance()
/*****************************************************************************/
{
        return static_cast<double>(link_scan_maximum_distance_);
}

/*****************************************************************************/
double Mapper::getParamLoopSearchMaximumDistance()
/*****************************************************************************/
{
        return static_cast<double>(loop_search_maximum_distance_);
}

/*****************************************************************************/
bool Mapper::getParamDoLoopClosing()
/*****************************************************************************/
{
        return static_cast<bool>(do_loop_closing_);
}

/*****************************************************************************/
int Mapper::getParamLoopMatchMinimumChainSize()
/*****************************************************************************/
{
        return static_cast<int>(loop_match_minimum_chain_size_);
}

/*****************************************************************************/
double Mapper::getParamLoopMatchMaximumVarianceCoarse()
/*****************************************************************************/
{
        return static_cast<double>(std::sqrt(loop_match_maximum_variance_coarse_));
}

/*****************************************************************************/
double Mapper::getParamLoopMatchMinimumResponseCoarse()
/*****************************************************************************/
{
        return static_cast<double>(loop_match_minimum_response_coarse_);
}

/*****************************************************************************/
double Mapper::getParamLoopMatchMinimumResponseFine()
/*****************************************************************************/
{
        return static_cast<double>(loop_match_minimum_response_fine_);
}

// Correlation Parameters - Correlation Parameters

/*****************************************************************************/
double Mapper::getParamCorrelationSearchSpaceDimension()
/*****************************************************************************/
{
        return static_cast<double>(correlation_search_space_dimension_);
}

/*****************************************************************************/
double Mapper::getParamCorrelationSearchSpaceResolution()
/*****************************************************************************/
{
        return static_cast<double>(correlation_search_space_resolution_);
}

/*****************************************************************************/
double Mapper::getParamCorrelationSearchSpaceSmearDeviation()
/*****************************************************************************/
{
        return static_cast<double>(correlation_search_space_smear_deviation_);
}

/*****************************************************************************/
/**
 * Correlation Parameters - Loop Correlation Parameters
 */
double Mapper::getParamLoopSearchSpaceDimension()
/*****************************************************************************/
{
        return static_cast<double>(loop_search_space_dimension_);
}

/*****************************************************************************/
double Mapper::getParamLoopSearchSpaceResolution()
/*****************************************************************************/
{
        return static_cast<double>(loop_search_space_resolution_);
}

/*****************************************************************************/
double Mapper::getParamLoopSearchSpaceSmearDeviation()
/*****************************************************************************/
{
        return static_cast<double>(loop_search_space_smear_deviation_);
}

// ScanMatcher Parameters

/*****************************************************************************/
double Mapper::getParamDistanceVariancePenalty()
/*****************************************************************************/
{
        return std::sqrt(static_cast<double>(distance_variance_penalty_));
}

/*****************************************************************************/
double Mapper::getParamAngleVariancePenalty()
/*****************************************************************************/
{
        return std::sqrt(static_cast<double>(angle_variance_penalty_));
}

/*****************************************************************************/
double Mapper::getParamFineSearchAngleOffset()
/*****************************************************************************/
{
        return static_cast<double>(fine_search_angle_offset_);
}

/*****************************************************************************/
double Mapper::getParamCoarseSearchAngleOffset()
/*****************************************************************************/
{
        return static_cast<double>(coarse_search_angle_offset_);
}

/*****************************************************************************/
double Mapper::getParamCoarseAngleResolution()
/*****************************************************************************/
{
        return static_cast<double>(coarse_angle_resolution_);
}

/*****************************************************************************/
double Mapper::getParamMinimumAnglePenalty()
/*****************************************************************************/
{
        return static_cast<double>(minimum_angle_penalty_);
}

/*****************************************************************************/
double Mapper::getParamMinimumDistancePenalty()
/*****************************************************************************/
{
        return static_cast<double>(minimum_distance_penalty_);
}

/*****************************************************************************/
bool Mapper::getParamUseResponseExpansion()
/*****************************************************************************/
{
        return static_cast<bool>(use_response_expansion_);
}

/*****************************************************************************/
int Mapper::getParamMinPassThrough()
/*****************************************************************************/
{
        return static_cast<int>(min_pass_through_);
}

/*****************************************************************************/
double Mapper::getParamOccupancyThreshold()
/*****************************************************************************/
{
        return static_cast<double>(occupancy_threshold_);
}

} // namespace mapper_utils