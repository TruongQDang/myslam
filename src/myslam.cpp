#include "myslam/myslam.hpp"
#include "solvers/ceres_solver.hpp"

namespace myslam
{

/*****************************************************************************/
MySlam::MySlam(rclcpp::NodeOptions options)
        : rclcpp_lifecycle::LifecycleNode("myslam", "", options),
        transform_timeout_(rclcpp::Duration::from_seconds(0.5)),
        minimum_time_interval_(std::chrono::nanoseconds(0)),
        first_measurement_(true),
        laser_(nullptr)
/*****************************************************************************/
{
        int stack_size = 40'000'000;
        {
                rcl_interfaces::msg::ParameterDescriptor descriptor;
                descriptor.read_only = true;
                this->declare_parameter(
                    "stack_size_to_use", rclcpp::ParameterType::PARAMETER_INTEGER, descriptor);
                if (this->get_parameter("stack_size_to_use", stack_size))
                {
                        RCLCPP_INFO(get_logger(), "Node using stack size %i", (int)stack_size);
                        struct rlimit stack_limit;
                        getrlimit(RLIMIT_STACK, &stack_limit);
                        if (stack_limit.rlim_cur < static_cast<rlim_t>(stack_size))
                        {
                                stack_limit.rlim_cur = stack_size;
                        }
                        setrlimit(RLIMIT_STACK, &stack_limit);
                }
        }
}

/*****************************************************************************/
void MySlam::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
        // get transform from odom to base
        scan_header_ = scan->header;
        karto::Pose2 odom_pose;
        if (!pose_helper_->getPose(odom_pose, scan->header.stamp, odom_frame_, base_frame_)) {
                RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
                return;
        }

        // create laser if not initialized
        if (laser_ == nullptr) {
                makeLaser(scan);
        }

        if (shouldProcessScan(scan, odom_pose)) {
                addScan(scan, odom_pose);
        }
}

/*****************************************************************************/
CallbackReturn MySlam::on_configure(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Configuring...");

        first_measurement_ = true;
        mapper_ = std::make_unique<karto::Mapper>();
        mapper_->configure(shared_from_this());
        setSolver();

        setParams();

        double tmp_val = 30.0;
        if (!this->has_parameter("tf_buffer_duration"))
        {
                this->declare_parameter("tf_buffer_duration", tmp_val);
        }
        tmp_val = this->get_parameter("tf_buffer_duration").as_double();
        tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(),
                                                tf2::durationFromSec(tmp_val));
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                get_node_base_interface(),
                get_node_timers_interface());
        tf_->setCreateTimerInterface(timer_interface);

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
        pose_helper_ = std::make_unique<pose_utils::PoseHelper>(tf_.get());

        closure_assistant_ =
            std::make_unique<loop_closure_assistant::LoopClosureAssistant>(
                shared_from_this(), mapper_.get(), mapper_->getScanManager());

        RCLCPP_INFO(get_logger(), "Configured");

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlam::on_activate(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Activating...");

        setROSInterfaces();

        map_publisher_->on_activate();
        map_metadata_publisher_->on_activate();
        pose_publisher_->on_activate();

        double transform_publish_period = 0.02;
        if (!this->has_parameter("transform_publish_period"))
        {
                this->declare_parameter("transform_publish_period", transform_publish_period);
        }
        transform_publish_period = this->get_parameter("transform_publish_period").as_double();

        threads_.push_back(
                std::make_unique<boost::thread>(
                [this, transform_publish_period]() { 
                        this->publishTransformLoop(transform_publish_period); 
                })
        );

        threads_.push_back(
                std::make_unique<boost::thread>(
                [this]() {
                        this->publishVisualizations();
                })
        );

        RCLCPP_INFO(get_logger(), "Activated");

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlam::on_deactivate(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Deactivating...");

        for (long unsigned int i = 0; i != threads_.size(); i++)
        {
                threads_[i]->interrupt();
                threads_[i]->join();
                threads_[i].reset();
        }
        threads_.clear();
        map_publisher_->on_deactivate();
        map_metadata_publisher_->on_deactivate();
        pose_publisher_->on_deactivate();

        // reset interfaces
        scan_filter_.reset();
        scan_filter_subscriber_.reset();
        map_metadata_publisher_.reset();
        map_publisher_.reset();
        pose_publisher_.reset();

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlam::on_cleanup(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Cleaning up...");

        closure_assistant_.reset();
        tf_broadcaster_.reset();
        mapper_.reset();
        pose_helper_.reset();

        tf_broadcaster_.reset();
        tf_listener_.reset();
        tf_.reset();

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlam::on_shutdown(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Shutting down...");

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
MySlam::~MySlam()
/*****************************************************************************/
{
        for (long unsigned int i = 0; i != threads_.size(); i++)
        {
                threads_[i]->interrupt();
                threads_[i]->join();
                threads_[i].reset();
        }
        threads_.clear();

        mapper_.reset();
        pose_helper_.reset();
        closure_assistant_.reset();

        scan_filter_.reset();
        scan_filter_subscriber_.reset();
        map_publisher_.reset();
        map_metadata_publisher_.reset();
        pose_publisher_.reset();

        tf_broadcaster_.reset();
        tf_listener_.reset();
        tf_.reset();
}

/*****************************************************************************/
void MySlam::setParams()
/*****************************************************************************/
{
        map_to_odom_.setIdentity();
        odom_frame_ = std::string("odom");
        if (!this->has_parameter("odom_frame"))
        {
                this->declare_parameter("odom_frame", odom_frame_);
        }
        odom_frame_ = this->get_parameter("odom_frame").as_string();

        map_frame_ = std::string("map");
        if (!this->has_parameter("map_frame"))
        {
                this->declare_parameter("map_frame", map_frame_);
        }
        map_frame_ = this->get_parameter("map_frame").as_string();

        base_frame_ = std::string("base_footprint");
        if (!this->has_parameter("base_frame"))
        {
                this->declare_parameter("base_frame", base_frame_);
        }
        base_frame_ = this->get_parameter("base_frame").as_string();

        resolution_ = 0.05;
        if (!this->has_parameter("resolution"))
        {
                this->declare_parameter("resolution", resolution_);
        }
        resolution_ = this->get_parameter("resolution").as_double();
        if (resolution_ <= 0.0)
        {
                RCLCPP_WARN(this->get_logger(),
                            "You've set resolution of map to be zero or negative,"
                            "this isn't allowed so it will be set to default value 0.05.");
                resolution_ = 0.05;
        }

        map_name_ = std::string("/map");
        if (!this->has_parameter("map_name"))
        {
                this->declare_parameter("map_name", map_name_);
        }
        map_name_ = this->get_parameter("map_name").as_string();

        scan_topic_ = std::string("/scan");
        if (!this->has_parameter("scan_topic"))
        {
                this->declare_parameter("scan_topic", scan_topic_);
        }
        scan_topic_ = this->get_parameter("scan_topic").as_string();

        scan_queue_size_ = 1;
        if (!this->has_parameter("scan_queue_size"))
        {
                this->declare_parameter("scan_queue_size", scan_queue_size_);
        }
        scan_queue_size_ = this->get_parameter("scan_queue_size").as_int();

        throttle_scans_ = 1;
        if (!this->has_parameter("throttle_scans"))
        {
                this->declare_parameter("throttle_scans", throttle_scans_);
        }
        throttle_scans_ = this->get_parameter("throttle_scans").as_int();
        if (throttle_scans_ == 0)
        {
                RCLCPP_WARN(this->get_logger(),
                            "You've set throttle_scans to be zero,"
                            "this isn't allowed so it will be set to default value 1.");
                throttle_scans_ = 1;
        }
        position_covariance_scale_ = 1.0;
        if (!this->has_parameter("position_covariance_scale"))
        {
                this->declare_parameter("position_covariance_scale", position_covariance_scale_);
        }
        position_covariance_scale_ = this->get_parameter("position_covariance_scale").as_double();

        yaw_covariance_scale_ = 1.0;
        if (!this->has_parameter("yaw_covariance_scale"))
        {
                this->declare_parameter("yaw_covariance_scale", yaw_covariance_scale_);
        }
        yaw_covariance_scale_ = this->get_parameter("yaw_covariance_scale").as_double();


        double tmp_val = 0.2;
        if (!this->has_parameter("transform_timeout"))
        {
                this->declare_parameter("transform_timeout", tmp_val);
        }
        tmp_val = this->get_parameter("transform_timeout").as_double();
        transform_timeout_ = rclcpp::Duration::from_seconds(tmp_val);
        tmp_val = 0.5;
        if (!this->has_parameter("minimum_time_interval"))
        {
                this->declare_parameter("minimum_time_interval", tmp_val);
        }
        tmp_val = this->get_parameter("minimum_time_interval").as_double();
        minimum_time_interval_ = rclcpp::Duration::from_seconds(tmp_val);
}

/*****************************************************************************/
void MySlam::setROSInterfaces()
/*****************************************************************************/
{
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "pose", 10);

        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                map_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        map_metadata_publisher_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
                map_name_ + "_metadata",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        scan_filter_subscriber_ =
                std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>(
                        shared_from_this().get(), 
                        scan_topic_, 
                        rmw_qos_profile_sensor_data);
        scan_filter_ =
                std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
                        *scan_filter_subscriber_, 
                        *tf_, 
                        odom_frame_, 
                        scan_queue_size_,
                        get_node_logging_interface(), 
                        get_node_clock_interface(),
                        tf2::durationFromSec(transform_timeout_.seconds()));
        scan_filter_->registerCallback(
                [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan) {
                        this->laserCallback(scan);
                });
}

/*****************************************************************************/
void MySlam::setSolver()
/*****************************************************************************/
{
        std::unique_ptr<solver_plugins::CeresSolver> solver = std::make_unique<solver_plugins::CeresSolver>();
        solver->configure(shared_from_this());
        mapper_->setScanSolver(std::move(solver));
}

/*****************************************************************************/
void MySlam::publishTransformLoop(const double &transform_publish_period)
/*****************************************************************************/
{
        if (transform_publish_period == 0) {
                return;
        }

        rclcpp::Rate r(1.0 / transform_publish_period);
        while (rclcpp::ok()) {
                boost::this_thread::interruption_point();
                {
                        boost::mutex::scoped_lock lock(map_to_odom_mutex_);
                        rclcpp::Time scan_timestamp = scan_header_.stamp;
                        // Avoid publishing tf with initial 0.0 scan timestamp
                        if (scan_timestamp.seconds() > 0.0 && !scan_header_.frame_id.empty()) {
                                geometry_msgs::msg::TransformStamped msg;
                                msg.transform = tf2::toMsg(map_to_odom_);
                                msg.child_frame_id = odom_frame_;
                                msg.header.frame_id = map_frame_;
                                msg.header.stamp = scan_timestamp + transform_timeout_;
                                tf_broadcaster_->sendTransform(msg);
                        }
                }
                r.sleep();
        }
}

/*****************************************************************************/
void MySlam::publishVisualizations()
/*****************************************************************************/
{
        nav_msgs::msg::OccupancyGrid &og = map_.map;
        og.info.resolution = resolution_;
        og.info.origin.position.x = 0.0;
        og.info.origin.position.y = 0.0;
        og.info.origin.position.z = 0.0;
        og.info.origin.orientation.x = 0.0;
        og.info.origin.orientation.y = 0.0;
        og.info.origin.orientation.z = 0.0;
        og.info.origin.orientation.w = 1.0;
        og.header.frame_id = map_frame_;

        double map_update_interval = 5.0;
        if (!this->has_parameter("map_update_interval"))
        {
                this->declare_parameter("map_update_interval", map_update_interval);
        }
        map_update_interval = this->get_parameter("map_update_interval").as_double();
        rclcpp::Rate r(1.0 / map_update_interval);

        while (rclcpp::ok()) {
                boost::this_thread::interruption_point();
                updateMap();
                {
                        boost::mutex::scoped_lock lock(mapper_mutex_);
                        closure_assistant_->publishGraph();
                }
                r.sleep();
        }
}

/*****************************************************************************/
bool MySlam::updateMap()
/*****************************************************************************/
{
        if (!map_publisher_ || !map_publisher_->is_activated() || map_publisher_->get_subscription_count() == 0) {
                return true;
        }
        boost::mutex::scoped_lock lock(mapper_mutex_);
        karto::OccupancyGrid *occ_grid = mapper_->getOccupancyGrid(resolution_);
        if (!occ_grid) {
                return false;
        }

        vis_utils::toNavMap(occ_grid, map_.map);

        // publish map as current
        map_.map.header.stamp = scan_header_.stamp;
        map_publisher_->publish(
            std::move(std::make_unique<nav_msgs::msg::OccupancyGrid>(map_.map)));
        map_metadata_publisher_->publish(
            std::move(std::make_unique<nav_msgs::msg::MapMetaData>(map_.map.info)));

        delete occ_grid;
        occ_grid = nullptr;
        return true;
}

/*****************************************************************************/
bool MySlam::shouldProcessScan(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
        const karto::Pose2 &pose)
/*****************************************************************************/
{
        static karto::Pose2 last_pose;
        static rclcpp::Time last_scan_time = rclcpp::Time(0.);
        static double min_dist2 =
                mapper_->getMinimumTravelDistance() * 
                mapper_->getMinimumTravelDistance();
        static double min_heading = mapper_->getMinimumTravelHeading();
        static int scan_count = 0;
        scan_count++;
        
        // always process first scan
        if (first_measurement_) {
                last_scan_time = scan->header.stamp;
                last_pose = pose;
                first_measurement_ = false;
                return true;
        }

        // only process every N = throttle_scans_ number of scans
        if ((scan_count % throttle_scans_) != 0) {
                return false;
        }
        
        // reject if not enough time between scans
        if (rclcpp::Time(scan->header.stamp) - last_scan_time < minimum_time_interval_) {
                return false;
        }

        // check if moved enough between scans, within 10% for correction error
        const double dist2 = last_pose.getSquaredDistance(pose);
        double heading_diff = karto::math::NormalizeAngle(pose.getHeading() - last_pose.getHeading());
        if (dist2 < 0.8 * min_dist2 && fabs(heading_diff) < 0.9 * min_heading || scan_count < 5) {
                return false;
        }

        last_pose = pose;
        last_scan_time = scan->header.stamp;

        return true;
}

/*****************************************************************************/
karto::LocalizedRangeScan *MySlam::addScan(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
        karto::Pose2 &odom_pose)
/*****************************************************************************/
{
        karto::LocalizedRangeScan *range_scan = getLocalizedRangeScan(laser_.get(), scan, odom_pose);

        // Add the localized range scan to the mapper
        boost::mutex::scoped_lock lock(mapper_mutex_);
        bool processed = false;

        Eigen::Matrix3d covariance;
        covariance.setIdentity();

        processed = mapper_->process(range_scan, &covariance);

        // if sucessfully processed, create odom2map transform
        if (processed) {
                setTransformFromPoses(
                        range_scan->getCorrectedPose(), 
                        odom_pose,
                        scan->header.stamp, 
                        false);

                publishPose(
                        range_scan->getCorrectedPose(), 
                        covariance, 
                        scan->header.stamp);
        } else {
                delete range_scan;
                range_scan = nullptr;
        }

        return range_scan;                
}

/*****************************************************************************/
tf2::Stamped<tf2::Transform> MySlam::setTransformFromPoses(
        const karto::Pose2 &corrected_pose,
        const karto::Pose2 &odom_pose, const rclcpp::Time &t,
        const bool &update_reprocessing_transform)
/*****************************************************************************/
{
        // Compute the map->odom transform
        tf2::Stamped<tf2::Transform> odom_to_map;
        tf2::Quaternion q(0., 0., 0., 1.0);
        q.setRPY(0., 0., corrected_pose.getHeading());
        tf2::Stamped<tf2::Transform> base_to_map(
                tf2::Transform(q, tf2::Vector3(corrected_pose.getX(),
                        corrected_pose.getY(), 0.0)).inverse(),
                tf2_ros::fromMsg(t), 
                base_frame_);
        try
        {
                geometry_msgs::msg::TransformStamped base_to_map_msg, odom_to_map_msg;

                base_to_map_msg.header.stamp = tf2_ros::toMsg(base_to_map.stamp_);
                base_to_map_msg.header.frame_id = base_to_map.frame_id_;
                base_to_map_msg.transform.translation.x = base_to_map.getOrigin().getX();
                base_to_map_msg.transform.translation.y = base_to_map.getOrigin().getY();
                base_to_map_msg.transform.translation.z = base_to_map.getOrigin().getZ();
                base_to_map_msg.transform.rotation = tf2::toMsg(base_to_map.getRotation());

                odom_to_map_msg = tf_->transform(base_to_map_msg, odom_frame_);
                tf2::fromMsg(odom_to_map_msg, odom_to_map);
        }
        catch (tf2::TransformException &e)
        {
                RCLCPP_ERROR(get_logger(), "Transform from base_link to odom failed: %s",
                             e.what());
                return odom_to_map;
        }

        // set map to odom for our transformation thread to publish
        boost::mutex::scoped_lock lock(map_to_odom_mutex_);
        map_to_odom_ = tf2::Transform(tf2::Quaternion(odom_to_map.getRotation()),
                tf2::Vector3(odom_to_map.getOrigin())).inverse();

        return odom_to_map;
}

/*****************************************************************************/
void MySlam::publishPose(
        const karto::Pose2 &pose,
        const Eigen::Matrix3d &cov,
        const rclcpp::Time &t)
/*****************************************************************************/
{
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = t;
        pose_msg.header.frame_id = map_frame_;

        tf2::Quaternion q(0., 0., 0., 1.0);
        q.setRPY(0., 0., pose.getHeading());
        tf2::Transform transform(q, tf2::Vector3(pose.getX(), pose.getY(), 0.0));
        tf2::toMsg(transform, pose_msg.pose.pose);

        pose_msg.pose.covariance[0] = cov(0, 0) * position_covariance_scale_; // x
        pose_msg.pose.covariance[1] = cov(0, 1) * position_covariance_scale_; // xy
        pose_msg.pose.covariance[6] = cov(1, 0) * position_covariance_scale_; // xy
        pose_msg.pose.covariance[7] = cov(1, 1) * position_covariance_scale_; // y
        pose_msg.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;     // yaw

        pose_publisher_->publish(pose_msg);
}

/*****************************************************************************/
karto::LocalizedRangeScan *MySlam::getLocalizedRangeScan(
        karto::LaserRangeFinder *laser,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
        karto::Pose2 &odom_pose)
/*****************************************************************************/
{
        // convert vector<float> to vector<double>
        karto::RangeReadingsVector readings(scan->ranges.size());
        std::copy(scan->ranges.begin(), scan->ranges.end(), readings.begin());

        karto::LocalizedRangeScan *range_scan = new karto::LocalizedRangeScan(laser, readings);
        range_scan->setOdometricPose(odom_pose);
        range_scan->setCorrectedPose(odom_pose);
        range_scan->setTime(rclcpp::Time(scan->header.stamp).nanoseconds() / 1.e9);

        return range_scan;
}

/*****************************************************************************/
void MySlam::makeLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
/*****************************************************************************/
{
        laser_ = std::make_unique<karto::LaserRangeFinder>();

        karto::Pose2 T_robot_laser;
        pose_helper_->getPose(T_robot_laser, scan->header.stamp, base_frame_, scan->header.frame_id);

        double max_laser_range = 25;
        if (!this->has_parameter("max_laser_range")) {
                this->declare_parameter("max_laser_range", max_laser_range);
        }
        max_laser_range = this->get_parameter("max_laser_range").as_double();

        if (max_laser_range <= 0) {
                RCLCPP_WARN(
                        get_logger(),
                        "You've set maximum_laser_range to be negative,"
                        "this isn't allowed so it will be set to (%.1f).",
                        scan->range_max);
                max_laser_range = scan->range_max;
        }

        if (max_laser_range > scan->range_max) {
                RCLCPP_WARN(
                        get_logger(),
                        "maximum laser range setting (%.1f m) exceeds the capabilities "
                        "of the used Lidar (%.1f m)",
                        max_laser_range, scan->range_max);
                max_laser_range = scan->range_max;
        }

        laser_->setRangeThreshold(max_laser_range);
        laser_->setFrameId(scan->header.frame_id);
        laser_->setOffsetPose(T_robot_laser);
        laser_->setMinimumRange(scan->range_min);
        laser_->setMaximumRange(scan->range_max);
        laser_->setMinimumAngle(scan->angle_min);
        laser_->setMaximumAngle(scan->angle_max);
        laser_->setAngularResolution(scan->angle_increment);
}


} // namespace myslam