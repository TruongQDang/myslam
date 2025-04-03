#include "myslam/myslam_common.hpp"

namespace myslam
{
/*****************************************************************************/
MySlamCommon::MySlamCommon(rclcpp::NodeOptions options)
        : rclcpp_lifecycle::LifecycleNode("myslam", "", options),
        transform_timeout_(rclcpp::Duration::from_seconds(0.5)),
        minimum_time_interval_(std::chrono::nanoseconds(0)),
        first_measurement_(true)
        
/*****************************************************************************/
{
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_configure(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Configuring...");

        setParams();

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
        pose_helper_ = std::make_unique<pose_utils::GetPoseHelper>(tf_.get(), base_frame_, odom_frame_);
        
        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_activate(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Activating...");

        setROSInterfaces();

        double transform_publish_period = 0.05; // default value
        threads_.push_back(
            std::make_unique<boost::thread>([this, transform_publish_period]()
                { this->publishTransformLoop(transform_publish_period); })
        );

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_deactivate(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Deactivating...");

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_cleanup(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Cleaning up...");

        tf_broadcaster_.reset();

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_shutdown(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Shutting down...");

        tf_broadcaster_.reset();

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
MySlamCommon::~MySlamCommon()
/*****************************************************************************/
{
        tf_broadcaster_.reset();
}

/*****************************************************************************/
void MySlamCommon::setParams()
/*****************************************************************************/
{
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

        scan_topic_ = std::string("/lidar");
        if (!this->has_parameter("scan_topic"))
        {
                this->declare_parameter("scan_topic", scan_topic_);
        }
        scan_topic_ = this->get_parameter("scan_topic").as_string();

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

        scan_queue_size_ = 1;
        if (!this->has_parameter("scan_queue_size"))
        {
                this->declare_parameter("scan_queue_size", scan_queue_size_);
        }
        scan_queue_size_ = this->get_parameter("scan_queue_size").as_int();

        double tmp_val = 0.5;
        if (!this->has_parameter("transform_timeout"))
        {
                this->declare_parameter("transform_timeout", tmp_val);
        }
        tmp_val = this->get_parameter("transform_timeout").as_double();
        transform_timeout_ = rclcpp::Duration::from_seconds(tmp_val);
        if (!this->has_parameter("minimum_time_interval"))
        {
                this->declare_parameter("minimum_time_interval", tmp_val);
        }
        tmp_val = this->get_parameter("minimum_time_interval").as_double();
        minimum_time_interval_ = rclcpp::Duration::from_seconds(tmp_val);
}

/*****************************************************************************/
void MySlamCommon::setROSInterfaces()
/*****************************************************************************/
{
        scan_filter_subcriber_ =
            std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
                                                         rclcpp_lifecycle::LifecycleNode>>(
                shared_from_this().get(), scan_topic_, rmw_qos_profile_sensor_data);
        scan_filter_ =
            std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
                *scan_filter_subcriber_, *tf_, odom_frame_, scan_queue_size_,
                get_node_logging_interface(), get_node_clock_interface(),
                tf2::durationFromSec(transform_timeout_.seconds()));
        scan_filter_->registerCallback(
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan) {
                this->laserCallback(scan);
            });
}

/*****************************************************************************/
void MySlamCommon::publishTransformLoop(const double &transform_publish_period)
/*****************************************************************************/
{
        rclcpp::Rate rate(1.0 / transform_publish_period);
        while (rclcpp::ok()) {
                rclcpp::Time scan_timestamp = this->now();
                geometry_msgs::msg::TransformStamped msg;
                msg.transform.translation.x = 1.0;
                msg.transform.translation.y = 2.0;
                msg.transform.rotation.w = 1.0;
                msg.header.frame_id = "odom";
                msg.child_frame_id = "base_link";
                tf_broadcaster_->sendTransform(msg);
                rate.sleep();
        }
}

/*****************************************************************************/
void MySlamCommon::publishVisualizations()
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

        double map_update_interval = 10.0;
        if (!this->has_parameter("map_update_interval"))
        {
                this->declare_parameter("map_update_interval", map_update_interval);
        }
        map_update_interval = this->get_parameter("map_update_interval").as_double();
        rclcpp::Rate r(1.0 / map_update_interval);

        while (rclcpp::ok())
        {
                boost::this_thread::interruption_point();
                updateMap();

                r.sleep();
        }
}

/*****************************************************************************/
bool MySlamCommon::updateMap()
/*****************************************************************************/
{
}

/*****************************************************************************/
bool MySlamCommon::shouldProcessScan(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
        const Pose2 &pose)
/*****************************************************************************/
{
        static Pose2 last_pose;
        static rclcpp::Time last_scan_time = rclcpp::Time(0.);
        static double min_dist2 =
                mapper_->getMinimumTravelDistance() * 
                mapper_->getMinimumTravelDistance();
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
        const double dist2 = last_pose.SquaredDistance(pose);
        if (dist2 < 0.8 * min_dist2 || scan_count < 5)
        {
                return false;
        }

        last_pose = pose;
        last_scan_time = scan->header.stamp;

        return true;
}

/*****************************************************************************/
laser_utils::LocalizedRangeScan *MySlamCommon::addScan(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
        Pose2 &pose)
/*****************************************************************************/
{
        laser_utils::LocalizedRangeScan *range_scan;
        Eigen::Matrix3d covariance;
        covariance.setIdentity();

        bool processed = false;
        processed = mapper_->process(range_scan, &covariance);

        // if sucessfully processed, create odom2map transform and add scan to storage
        if (processed) {

        }

        return range_scan;
}

/*****************************************************************************/
tf2::Stamped<tf2::Transform> MySlamCommon::setTransformFromPoses(
        const Pose2 &corrected_pose,
        const Pose2 &odom_pose, const rclcpp::Time &t,
        const bool &update_reprocessing_transform)
/*****************************************************************************/
{
        // Compute the map->odom transform
        tf2::Stamped<tf2::Transform> odom_to_map;
        tf2::Quaternion q(0., 0., 0., 1.0);
        q.setRPY(0., 0., corrected_pose.getHeading());
        tf2::Stamped<tf2::Transform> base_to_map(
            tf2::Transform(q, tf2::Vector3(corrected_pose.getX(),
                                           corrected_pose.getY(), 0.0))
                .inverse(),
            tf2_ros::fromMsg(t), base_frame_);
        try
        {
                geometry_msgs::msg::TransformStamped base_to_map_msg, odom_to_map_msg;

                // https://github.com/ros2/geometry2/issues/176
                // not working for some reason...
                // base_to_map_msg = tf2::toMsg(base_to_map);
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
                                      tf2::Vector3(odom_to_map.getOrigin()))
                           .inverse();

        return odom_to_map;
}

void MySlamCommon::publishPose(
        const Pose2 &pose,
        const Eigen::Matrix3d &cov,
        const rclcpp::Time &t)
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

} // namespace myslam