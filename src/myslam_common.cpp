#include "myslam/myslam_common.hpp"

namespace myslam
{
/*****************************************************************************/
MySlamCommon::MySlamCommon(rclcpp::NodeOptions options)
: rclcpp_lifecycle::LifecycleNode("myslam", "", options)
/*****************************************************************************/
{
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_configure(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Configuring...");
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_activate(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Activating...");

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

        tfBroadcaster_.reset();

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn MySlamCommon::on_shutdown(const rclcpp_lifecycle::State &)
/*****************************************************************************/
{
        RCLCPP_INFO(get_logger(), "Shutting down...");

        tfBroadcaster_.reset();

        return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
MySlamCommon::~MySlamCommon()
/*****************************************************************************/
{
        tfBroadcaster_.reset();
}

// /*****************************************************************************/
// void MySlamCommon::publishTransformLoop(const double &transform_publish_period)
// /*****************************************************************************/
// {

// }

} // namespace myslam