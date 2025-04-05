#include "myslam/laser_helper.hpp"

namespace laser_utils
{

template<class NodeT>
LaserRangeFinder::LaserRangeFinder(
        NodeT node, 
        tf2_ros::Buffer *tf, 
        const std::string &base_frame)
        :  logger_(node->get_logger()), tf_(tf), base_frame_(base_frame)
{        
}

bool LaserRangeFinder::makeLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
{
        laser_frame_ = scan->header.frame_id;

        geometry_msgs::msg::TransformStamped laser_ident;
        laser_ident.header.stamp = scan->header.stamp;
        laser_ident.header.frame_id = laser_frame_;
        laser_ident.transform.rotation.w = 1.0;

        laser_pose_ = tf_->transform(laser_ident, base_frame_);

        return true;
}

///////////////////////////////////////////////////////////////////

LocalizedRangeScan::LocalizedRangeScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
{

}

} // namespace laser_utils