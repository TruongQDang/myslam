#ifndef MYSLAM_HPP
#define MYSLAM_HPP

#include "myslam/myslam_common.hpp"

namespace myslam 
{

class MySlam : public MySlamCommon
{
public:
        explicit MySlam(rclcpp::NodeOptions options);
protected:
        void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;
};

} // namespace myslam

#endif // MYSLAM_HPP