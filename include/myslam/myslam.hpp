#ifndef MYSLAM_HPP
#define MYSLAM_HPP

#include "myslam/myslam_common.hpp"

namespace myslam 
{

class MySlam : public MySlamCommon
{
public:
        explicit MySlam(rclcpp::NodeOptions options);
};

} // namespace myslam

#endif // MYSLAM_HPP