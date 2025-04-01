#ifndef LASER_HELPER_HPP
#define LASER_HELPER_HPP

namespace laser_utils
{

/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized range scan relative to the robot.
 */
class LaserRangeFinder
{
public:
        LaserRangeFinder()
        {
        }
        virtual ~LaserRangeFinder()
        {
        }
};

class ScanHolder
{

}; 

} // namespace laser_helper


#endif // LASER_HELPER_HPP