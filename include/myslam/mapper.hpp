#ifndef MYSLAM_MAPPER_HPP
#define MYSLAM_MAPPER_HPP

namespace mapper_utils
{

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
private:
        // parameters
        double minimum_travel_distance_;
        double minimum_travel_heading_;
};

} // namespace mapper_utils

#endif // MYSLAM_MAPPER_HPP