#ifndef MYSLAM_TYPES_HPP
#define MYSLAM_TYPES_HPP

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <Eigen/Geometry>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace myslam_types
{

/**
 * Represents a size (width, height) in 2-dimensional real space.
 */
template <typename T>
class Size2
{
public:
        /**
         * Default constructor
         */
        Size2()
                : width_(0),
                height_(0)
        {
        }

        /**
         * Constructor initializing point location
         * @param width
         * @param height
         */
        Size2(T width, T height)
                : width_(width),
                height_(height)
        {
        }

        /**
         * Gets the width
         * @return the width
         */
        inline const T getWidth() const
        {
                return width_;
        }

        /**
         * Gets the height
         * @return the height
         */
        inline const T getHeight() const
        {
                return height_;
        }

private:
        T width_;
        T height_;

}; // Size2

/////////////////////////////////////////////////////////

/**
 * @brief 
 */
class Pose2
{
public:
        // Default constructor: Identity transformation
        Pose2()
            : pose_(Eigen::Isometry2d::Identity())
        {
        }

        // Constructor with x, y, heading
        Pose2(double x, double y, double heading)
            : pose_(Eigen::Translation2d(x, y) * Eigen::Rotation2D<double>(heading))
        {
        }

        // Constructor with Vector2d position and heading
        Pose2(const Eigen::Vector2d &position, double heading)
            : pose_(Eigen::Translation2d(position) * Eigen::Rotation2D<double>(heading))
        {
        }

        Pose2(const Eigen::Isometry2d &pose)
                : pose_(pose)
        {
        }

        inline double getX() const
        {
                return pose_.translation().x(); 
        }

        inline double getY() const
        {
                return pose_.translation().y();
        }

        inline double getHeading() const
        {
                return Eigen::Rotation2D<double>(pose_.linear()).angle();
        }

        inline Eigen::Vector2d getPosition() const
        {
                return pose_.translation();
        }

        inline double getSquaredDistance(const Pose2 &other) const
        {
                return (pose_.translation() - other.pose_.translation()).squaredNorm();
        }

        inline bool operator==(const Pose2 &other) const
        {
                return pose_.translation() == other.pose_.translation() && this->getHeading() == other.getHeading();
        }

        inline Pose2 inverse() const
        {
                return Pose2(pose_.inverse());
        }

        /**
         * @return left_pose * right_pose
         */
        static Pose2 transformPose(const Pose2 &left_pose, const Pose2 &right_pose)
        {
                return Pose2(left_pose.pose_ * right_pose.pose_);
        }

        /**
         * @param target_pose T_a_b 
         * @param source_pose T_c_b
         * @return T_a_c
         */
        static Pose2 getRelativePose(const Pose2& target_pose, const Pose2& source_pose)
        {
                return Pose2(target_pose.pose_ * source_pose.pose_.inverse());
        }

private:
        Eigen::Isometry2d pose_;

}; // Pose2

/////////////////////////////////////////////////////////

class BoundingBox2
{
public:
        BoundingBox2()
                : minimum_(999999999999999999.99999, 999999999999999999.99999),
                maximum_(-999999999999999999.99999, -999999999999999999.99999)
        {
        }

        /**
         * Get bounding box minimum
         */
        inline const Eigen::Vector2d &getMinimum() const
        {
                return minimum_;
        }

        /**
         * Set bounding box minimum
         */
        inline void setMinimum(const Eigen::Vector2d &minimum)
        {
                minimum_ = minimum;
        }

        /**
         * Get bounding box maximum
         */
        inline const Eigen::Vector2d &getMaximum() const
        {
                return maximum_;
        }

        /**
         * Set bounding box maximum
         */
        inline void setMaximum(const Eigen::Vector2d &maximum)
        {
                maximum_ = maximum;
        }

        /**
         * Get the size of the bounding box
         */
        inline Size2<double> getSize() const
        {
                Eigen::Vector2d size = maximum_ - minimum_;

                return Size2<double>(size.x(), size.y());
        }

        /**
         * Add vector to bounding box
         */
        inline void add(const Eigen::Vector2d &point)
        {
                minimum_ = minimum_.cwiseMin(point);
                maximum_ = maximum_.cwiseMax(point);
        }

        /**
         * Add other bounding box to bounding box
         */
        inline void add(const BoundingBox2 &bounding_box)
        {
                add(bounding_box.getMinimum());
                add(bounding_box.getMaximum());
        }

private:
        Eigen::Vector2d minimum_;
        Eigen::Vector2d maximum_;
}; // BoundingBox2

///////////////////////////////////////////////////////////////

template<typename T>
class Rectangle2
{
private:
        Eigen::Matrix<T, 2, 1> position_;
        Size2<T> size_; 
public:
        Rectangle2()
        {
        }

        Rectangle2(const Rectangle2 &other)
            : position_(other.position_),
              size_(other.size_)
        {
        }

public:
        inline T getWidth() const
        {
                return size_.getWidth();
        }

        inline T getHeight() const
        {
                return size_.getHeight();
        }

}; // Rectangle2

///////////////////////////////////////////////////////////////

/**
 * An array that can be resized as long as the size
 * does not exceed the initial capacity
 */
class LookupArray
{
private:
        std::unique_ptr<int32_t[]> array_;
        uint32_t capacity_;
        uint32_t size_;
public:
        LookupArray()
        : array_(nullptr), capacity_(0), size_(0)
        {
        }

        /**
         * Sets size of array (resize if not big enough)
         * @param size
         */
        void setSize(uint32_t size)
        {
                assert(size != 0);

                if (size > capacity_)
                {
                        if (array_ != nullptr) {
                                array_.reset();
                        }
                        capacity_ = size;
                        array_ = std::make_unique<int32_t[]>(capacity_);
                }

                size_ = size;
        }

        /**
         * Gets size of array
         * @return array size
         */
        uint32_t getSize() const
        {
                return size_;
        }

        /**
         * Gets array pointer
         * @return array pointer
         */
        inline int32_t *getArrayPointer()
        {
                return array_.get();
        }

        /**
         * Gets array pointer
         * @return array pointer
         */
        inline int32_t *getArrayPointer() const
        {
                return array_.get();
        }

}; // LookupArray

///////////////////////////////////////////////////////////////

enum class GridStates : uint8_t
{
        UNKNOWN = 0,
        OCCUPIED = 100,
        FREE = 255
};

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;

typedef std::vector<Pose2> Pose2Vector;
typedef Eigen::Matrix<int32_t, 2, 1> Vector2i;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Matrix3d Matrix3d;

} // namespace myslam_types

#endif // MYSLAM_TYPES_HPP