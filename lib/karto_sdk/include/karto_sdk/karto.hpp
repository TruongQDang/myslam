#ifndef KARTO_SDK_KARTO_HPP
#define KARTO_SDK_KARTO_HPP

#include <memory>
#include <vector>
#include <boost/thread.hpp>
#include <iostream>

#include "math.hpp"

namespace karto
{

/**
 * Enumerated type for valid grid cell states
 */
enum GridStates
{
        GRIDSTATES_UNKNOWN = 0,
        GRIDSTATES_OCCUPIED = 100,
        GRIDSTATES_FREE = 255
};


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Represents a vector (x, y) in 2-dimensional real space.
 */
template <typename T>
class Vector2
{
public:
        /**
         * Default constructor
         */
        Vector2()
        {
                values_[0] = 0;
                values_[1] = 0;
        }

        /**
         * Constructor initializing vector location
         * @param x
         * @param y
         */
        Vector2(T x, T y)
        {
                values_[0] = x;
                values_[1] = y;
        }

public:
        /**
         * Gets the x-coordinate of this vector2
         * @return the x-coordinate of the vector2
         */
        inline const T &getX() const
        {
                return values_[0];
        }

        /**
         * Sets the x-coordinate of this vector2
         * @param x the x-coordinate of the vector2
         */
        inline void setX(const T &x)
        {
                values_[0] = x;
        }

        /**
         * Gets the y-coordinate of this vector2
         * @return the y-coordinate of the vector2
         */
        inline const T &getY() const
        {
                return values_[1];
        }

        /**
         * Sets the y-coordinate of this vector2
         * @param y the y-coordinate of the vector2
         */
        inline void setY(const T &y)
        {
                values_[1] = y;
        }

        /**
         * Floor point operator
         * @param other
         */
        inline void makeFloor(const Vector2 &other)
        {
                if (other.values_[0] < values_[0])
                {
                        values_[0] = other.values_[0];
                }
                if (other.values_[1] < values_[1])
                {
                        values_[1] = other.values_[1];
                }
        }

        /**
         * Ceiling point operator
         * @param other
         */
        inline void makeCeil(const Vector2 &other)
        {
                if (other.values_[0] > values_[0])
                {
                        values_[0] = other.values_[0];
                }
                if (other.values_[1] > values_[1])
                {
                        values_[1] = other.values_[1];
                }
        }

        /**
         * Returns the square of the length of the vector
         * @return square of the length of the vector
         */
        inline double computeSquaredLength() const
        {
                return math::Square(values_[0]) + math::Square(values_[1]);
        }

        /**
         * Returns the length of the vector (x and y).
         * @return length of the vector
         */
        inline double computeLength() const
        {
                return sqrt(computeSquaredLength());
        }

        /**
         * Returns the square distance to the given vector
         * @returns square distance to the given vector
         */
        inline double computeSquaredDistance(const Vector2 &other) const
        {
                return (*this - other).computeSquaredLength();
        }

        /**
         * Gets the distance to the other vector2
         * @param other
         * @return distance to other vector2
         */
        inline double Distance(const Vector2 &other) const
        {
                return sqrt(computeSquaredDistance(other));
        }

public:
        /**
         * In place Vector2 addition.
         */
        inline void operator+=(const Vector2 &other)
        {
                values_[0] += other.values_[0];
                values_[1] += other.values_[1];
        }

        /**
         * In place Vector2 subtraction.
         */
        inline void operator-=(const Vector2 &other)
        {
                values_[0] -= other.values_[0];
                values_[1] -= other.values_[1];
        }

        /**
         * Addition operator
         * @param other
         * @return vector resulting from adding this vector with the given vector
         */
        inline const Vector2 operator+(const Vector2 &other) const
        {
                return Vector2(values_[0] + other.values_[0], values_[1] + other.values_[1]);
        }

        /**
         * Subtraction operator
         * @param other
         * @return vector resulting from subtracting this vector from the given vector
         */
        inline const Vector2 operator-(const Vector2 &other) const
        {
                return Vector2(values_[0] - other.values_[0], values_[1] - other.values_[1]);
        }

        /**
         * In place scalar division operator
         * @param scalar
         */
        inline void operator/=(T scalar)
        {
                values_[0] /= scalar;
                values_[1] /= scalar;
        }

        /**
         * Divides a Vector2
         * @param scalar
         * @return scalar product
         */
        inline const Vector2 operator/(T scalar) const
        {
                return Vector2(values_[0] / scalar, values_[1] / scalar);
        }

        /**
         * Computes the dot product between the two vectors
         * @param other
         * @return dot product
         */
        inline double operator*(const Vector2 &other) const
        {
                return values_[0] * other.values_[0] + values_[1] * other.values_[1];
        }

        /**
         * Scales the vector by the given scalar
         * @param scalar
         */
        inline const Vector2 operator*(T scalar) const
        {
                return Vector2(values_[0] * scalar, values_[1] * scalar);
        }

        /**
         * Subtract the vector by the given scalar
         * @param scalar
         */
        inline const Vector2 operator-(T scalar) const
        {
                return Vector2(values_[0] - scalar, values_[1] - scalar);
        }

        /**
         * In place scalar multiplication operator
         * @param scalar
         */
        inline void operator*=(T scalar)
        {
                values_[0] *= scalar;
                values_[1] *= scalar;
        }

        /**
         * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
         * @param other
         */
        inline bool operator==(const Vector2 &other) const
        {
                return values_[0] == other.values_[0] && values_[1] == other.values_[1];
        }

        /**
         * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
         * @param other
         */
        inline bool operator!=(const Vector2 &other) const
        {
                return values_[0] != other.values_[0] || values_[1] != other.values_[1];
        }

        /**
         * Less than operator
         * @param other
         * @return true if left vector is less than right vector
         */
        inline bool operator<(const Vector2 &other) const
        {
                if (values_[0] < other.values_[0])
                {
                        return true;
                }
                else if (values_[0] > other.values_[0])
                {
                        return false;
                }
                else
                {
                        return values_[1] < other.values_[1];
                }
        }

        /**
         * Write Vector2 onto output stream
         * @param stream output stream
         * @param vector to write
         */
        friend inline std::ostream &operator<<(std::ostream &stream, const Vector2 &vector)
        {
                stream << vector.getX() << " " << vector.getY();
                return stream;
        }

private:
        T values_[2];
}; // Vector2<T>

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Defines a position (x, y) in 2-dimensional space and heading.
 */
class Pose2
{
public:
        /**
         * Default Constructor
         */
        Pose2()
        : heading_(0.0)
        {
        }

        /**
         * Constructor initializing pose parameters
         * @param position position
         * @param heading heading
         **/
        Pose2(const Vector2<double> &position, double heading)
            : position_(position),
              heading_(heading)
        {
        }

        /**
         * Constructor initializing pose parameters
         * @param x x-coordinate
         * @param y y-coordinate
         * @param heading heading
         **/
        Pose2(double x, double y, double heading)
        : position_(x, y),
          heading_(heading)
        {
        }

        /**
         * Copy constructor
         */
        Pose2(const Pose2 &other)
        : position_(other.position_),
          heading_(other.heading_)
        {
        }

public:
        /**
         * Returns the x-coordinate
         * @return the x-coordinate of the pose
         */
        inline double getX() const
        {
                return position_.getX();
        }

        /**
         * Sets the x-coordinate
         * @param x the x-coordinate of the pose
         */
        inline void setX(double x)
        {
                position_.setX(x);
        }

        /**
         * Returns the y-coordinate
         * @return the y-coordinate of the pose
         */
        inline double getY() const
        {
                return position_.getY();
        }

        /**
         * Sets the y-coordinate
         * @param y the y-coordinate of the pose
         */
        inline void setY(double y)
        {
                position_.setY(y);
        }

        /**
         * Returns the position
         * @return the position of the pose
         */
        inline const Vector2<double> &getPosition() const
        {
                return position_;
        }

        /**
         * Sets the position
         * @param position of the pose
         */
        inline void setPosition(const Vector2<double> &position)
        {
                position_ = position;
        }

        /**
         * Returns the heading of the pose (in radians)
         * @return the heading of the pose
         */
        inline double getHeading() const
        {
                return heading_;
        }

        /**
         * Sets the heading
         * @param heading of the pose
         */
        inline void setHeading(double heading)
        {
                heading_ = heading;
        }

        /**
         * Return the squared distance between two Pose2
         * @return squared distance
         */
        inline double computeSquaredDistance(const Pose2 &other) const
        {
                return position_.computeSquaredDistance(other.position_);
        }

public:
        /**
         * Assignment operator
         */
        inline Pose2 &operator=(const Pose2 &other)
        {
                position_ = other.position_;
                heading_ = other.heading_;

                return *this;
        }

        /**
         * Equality operator
         */
        inline bool operator==(const Pose2 &other) const
        {
                return position_ == other.position_ && heading_ == other.heading_;
        }

        /**
         * Inequality operator
         */
        inline bool operator!=(const Pose2 &other) const
        {
                return position_ != other.position_ || heading_ != other.heading_;
        }

        /**
         * In place Pose2 add.
         */
        inline void operator+=(const Pose2 &other)
        {
                position_ += other.position_;
                heading_ = math::NormalizeAngle(heading_ + other.heading_);
        }

        /**
         * Binary Pose2 add
         * @param other
         * @return Pose2 sum
         */
        inline Pose2 operator+(const Pose2 &other) const
        {
                return Pose2(position_ + other.position_,
                             math::NormalizeAngle(heading_ + other.heading_));
        }

        /**
         * Binary Pose2 subtract
         * @param other
         * @return Pose2 difference
         */
        inline Pose2 operator-(const Pose2 &other) const
        {
                return Pose2(position_ - other.position_,
                             math::NormalizeAngle(heading_ - other.heading_));
        }

        /**
         * Write this pose onto output stream
         * @param stream output stream
         * @param pose to read
         */
        friend inline std::ostream &operator<<(std::ostream &stream, const Pose2 &pose)
        {
                stream << pose.position_.getX() << " " << pose.position_.getY() << " " << pose.heading_;
                return stream;
        }

private:
        Vector2<double> position_;

        double heading_;
}; // Pose2

/**
 * Type declaration of Pose2 vector
 */
typedef std::vector<Pose2> Pose2Vector;

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Defines a Matrix 3 x 3 class.
 */
class Matrix3
{
public:
        /**
         * Default constructor
         */
        Matrix3()
        {
                clear();
        }

        /**
         * Copy constructor
         */
        inline Matrix3(const Matrix3 &other)
        {
                memcpy(matrix_, other.matrix_, 9 * sizeof(double));
        }

public:
        /**
         * Sets this matrix to identity matrix
         */
        void setToIdentity()
        {
                memset(matrix_, 0, 9 * sizeof(double));

                for (int32_t i = 0; i < 3; i++)
                {
                        matrix_[i][i] = 1.0;
                }
        }

        /**
         * Sets this matrix to zero matrix
         */
        void clear()
        {
                memset(matrix_, 0, 9 * sizeof(double));
        }

        /**
         * Sets this matrix to be the rotation matrix of rotation around given axis
         * @param x x-coordinate of axis
         * @param y y-coordinate of axis
         * @param z z-coordinate of axis
         * @param radians amount of rotation
         */
        void fromAxisAngle(double x, double y, double z, const double radians)
        {
                double cosRadians = cos(radians);
                double sinRadians = sin(radians);
                double oneMinusCos = 1.0 - cosRadians;

                double xx = x * x;
                double yy = y * y;
                double zz = z * z;

                double xyMCos = x * y * oneMinusCos;
                double xzMCos = x * z * oneMinusCos;
                double yzMCos = y * z * oneMinusCos;

                double xSin = x * sinRadians;
                double ySin = y * sinRadians;
                double zSin = z * sinRadians;

                matrix_[0][0] = xx * oneMinusCos + cosRadians;
                matrix_[0][1] = xyMCos - zSin;
                matrix_[0][2] = xzMCos + ySin;

                matrix_[1][0] = xyMCos + zSin;
                matrix_[1][1] = yy * oneMinusCos + cosRadians;
                matrix_[1][2] = yzMCos - xSin;

                matrix_[2][0] = xzMCos - ySin;
                matrix_[2][1] = yzMCos + xSin;
                matrix_[2][2] = zz * oneMinusCos + cosRadians;
        }

        /**
         * Returns transposed version of this matrix
         * @return transposed matrix
         */
        Matrix3 transpose() const
        {
                Matrix3 transpose;

                for (uint32_t row = 0; row < 3; row++)
                {
                        for (uint32_t col = 0; col < 3; col++)
                        {
                                transpose.matrix_[row][col] = matrix_[col][row];
                        }
                }

                return transpose;
        }

        /**
         * Returns the inverse of the matrix
         */
        Matrix3 inverse() const
        {
                Matrix3 k_inverse = *this;
                bool have_inverse = computeInverseFast(k_inverse, 1e-14);
                if (have_inverse == false)
                {
                        assert(false);
                }
                return k_inverse;
        }

        /**
         * Internal helper method for inverse matrix calculation
         * This code is lifted from the OgreMatrix3 class!!
         */
        bool computeInverseFast(Matrix3 &k_inverse, double tolerance = KT_TOLERANCE) const
        {
                // Invert a 3x3 using cofactors.  This is about 8 times faster than
                // the Numerical Recipes code which uses Gaussian elimination.
                k_inverse.matrix_[0][0] = matrix_[1][1] * matrix_[2][2] - matrix_[1][2] * matrix_[2][1];
                k_inverse.matrix_[0][1] = matrix_[0][2] * matrix_[2][1] - matrix_[0][1] * matrix_[2][2];
                k_inverse.matrix_[0][2] = matrix_[0][1] * matrix_[1][2] - matrix_[0][2] * matrix_[1][1];
                k_inverse.matrix_[1][0] = matrix_[1][2] * matrix_[2][0] - matrix_[1][0] * matrix_[2][2];
                k_inverse.matrix_[1][1] = matrix_[0][0] * matrix_[2][2] - matrix_[0][2] * matrix_[2][0];
                k_inverse.matrix_[1][2] = matrix_[0][2] * matrix_[1][0] - matrix_[0][0] * matrix_[1][2];
                k_inverse.matrix_[2][0] = matrix_[1][0] * matrix_[2][1] - matrix_[1][1] * matrix_[2][0];
                k_inverse.matrix_[2][1] = matrix_[0][1] * matrix_[2][0] - matrix_[0][0] * matrix_[2][1];
                k_inverse.matrix_[2][2] = matrix_[0][0] * matrix_[1][1] - matrix_[0][1] * matrix_[1][0];

                double f_det = matrix_[0][0] * k_inverse.matrix_[0][0] +
                              matrix_[0][1] * k_inverse.matrix_[1][0] +
                              matrix_[0][2] * k_inverse.matrix_[2][0];

                if (fabs(f_det) <= tolerance)
                {
                        return false;
                }

                double f_inv_det = 1.0 / f_det;
                for (size_t row = 0; row < 3; row++)
                {
                        for (size_t col = 0; col < 3; col++)
                        {
                                k_inverse.matrix_[row][col] *= f_inv_det;
                        }
                }

                return true;
        }

        /**
         * Returns a string representation of this matrix
         * @return string representation of this matrix
         */
        inline std::string toString() const
        {
                std::stringstream converter;
                converter.precision(std::numeric_limits<double>::digits10);

                for (int row = 0; row < 3; row++)
                {
                        for (int col = 0; col < 3; col++)
                        {
                                converter << matrix_[row][col] << " ";
                        }
                }

                return converter.str();
        }

public:
        /**
         * Assignment operator
         */
        inline Matrix3 &operator=(const Matrix3 &other)
        {
                memcpy(matrix_, other.matrix_, 9 * sizeof(double));
                return *this;
        }

        /**
         * Matrix element access, allows use of construct mat(r, c)
         * @param row
         * @param column
         * @return reference to mat(r,c)
         */
        inline double &operator()(uint32_t row, uint32_t column)
        {
                return matrix_[row][column];
        }

        /**
         * Read-only matrix element access, allows use of construct mat(r, c)
         * @param row
         * @param column
         * @return mat(r,c)
         */
        inline double operator()(uint32_t row, uint32_t column) const
        {
                return matrix_[row][column];
        }

        /**
         * Binary Matrix3 multiplication.
         * @param other
         * @return Matrix3 product
         */
        Matrix3 operator*(const Matrix3 &other) const
        {
                Matrix3 product;

                for (size_t row = 0; row < 3; row++)
                {
                        for (size_t col = 0; col < 3; col++)
                        {
                                product.matrix_[row][col] = matrix_[row][0] * other.matrix_[0][col] +
                                                             matrix_[row][1] * other.matrix_[1][col] +
                                                             matrix_[row][2] * other.matrix_[2][col];
                        }
                }

                return product;
        }

        /**
         * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
         * @param pose2
         * @return Pose2 product
         */
        inline Pose2 operator*(const Pose2 &pose2) const
        {
                Pose2 pose_holder;

                pose_holder.setX(matrix_[0][0] * pose2.getX() + matrix_[0][1] * pose2.getY() + matrix_[0][2] * pose2.getHeading());
                pose_holder.setY(matrix_[1][0] * pose2.getX() + matrix_[1][1] * pose2.getY() + matrix_[1][2] * pose2.getHeading());
                pose_holder.setHeading(matrix_[2][0] * pose2.getX() + matrix_[2][1] * pose2.getY() + matrix_[2][2] * pose2.getHeading());

                return pose_holder;
        }

        /**
         * In place Matrix3 add.
         * @param k_matrix
         */
        inline void operator+=(const Matrix3 &k_matrix)
        {
                for (uint32_t row = 0; row < 3; row++)
                {
                        for (uint32_t col = 0; col < 3; col++)
                        {
                                matrix_[row][col] += k_matrix.matrix_[row][col];
                        }
                }
        }

        /**
         * Write Matrix3 onto output stream
         * @param stream output stream
         * @param matrix to write
         */
        friend inline std::ostream &operator<<(std::ostream &stream, const Matrix3 &matrix)
        {
                stream << matrix.toString();
                return stream;
        }

private:
        double matrix_[3][3];
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Implementation of a Pose2 transform
 */
class Transform
{
public:
        /**
         * Constructs a transformation from the origin to the given pose
         * @param pose pose
         */
        Transform(const Pose2 &pose)
        {
                setTransform(Pose2(), pose);
        }

        /**
         * Constructs a transformation from the first pose to the second pose
         * @param pose1 first pose
         * @param pose2 second pose
         */
        Transform(const Pose2 &pose1, const Pose2 &pose2)
        {
                setTransform(pose1, pose2);
        }

public:
        /**
         * Transforms the pose according to this transform
         * @param source_pose pose to transform from
         * @return transformed pose
         */
        inline Pose2 transformPose(const Pose2 &source_pose)
        {
                Pose2 new_position = transform_ + rotation_ * source_pose;
                double angle = math::NormalizeAngle(source_pose.getHeading() + transform_.getHeading());

                return Pose2(new_position.getPosition(), angle);
        }

        /**
         * Inverse transformation of the pose according to this transform
         * @param source_pose pose to transform from
         * @return transformed pose
         */
        inline Pose2 inverseTransformPose(const Pose2 &source_pose)
        {
                Pose2 new_position = inverse_rotation_ * (source_pose - transform_);
                double angle = math::NormalizeAngle(source_pose.getHeading() - transform_.getHeading());

                // components of transform
                return Pose2(new_position.getPosition(), angle);
        }

private:
        /**
         * Sets this to be the transformation from the first pose to the second pose
         * @param pose1 first pose
         * @param pose2 second pose
         */
        void setTransform(const Pose2 &pose1, const Pose2 &pose2)
        {
                if (pose1 == pose2)
                {
                        rotation_.setToIdentity();
                        inverse_rotation_.setToIdentity();
                        transform_ = Pose2();
                        return;
                }

                // heading transformation
                rotation_.fromAxisAngle(0, 0, 1, pose2.getHeading() - pose1.getHeading());
                inverse_rotation_.fromAxisAngle(0, 0, 1, pose1.getHeading() - pose2.getHeading());

                // position transformation
                Pose2 new_position;
                if (pose1.getX() != 0.0 || pose1.getY() != 0.0)
                {
                        new_position = pose2 - rotation_ * pose1;
                }
                else
                {
                        new_position = pose2;
                }

                transform_ = Pose2(new_position.getPosition(), pose2.getHeading() - pose1.getHeading());
        }

private:
        // pose transformation
        Pose2 transform_;

        Matrix3 rotation_;
        Matrix3 inverse_rotation_;
}; // Transform

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

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
         * Copy constructor
         * @param other
         */
        Size2(const Size2 &other)
            : width_(other.width_),
              height_(other.height_)
        {
        }

public:
        /**
         * Gets the width
         * @return the width
         */
        inline const T getWidth() const
        {
                return width_;
        }

        /**
         * Sets the width
         * @param width
         */
        inline void setWidth(T width)
        {
                width_ = width;
        }

        /**
         * Gets the height
         * @return the height
         */
        inline const T getHeight() const
        {
                return height_;
        }

        /**
         * Sets the height
         * @param height
         */
        inline void setHeight(T height)
        {
                height_ = height;
        }

        /**
         * Assignment operator
         */
        inline Size2 &operator=(const Size2 &other)
        {
                width_ = other.width_;
                height_ = other.height_;

                return *this;
        }

        /**
         * Equality operator
         */
        inline bool operator==(const Size2 &other) const
        {
                return width_ == other.width_ && height_ == other.height_;
        }

        /**
         * Inequality operator
         */
        inline bool operator!=(const Size2 &other) const
        {
                return width_ != other.width_ || height_ != other.height_;
        }

        /**
         * Write Size2 onto output stream
         * @param stream output stream
         * @param size to write
         */
        friend inline std::ostream &operator<<(std::ostream &stream, const Size2 &size)
        {
                stream << "(" << size.width_ << ", " << size.height_ << ")";
                return stream;
        }

private:
        T width_;
        T height_;
}; // Size2

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Defines a bounding box in 2-dimensional real space.
 */
class BoundingBox2
{
public:
        /*
         * Default constructor
         */
        BoundingBox2()
            : minimum_(999999999999999999.99999, 999999999999999999.99999),
              maximum_(-999999999999999999.99999, -999999999999999999.99999)
        {
        }

public:
        /**
         * Get bounding box minimum
         */
        inline const Vector2<double> &getMinimum() const
        {
                return minimum_;
        }

        /**
         * Set bounding box minimum
         */
        inline void setMinimum(const Vector2<double> &mMinimum)
        {
                minimum_ = mMinimum;
        }

        /**
         * Get bounding box maximum
         */
        inline const Vector2<double> &getMaximum() const
        {
                return maximum_;
        }

        /**
         * Set bounding box maximum
         */
        inline void setMaximum(const Vector2<double> &rMaximum)
        {
                maximum_ = rMaximum;
        }

        /**
         * Get the size of the bounding box
         */
        inline Size2<double> getSize() const
        {
                Vector2<double> size = maximum_ - minimum_;

                return Size2<double>(size.getX(), size.getY());
        }

        /**
         * Add vector to bounding box
         */
        inline void add(const Vector2<double> &point)
        {
                minimum_.makeFloor(point);
                maximum_.makeCeil(point);
        }

        /**
         * Add other bounding box to bounding box
         */
        inline void add(const BoundingBox2 &rBoundingBox)
        {
                add(rBoundingBox.getMinimum());
                add(rBoundingBox.getMaximum());
        }

        /**
         * Whether the given point is in the bounds of this box
         * @param point
         * @return in bounds?
         */
        inline bool isInBounds(const Vector2<double> &point) const
        {
                return math::InRange(point.getX(), minimum_.getX(), maximum_.getX()) &&
                       math::InRange(point.getY(), minimum_.getY(), maximum_.getY());
        }
private:
        Vector2<double> minimum_;
        Vector2<double> maximum_;
}; // BoundingBox2

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Stores x, y, width and height that represents the location and size of a rectangle
 * (x, y) is at bottom left in mapper!
 */
template <typename T>
class Rectangle2
{
public:
        /**
         * Default constructor
         */
        Rectangle2()
        {
        }

        /**
         * Constructor initializing rectangle parameters
         * @param x x-coordinate of left edge of rectangle
         * @param y y-coordinate of bottom edge of rectangle
         * @param width width of rectangle
         * @param height height of rectangle
         */
        Rectangle2(T x, T y, T width, T height)
            : position_(x, y),
              size_(width, height)
        {
        }

        /**
         * Constructor initializing rectangle parameters
         * @param position (x,y)-coordinate of rectangle
         * @param size Size of the rectangle
         */
        Rectangle2(const Vector2<T> &position, const Size2<T> &size)
            : position_(position),
              size_(size)
        {
        }

        /**
         * Copy constructor
         */
        Rectangle2(const Rectangle2 &other)
            : position_(other.position_),
              size_(other.size_)
        {
        }

public:
        /**
         * Gets the x-coordinate of the left edge of this rectangle
         * @return the x-coordinate of the left edge of this rectangle
         */
        inline T getX() const
        {
                return position_.getX();
        }

        /**
         * Sets the x-coordinate of the left edge of this rectangle
         * @param x the x-coordinate of the left edge of this rectangle
         */
        inline void setX(T x)
        {
                position_.setX(x);
        }

        /**
         * Gets the y-coordinate of the bottom edge of this rectangle
         * @return the y-coordinate of the bottom edge of this rectangle
         */
        inline T getY() const
        {
                return position_.getY();
        }

        /**
         * Sets the y-coordinate of the bottom edge of this rectangle
         * @param y the y-coordinate of the bottom edge of this rectangle
         */
        inline void setY(T y)
        {
                position_.setY(y);
        }

        /**
         * Gets the width of this rectangle
         * @return the width of this rectangle
         */
        inline T getWidth() const
        {
                return size_.getWidth();
        }

        /**
         * Sets the width of this rectangle
         * @param width the width of this rectangle
         */
        inline void setWidth(T width)
        {
                size_.setWidth(width);
        }

        /**
         * Gets the height of this rectangle
         * @return the height of this rectangle
         */
        inline T getHeight() const
        {
                return size_.getHeight();
        }

        /**
         * Sets the height of this rectangle
         * @param height the height of this rectangle
         */
        inline void setHeight(T height)
        {
                size_.setHeight(height);
        }

        /**
         * Gets the position of this rectangle
         * @return the position of this rectangle
         */
        inline const Vector2<T> &getPosition() const
        {
                return position_;
        }

        /**
         * Sets the position of this rectangle
         * @param x x
         * @param y y
         */
        inline void setPosition(const T &x, const T &y)
        {
                position_ = Vector2<T>(x, y);
        }

        /**
         * Sets the position of this rectangle
         * @param position position
         */
        inline void setPosition(const Vector2<T> &position)
        {
                position_ = position;
        }

        /**
         * Gets the size of this rectangle
         * @return the size of this rectangle
         */
        inline const Size2<T> &getSize() const
        {
                return size_;
        }

        /**
         * Sets the size of this rectangle
         * @param size size
         */
        inline void setSize(const Size2<T> &size)
        {
                size_ = size;
        }

        /**
         * Gets the center of this rectangle
         * @return the center of this rectangle
         */
        inline const Vector2<T> getCenter() const
        {
                return Vector2<T>(position_.getX() + size_.getWidth() * 0.5,
                                  position_.getY() + size_.getHeight() * 0.5);
        }

public:
        /**
         * Assignment operator
         */
        Rectangle2 &operator=(const Rectangle2 &other)
        {
                position_ = other.position_;
                size_ = other.size_;

                return *this;
        }

        /**
         * Equality operator
         */
        inline double operator==(const Rectangle2 &other) const
        {
                return position_ == other.position_ && size_ == other.size_;
        }

        /**
         * Inequality operator
         */
        inline double operator!=(const Rectangle2 &other) const
        {
                return position_ != other.position_ || size_ != other.size_;
        }

private:
        Vector2<T> position_;
        Size2<T> size_;
}; // Rectangle2

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized range scan relative to the robot.
 * The user can set an offset pose for the sensor relative to the robot coordinate system. If no value is provided
 * by the user, the sensor is set to be at the origin of the robot coordinate system.
 * The LaserRangeFinder contains parameters for physical laser sensor used by the mapper for scan matching
 * Also contains information about the maximum range of the sensor and provides a threshold
 * for limiting the range of readings.
 * The optimal value for the range threshold depends on the angular resolution of the scan and
 * the desired map resolution.  RangeThreshold should be set as large as possible while still
 * providing "solid" coverage between consecutive range readings.  The diagram below illustrates
 * the relationship between map resolution and the range threshold.
 */
class LaserRangeFinder
{
private:
        // sensor parameters
        double range_threshold_;
        double minimum_range_;
        double maximum_range_;

        bool is_360_laser_;
        double minimum_angle_;
        double maximum_angle_;
        double angular_resolution_;
        uint32_t number_of_range_readings_;

        // pose_robot_laser
        Pose2 offset_pose_;
        // name of laser frame
        std::string frame_id_;  

public:
        LaserRangeFinder()
        {
        }

public:
        inline void setFrameId(std::string frame_id)
        {
                frame_id_ = frame_id;
        }

        inline void setIs360Laser(bool is_360_laser)
        {
                is_360_laser_ = is_360_laser;

                update();
        }

        inline bool getIs360Laser() const
        {
                return is_360_laser_;
        }

        inline void setMinimumRange(double minimum_range)
        {
                minimum_range_ = minimum_range;
        }

        inline double getMinimumRange() const
        {
                return minimum_range_;
        }

        inline void setMaximumRange(double maximum_range)
        {
                maximum_range_ = maximum_range;
        }

        inline double getMaximumRange() const
        {
                return maximum_range_;
        }

        inline double getRangeThreshold() const
        {
                return range_threshold_;
        }

        inline void setRangeThreshold(double range_threshold)
        {
                // make sure rangeThreshold is within laser range finder range
                range_threshold_ = math::Clip(range_threshold, getMinimumRange(), getMaximumRange());

                if (math::DoubleEqual(getRangeThreshold(), range_threshold) == false) {
                        std::cout << "Info: clipped range threshold to be within minimum and maximum range!" << std::endl;
                }
        }

        inline void setMaximumAngle(double maximum_angle)
        {
                maximum_angle_ = maximum_angle;
                update();
        }

        inline double getMaximumAngle()
        {
                return maximum_angle_;
        }

        inline void setMinimumAngle(double minimum_angle)
        {
                minimum_angle_ = minimum_angle;
                update();
        }

        inline double getMinimumAngle() const
        {
                return minimum_angle_;
        }

        inline void setAngularResolution(double angular_resolution)
        {
                angular_resolution_ = angular_resolution;
                update();
        }

        inline double getAngularResolution() const
        {
                return angular_resolution_;
        }

        inline uint32_t getNumberOfRangeReadings() const
        {
                return number_of_range_readings_;
        }

        inline void setOffsetPose(const Pose2 &offset_pose)
        {
                offset_pose_ = offset_pose;
        }

        inline const Pose2 &getOffsetPose() const
        {
                return offset_pose_;
        }

        /**
         * Set the number of range readings based on the minimum and
         * maximum angles of the sensor and the angular resolution
         */
        void update()
        {
                int residual = 1;
                if (getIs360Laser()) {
                        // residual is 0 by 360 lidar conventions
                        residual = 0;
                }

                number_of_range_readings_ = static_cast<uint32_t>(
                        math::Round((getMaximumAngle() -
                                     getMinimumAngle()) /
                                     getAngularResolution()) +
                                     residual);
        }
}; // LaserRangeFinder

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Type declaration of range readings vector
 */
typedef std::vector<double> RangeReadingsVector;

/**
 * The LocalizedRangeScan contains range data from a single sweep of a laser range finder sensor
 * in a two-dimensional space and position information. The odometer position is the position
 * reported by the robot when the range data was recorded. The corrected position is the position
 * calculated by the mapper (or localizer)
 */
class LocalizedRangeScan
{
private:
        /**
         * Corrected pose of robot calculated by mapper (or localizer)
         */
        Pose2 corrected_pose_;

        /**
         * Odometric pose of robot
         */

        Pose2 odom_pose_;
        
        /**
         * Average of all the point readings
         */
        Pose2 barycenter_pose_;

        /**
         * Bounding box of localized range scan
         */
        BoundingBox2 bounding_box_;
        
        /**
         * Internal flag used to update point readings, barycenter and bounding box
         */
        bool is_dirty_;

        /**
         * ID unique across all sensor data
         */
        int32_t scan_id_;

        /**
         * Time the sensor data was created
         */
        double time_;

        /**
         * Laser that created this sensor data
         */
        LaserRangeFinder *laser_;

        std::unique_ptr<double[]> range_readings_;
        uint32_t number_of_range_readings_;
        std::vector<Vector2<double>> point_readings_;
        std::vector<Vector2<double>> unfiltered_point_readings_;
        mutable boost::shared_mutex lock_;

public:
        LocalizedRangeScan()
        {
        }

        LocalizedRangeScan(LaserRangeFinder *laser, const RangeReadingsVector &range_readings)
        : is_dirty_(true), 
          laser_(laser)
        {
                if (!range_readings.empty()) {
                        number_of_range_readings_ = range_readings.size();
                        range_readings_ = std::make_unique<double[]>(number_of_range_readings_);

                        uint32_t index = 0;
                        for (const auto &reading : range_readings) {
                                range_readings_[index++] = reading;
                        }
                }
        }

public:
        inline void setScanId(int32_t scan_id)
        {
                scan_id_ = scan_id;
        }

        inline int32_t getScanId()
        {
                return scan_id_;
        }

        inline void setOdometricPose(const Pose2 &pose)
        {
                odom_pose_ = pose;
        }

        inline const Pose2 &getOdometricPose() const
        {
                return odom_pose_;
        }

        inline void setCorrectedPose(const Pose2 &pose)
        {
                corrected_pose_ = pose;
        }

        inline const Pose2 &getCorrectedPose() const
        {
                return corrected_pose_;
        }

        void setSensorPose(const Pose2 &scan_pose)
        {
                corrected_pose_ = getCorrectedAt(scan_pose);
                update();
        }

        /**
         * @brief Computes the pose of the robot if the sensor were at the given pose
         * @param sensor_pose sensor pose
         * @return robot pose
         */
        inline Pose2 getCorrectedAt(const Pose2 &sensor_pose) const
        {
                Pose2 deviceOffsetPose2 = getLaserRangeFinder()->getOffsetPose();
                double offsetLength = deviceOffsetPose2.getPosition().computeLength();
                double offsetHeading = deviceOffsetPose2.getHeading();
                double angleoffset = atan2(deviceOffsetPose2.getY(), deviceOffsetPose2.getX());
                double correctedHeading = math::NormalizeAngle(sensor_pose.getHeading());
                Pose2 worldSensorOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                                                offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                                                offsetHeading);

                return sensor_pose - worldSensorOffset;
        }

        inline void setTime(double time)
        {
                time_ = time;
        }

        inline double getTime() const
        {
                return time_;
        }

        inline LaserRangeFinder *getLaserRangeFinder() const
        {
                return laser_;
        }

        /**
         * Get point readings in local coordinates
         */
        inline const std::vector<Vector2<double>> &getPointReadings(bool want_filtered = false) const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);
                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> uniqueLock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                if (want_filtered == true) {
                        return point_readings_;
                } else {
                        return unfiltered_point_readings_;
                }
        }

        /**
         * Computes the position of the sensor
         * @return scan pose
         */
        inline Pose2 getSensorPose() const
        {
                return getSensorAt(corrected_pose_);
        }

        /**
         * Computes the position of the sensor if the robot were at the given pose
         * @param robot_pose
         * @return sensor pose
         */
        inline Pose2 getSensorAt(const Pose2 &robot_pose) const
        {
                return Transform(robot_pose).transformPose(getLaserRangeFinder()->getOffsetPose());
        }

        /**
         * Gets the range readings of this scan
         * @return range readings of this scan
         */
        inline const double *getRangeReadings() const
        {
                return range_readings_.get();
        }

        inline RangeReadingsVector getRangeReadingsVector() const
        {
                return RangeReadingsVector(range_readings_.get(), range_readings_.get() + number_of_range_readings_);
        }

        /**
         * Gets the bounding box of this scan
         * @return bounding box of this scan
         */
        inline const BoundingBox2 &getBoundingBox() const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);

                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> unique_lock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                return bounding_box_;
        }

        inline uint32_t getNumberOfRangeReadings() const
        {
                return number_of_range_readings_;
        }

        /**
         * Moves the scan by moving the robot pose to the given location and update point readings.
         * @param pose new pose of the robot of this scan
         */
        inline void setCorrectedPoseAndUpdate(const Pose2 &pose)
        {
                setCorrectedPose(pose);
                update();
        }

        /**
         * Gets barycenter of point readings
         */
        inline const Pose2 &getBarycenterPose() const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);
                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> unique_lock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                return barycenter_pose_;
        }

        /**
         * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
         * @param useBarycenter
         * @return barycenter if parameter is true, otherwise scanner pose
         */
        inline Pose2 getReferencePose(bool use_barycenter) const
        {
                boost::shared_lock<boost::shared_mutex> lock(lock_);
                if (is_dirty_) {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> unique_lock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                return use_barycenter ? getBarycenterPose() : getSensorPose();
        }

private:
        /**
         * Compute point readings based on range readings
         * Only range readings within [minimum range; range threshold] are returned
         */
        void update()
        {
                if (laser_ != nullptr) {
                        point_readings_.clear();
                        unfiltered_point_readings_.clear();

                        double range_threshold = laser_->getRangeThreshold();
                        double minimum_angle = laser_->getMinimumAngle();
                        double angular_resolution = laser_->getAngularResolution();
                        Pose2 scan_pose = getSensorPose();
                        // compute point readings
                        Vector2<double> range_points_sum(0, 0);
                        uint32_t beam_num = 0;

                        for (uint32_t i = 0; i < laser_->getNumberOfRangeReadings(); i++, beam_num++) {
                                double range_reading = getRangeReadings()[i];
                                double angle = scan_pose.getHeading() + minimum_angle + beam_num * angular_resolution;
                                Vector2<double> point;
                                point.setX(scan_pose.getX() + (range_reading * cos(angle)));
                                point.setY(scan_pose.getY() + (range_reading * sin(angle)));

                                if (!math::InRange(range_reading, laser_->getMinimumRange(), range_threshold)) {
                                        unfiltered_point_readings_.push_back(point);
                                        continue;
                                }

                                point_readings_.push_back(point);
                                unfiltered_point_readings_.push_back(point);
                                range_points_sum += point;
                        }

                        // compute barycenter
                        double n_points = static_cast<double>(point_readings_.size());
                        if (n_points != 0.0) {
                                Vector2<double> average_position = Vector2<double>(range_points_sum / n_points);
                                barycenter_pose_ = Pose2(average_position, 0.0);
                        } else {
                                barycenter_pose_ = scan_pose;
                        }

                        // calculate bounding box of scan
                        bounding_box_ = BoundingBox2();
                        bounding_box_.add(scan_pose.getPosition());

                        for (const auto &point_reading : point_readings_) {
                                bounding_box_.add(point_reading);
                        }
                }

                is_dirty_ = false;
        }

private:
        /**
         * Restrict copy constructor
         */
        LocalizedRangeScan(const LocalizedRangeScan &);
        /**
         * Restrict assignment operator
         */
        const LocalizedRangeScan &operator=(const LocalizedRangeScan &);

}; // LocalizedRangeScan

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class LookupArray
{
private:
        std::unique_ptr<int32_t[]> array_;
        uint32_t capacity_;
        uint32_t size_;

public:
        /**
         * Constructs lookup array
         */
        LookupArray()
        : array_(nullptr), 
          capacity_(0), 
          size_(0)
        {
        }

public:
        /**
         * Clear array
         */
        void clear()
        {
                memset(array_.get(), 0, sizeof(int32_t) * capacity_);
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
                        if (array_ != nullptr)
                        {
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
         * Gets reference to value at given index
         * @param index
         * @return reference to value at index
         */
        inline int32_t &operator[](uint32_t index)
        {
                assert(index < size_);

                return array_[index];
        }

        /**
         * Gets value at given index
         * @param index
         * @return value at index
         */
        inline int32_t operator[](uint32_t index) const
        {
                assert(index < size_);

                return array_[index];
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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * The CoordinateConverter class is used to convert coordinates between world and grid coordinates
 * In world coordinates 1.0 = 1 meter where 1 in grid coordinates = 1 pixel!
 * Default scale for coordinate converter is 20 that converters to 1 pixel = 0.05 meter
 */
class CoordinateConverter
{
private:
        double scale_;
        Size2<int32_t> size_;
        Vector2<double> offset_;

public:
        CoordinateConverter()
        : scale_(20.0)
        {
        }

public:
        /**
         * Sets the size
         * @param size
         */
        inline void setSize(const Size2<int32_t> &size)
        {
                size_ = size;
        }

        /**
         * Sets the scale
         * @param scale
         */
        inline void setScale(double scale)
        {
                scale_ = scale;
        }

        inline double getResolution() const
        {
                return 1.0 / scale_;
        }

        /**
         * Gets the size
         * @return size
         */
        inline const Size2<int32_t> &getSize() const
        {
                return size_;
        }

        /**
         * Gets the offset
         * @return offset
         */
        inline const Vector2<double> &getOffset() const
        {
                return offset_;
        }

        /**
         * Sets the offset
         * @param rOffset
         */
        inline void setOffset(const Vector2<double> &offset)
        {
                offset_ = offset;
        }

        /**
         * Converts the point from world coordinates to grid coordinates
         * @param world world coordinate
         * @param flip_y
         * @return grid coordinate
         */
        inline Vector2<int32_t> convertWorldToGrid(
                const Vector2<double> &world,
                bool flip_y = false) const
        {
                double grid_x = (world.getX() - offset_.getX()) * scale_;
                double grid_y = 0.0;

                if (flip_y == false) {
                        grid_y = (world.getY() - offset_.getY()) * scale_;
                } else {
                        grid_y = (size_.getHeight() / scale_ - world.getY() + offset_.getY()) * scale_;
                }

                return Vector2<int32_t>(
                        static_cast<int32_t>(math::Round(grid_x)),
                        static_cast<int32_t>(math::Round(grid_y)));
        }

        /**
         * Converts the point from grid coordinates to world coordinates
         * @param grid world coordinate
         * @param flipY
         * @return world coordinate
         */
        inline Vector2<double> convertGridToWorld(
                const Vector2<int32_t> &grid,
                bool flipY = false) const
        {
                double worldX = offset_.getX() + grid.getX() / scale_;
                double worldY = 0.0;

                if (flipY == false) {
                        worldY = offset_.getY() + grid.getY() / scale_;
                } else {
                        worldY = offset_.getY() + (size_.getHeight() - grid.getY()) / scale_;
                }

                return Vector2<double>(worldX, worldY);
        }

        /**
         * Gets the bounding box
         * @return bounding box
         */
        inline BoundingBox2 getBoundingBox() const
        {
                BoundingBox2 box;

                double minX = getOffset().getX();
                double minY = getOffset().getY();
                double maxX = minX + getSize().getWidth() * getResolution();
                double maxY = minY + getSize().getHeight() * getResolution();

                box.setMinimum(getOffset());
                box.setMaximum(Vector2<double>(maxX, maxY));
                return box;
        }
}; // CoordinateConverter

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class OccupancyGrid;

class CellUpdater
{
public:
        CellUpdater(OccupancyGrid *grid)
        : occupancy_grid_(grid)
        {
        }

        /**
         * Updates the cell at the given index based on the grid's hits and pass counters
         * @param index
         */
        void operator()(uint32_t index);

private:
        OccupancyGrid *occupancy_grid_;
}; // CellUpdater

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

template <typename T>
class Grid
{
private:
        int32_t width_;                                             // width of grid
        int32_t height_;                                            // height of grid
        int32_t width_step_;                                        // 8 bit aligned width of grid
        std::unique_ptr<T[]> data_;                                 // grid data
        std::unique_ptr<CoordinateConverter> coordinate_converter_; // utility to convert between world coordinates and grid coordinates

protected:
        /**
         * Constructs grid of given size
         * @param width
         * @param height
         */
        Grid(int32_t width, int32_t height)
        : data_(nullptr),
          coordinate_converter_(nullptr)
        {
                resize(width, height);
        }

public:
        Grid()
        {
        }

        /**
         * Creates a grid of given size and resolution
         * @param width
         * @param height
         * @param resolution
         * @return grid pointer
         */
        static Grid *createGrid(int32_t width, int32_t height, double resolution)
        {
                Grid *grid = new Grid(width, height);

                grid->getCoordinateConverter()->setScale(1.0 / resolution);

                return grid;
        }

public:
        /**
         * Clear out the grid data
         */
        void clear()
        {
                memset(data_.get(), 0, getDataSize() * sizeof(T));
        }

        /**
         * Resizes the grid (deletes all old data)
         * @param width
         * @param height
         */
        virtual void resize(int32_t width, int32_t height)
        {
                width_ = width;
                height_ = height;
                width_step_ = math::AlignValue<int32_t>(width, 8);

                if (data_ != nullptr) {
                        data_.reset();
                }

                try
                {
                        data_ = std::make_unique<T[]>(getDataSize());

                        if (coordinate_converter_ == nullptr)
                        {
                                coordinate_converter_ = std::make_unique<CoordinateConverter>();
                        }
                        coordinate_converter_->setSize(Size2<int32_t>(width, height));
                }
                catch (...)
                {
                        data_.reset();
                        width_ = 0;
                        height_ = 0;
                        width_step_ = 0;
                }

                clear();
        }

        /**
         * Gets the grid data pointer
         * @return data pointer
         */
        inline T *getDataPointer()
        {
                return data_.get();
        }

        /**
         * Gets pointer to data at given grid coordinate
         * @param grid grid coordinate
         * @return grid point
         */
        T *getDataPointer(const Vector2<int32_t> &grid)
        {
                int32_t index = getGridIndex(grid, true);
                return data_.get() + index;
        }

        /**
         * Gets the width step in bytes
         * @return width step
         */
        inline int32_t getWidthStep() const
        {
                return width_step_;
        }

        /**
         * Gets the width of the grid
         * @return width of the grid
         */
        inline int32_t getWidth() const
        {
                return width_;
        }

        /**
         * Gets the height of the grid
         * @return height of the grid
         */
        inline int32_t getHeight() const
        {
                return height_;
        }

        inline double getResolution() const
        {
                return coordinate_converter_->getResolution();
        }

        /**
         * Get value at given grid coordinate
         * @param grid_coordinate grid coordinate
         * @return value
         */
        inline T getValue(const Vector2<int32_t> &grid_coordinate) const
        {
                int32_t index = getGridIndex(grid_coordinate);
                return data_[index];
        }

        /**
         * Increments all the grid cells from (x0, y0) to (x1, y1);
         * if applicable, apply f to each cell traced
         * @param x0
         * @param y0
         * @param x1
         * @param y1
         * @param cell_updater
         */
        void traceLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, CellUpdater *cell_updater = nullptr)
        {
                bool steep = abs(y1 - y0) > abs(x1 - x0);
                if (steep) {
                        std::swap(x0, y0);
                        std::swap(x1, y1);
                }
                if (x0 > x1) {
                        std::swap(x0, x1);
                        std::swap(y0, y1);
                }

                int32_t delta_x = x1 - x0;
                int32_t delta_y = abs(y1 - y0);
                int32_t error = 0;
                int32_t y_step;
                int32_t y = y0;

                if (y0 < y1) {
                        y_step = 1;
                } else {
                        y_step = -1;
                }

                int32_t point_x;
                int32_t point_y;
                for (int32_t x = x0; x <= x1; x++) {
                        if (steep) {
                                point_x = y;
                                point_y = x;
                        } else {
                                point_x = x;
                                point_y = y;
                        }

                        error += delta_y;

                        if (2 * error >= delta_x) {
                                y += y_step;
                                error -= delta_x;
                        }

                        Vector2<int32_t> grid_index(point_x, point_y);
                        if (isValidGridIndex(grid_index)) {
                                int32_t index = getGridIndex(grid_index, false);
                                T *grid_pointer = getDataPointer();
                                grid_pointer[index]++;

                                if (cell_updater != nullptr) {
                                        (*cell_updater)(index);
                                }
                        }
                }
        }

        /**
         * Checks whether the given coordinates are valid grid indices
         * @param grid
         */
        inline bool isValidGridIndex(const Vector2<int32_t> &grid) const
        {
                return (grid.getX() < width_ && grid.getY() < height_);
        }

        /**
         * Converts the point from world coordinates to grid coordinates
         * @param world world coordinate
         * @param flip_y
         * @return grid coordinate
         */
        inline Vector2<int32_t> convertWorldToGrid(
            const Vector2<double> &world,
            bool flip_y = false) const
        {
                return coordinate_converter_->convertWorldToGrid(world, flip_y);
        }

        /**
         * Gets the index into the data pointer of the given grid coordinate
         * @param grid
         * @param boundary_check default value is true
         * @return grid index
         */
        virtual int32_t getGridIndex(const Vector2<int32_t> &grid, bool boundary_check = true) const
        {
                if (boundary_check == true) {
                        if (isValidGridIndex(grid) == false) {
                                throw std::runtime_error("Grid index out of range");
                        }
                }

                int32_t index = grid.getX() + (grid.getY() * width_step_);

                if (boundary_check == true) {
                        assert(math::IsUpTo(index, getDataSize()));
                }

                return index;
        }

        /**
         * Gets the allocated grid size in bytes
         * @return data size
         */
        inline int32_t getDataSize() const
        {
                return width_step_ * height_;
        }

        /**
         * Gets the coordinate converter for this grid
         * @return coordinate converter
         */
        inline CoordinateConverter *getCoordinateConverter() const
        {
                return coordinate_converter_.get();
        }
}; // Grid

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

typedef std::vector<Vector2<double>> PointVectorDouble;
/**
 * Create lookup tables for point readings at varying angles in grid.
 * For each angle, grid indexes are calculated for each range reading.
 * This is to speed up finding best angle/position for a localized range scan
 *
 * Used heavily in mapper and localizer.
 *
 * In the localizer, this is a huge speed up for calculating possible position. For each particle,
 * a probability is calculated.  The range scan is the same, but all grid indexes at all possible angles are
 * calculated.  So when calculating the particle probability at a specific angle, the index table is used
 * to look up probability in probability grid!
 *
 */
template <typename T>
class GridIndexLookup
{
private:
        Grid<T> *grid_;

        uint32_t capacity_;
        uint32_t size_;

        std::vector<std::unique_ptr<LookupArray>> lookup_array_;

        // for sanity check
        std::vector<double> angles_;

public:
        GridIndexLookup()
        {
        }

        GridIndexLookup(Grid<T> *grid)
            : grid_(grid),
              capacity_(0),
              size_(0)
        {
        }

        /**
         * Gets the lookup array for a particular angle index
         * @param index
         * @return lookup array
         */
        const LookupArray *getLookupArray(uint32_t index) const
        {
                assert(math::IsUpTo(index, size_));

                return lookup_array_[index].get();
        }

        /**
         * Compute lookup table of the points of the given scan for the given angular space
         * @param scan the scan
         * @param angle_center
         * @param angle_offset computes lookup arrays for the angles within this offset around angleStart
         * @param angle_resolution how fine a granularity to compute lookup arrays in the angular space
         */
        void computeOffsets(
            LocalizedRangeScan *scan,
            double angle_center,
            double angle_offset,
            double angle_resolution)
        {
                assert(angle_offset != 0.0);
                assert(angle_resolution != 0.0);

                uint32_t n_angles =
                    static_cast<uint32_t>(math::Round(angle_offset * 2.0 / angle_resolution) + 1);
                setSize(n_angles);

                //////////////////////////////////////////////////////
                // convert points into local coordinates of scan pose

                const PointVectorDouble &point_readings = scan->getPointReadings();

                // compute transform to scan pose
                Transform transform(scan->getSensorPose());

                Pose2Vector local_points;
                for (const auto &point : point_readings)
                {
                        // do inverse transform to get points in local coordinates
                        Pose2 vec = transform.inverseTransformPose(Pose2(point, 0.0));
                        local_points.push_back(vec);
                }

                //////////////////////////////////////////////////////
                // create lookup array for different angles
                double angle = 0.0;
                double start_angle = angle_center - angle_offset;
                for (uint32_t angle_index = 0; angle_index < n_angles; angle_index++)
                {
                        angle = start_angle + angle_index * angle_resolution;
                        computeOffsets(angle_index, angle, local_points, scan);
                }
        }

private:
        /**
         * Compute lookup value of points for given angle
         * @param angle_index
         * @param angle
         * @param local_points
         */
        void computeOffsets(
            uint32_t angle_index, double angle, const Pose2Vector &local_points,
            LocalizedRangeScan *scan)
        {
                lookup_array_[angle_index]->setSize(static_cast<uint32_t>(local_points.size()));
                angles_.at(angle_index) = angle;

                // set up point array by computing relative offsets to points readings
                // when rotated by given angle

                const Vector2<double> &grid_offset = grid_->getCoordinateConverter()->getOffset();

                double cosine = cos(angle);
                double sine = sin(angle);

                uint32_t reading_index = 0;

                int32_t *angle_index_pointer = lookup_array_[angle_index]->getArrayPointer();

                for (const auto &point : local_points)
                {
                        const Vector2<double> &position = point.getPosition();
                        if (std::isnan(scan->getRangeReadings()[reading_index]) ||
                            std::isinf(scan->getRangeReadings()[reading_index]))
                        {
                                angle_index_pointer[reading_index] = INVALID_SCAN;
                                reading_index++;
                                continue;
                        }

                        // counterclockwise rotation and that rotation is about the origin (0, 0).
                        Vector2<double> offset;
                        offset.setX(cosine * position.getX() - sine * position.getY());
                        offset.setY(sine * position.getX() + cosine * position.getY());

                        // have to compensate for the grid offset when getting the grid index
                        Vector2<int32_t> grid_point = grid_->convertWorldToGrid(offset + grid_offset);

                        // use base GridIndex to ignore ROI
                        int32_t lookup_index = grid_->Grid<T>::getGridIndex(grid_point, false);

                        angle_index_pointer[reading_index] = lookup_index;

                        reading_index++;
                }

                assert(reading_index == local_points.size());
        }

        /**
         * Sets size of lookup table (resize if not big enough)
         * @param size
         */
        void setSize(uint32_t size)
        {
                assert(size != 0);

                if (size > capacity_)
                {
                        lookup_array_.clear();

                        capacity_ = size;
                        lookup_array_.reserve(capacity_);

                        for (uint32_t i = 0; i < capacity_; i++)
                        {
                                lookup_array_.push_back(std::make_unique<LookupArray>());
                        }
                }

                size_ = size;

                angles_.resize(size);
        }

}; // GridIndexLookup

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/**
 * Occupancy grid definition. See GridStates for possible grid values.
 */
class OccupancyGrid : public Grid<uint8_t>
{

        friend class CellUpdater;

private:
        std::unique_ptr<CellUpdater> cell_updater_;

        ////////////////////////////////////////////////////////////
        // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
        // then not many beams will hit the cell!

        // Number of beams that must pass through a cell before it will be considered to be occupied
        // or unoccupied.  This prevents stray beams from messing up the map.
        uint32_t min_pass_through_;

        // Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied
        double occupancy_threshold_;

protected:
        /**
         * Counters of number of times a beam passed through a cell
         */
        std::unique_ptr<Grid<uint32_t>> cell_pass_cnt_;

        /**
         * Counters of number of times a beam ended at a cell
         */
        std::unique_ptr<Grid<uint32_t>> cell_hit_cnt_;

public:
        /**
         * Constructs an occupancy grid of given size
         * @param width
         * @param height
         * @param offset
         * @param resolution
         */
        OccupancyGrid(
            int32_t width, int32_t height,
            const Vector2<double> &offset,
            double resolution)
        : Grid<uint8_t>(width, height),
          cell_updater_(nullptr),
          cell_pass_cnt_(Grid<uint32_t>::createGrid(0, 0, resolution)),
          cell_hit_cnt_(Grid<uint32_t>::createGrid(0, 0, resolution))
        {
                cell_updater_ = std::make_unique<CellUpdater>(this);
                if (math::DoubleEqual(resolution, 0.0)) {
                        throw std::invalid_argument("Resolution cannot be 0");
                }

                // set default value
                min_pass_through_ = 2;
                occupancy_threshold_ = 0.1;
                getCoordinateConverter()->setScale(1.0 / resolution);
                getCoordinateConverter()->setOffset(offset);
        }

        /**
         * Destructor
         */
        virtual ~OccupancyGrid()
        {
                cell_updater_.reset();
                cell_pass_cnt_.reset();
                cell_hit_cnt_.reset();
        }

public:
        /**
         * Create an occupancy grid from the given scans using the given resolution
         * @param scans
         * @param resolution
         */
        static OccupancyGrid *createFromScans(
                const std::vector<LocalizedRangeScan *> &scans,
                double resolution,
                uint32_t min_pass_through,
                double occupancy_threshold)
        {
                if (scans.empty()) {
                        return nullptr;
                }

                int32_t width, height;
                Vector2<double> offset;
                computeGridDimensions(scans, resolution, width, height, offset);
                OccupancyGrid *occupancy_grid = new OccupancyGrid(width, height, offset, resolution);
                occupancy_grid->setMinPassThrough(min_pass_through);
                occupancy_grid->setOccupancyThreshold(occupancy_threshold);
                occupancy_grid->createFromScans(scans);

                return occupancy_grid;
        }

        /**
         * Calculate grid dimensions from localized range scans
         * @param scans
         * @param resolution
         * @param width
         * @param height
         * @param offset
         */
        static void computeGridDimensions(
            const std::vector<LocalizedRangeScan *> &scans,
            double resolution,
            int32_t &width,
            int32_t &height,
            Vector2<double> &offset)
        {
                BoundingBox2 bounding_box;

                for (const auto &scan : scans)
                {
                        if (scan == nullptr) {
                                continue;
                        }

                        bounding_box.add(scan->getBoundingBox());
                }

                double scale = 1.0 / resolution;
                Size2<double> size = bounding_box.getSize();

                width = static_cast<int32_t>(math::Round(size.getWidth() * scale));
                height = static_cast<int32_t>(math::Round(size.getHeight() * scale));
                offset = bounding_box.getMinimum();
        }

        /**
         * Traces a beam from the start position to the end position marking
         * the bookkeeping arrays accordingly.
         * @param world_from start position of beam
         * @param world_to end position of beam
         * @param is_endpoint_valid is the reading within the range threshold?
         * @param do_update whether to update the cells' occupancy status immediately
         * @return returns false if an endpoint fell off the grid, otherwise true
         */
        bool traceRay(
            const Vector2<double> &world_from,
            const Vector2<double> &world_to,
            bool is_endpoint_valid,
            bool do_update = false)
        {
                assert(cell_pass_cnt_ != nullptr && cell_hit_cnt_ != nullptr);

                Vector2<int32_t> grid_from = cell_pass_cnt_->convertWorldToGrid(world_from);
                Vector2<int32_t> grid_to = cell_pass_cnt_->convertWorldToGrid(world_to);

                CellUpdater *cell_updater = do_update ? cell_updater_.get() : nullptr;
                cell_pass_cnt_->traceLine(grid_from.getX(), grid_from.getY(), grid_to.getX(),
                                          grid_to.getY(), cell_updater);

                // for the end point
                if (is_endpoint_valid)
                {
                        if (cell_pass_cnt_->isValidGridIndex(grid_to))
                        {
                                int32_t index = cell_pass_cnt_->getGridIndex(grid_to, false);

                                uint32_t *cell_pass_cnt_ptr = cell_pass_cnt_->getDataPointer();
                                uint32_t *cell_hit_cnt_ptr = cell_hit_cnt_->getDataPointer();

                                // increment cell pass through and hit count
                                cell_pass_cnt_ptr[index]++;
                                cell_hit_cnt_ptr[index]++;

                                if (do_update)
                                {
                                        (*cell_updater)(index);
                                }
                        }
                }

                return cell_pass_cnt_->isValidGridIndex(grid_to);
        }

        /**
         * Updates a single cell's value based on the given counters
         * @param cell
         * @param cell_pass_cnt
         * @param cell_hit_cnt
         */
        virtual void updateCell(uint8_t *cell, uint32_t cell_pass_cnt, uint32_t cell_hit_cnt)
        {
                if (cell_pass_cnt > min_pass_through_)
                {
                        double hit_ratio = static_cast<double>(cell_hit_cnt) / static_cast<double>(cell_pass_cnt);

                        if (hit_ratio > occupancy_threshold_)
                        {
                                *cell = GRIDSTATES_OCCUPIED;
                        }
                        else
                        {
                                *cell = GRIDSTATES_FREE;
                        }
                }
        }

        /**
         * Create grid using scans
         * @param scans
         */
        void createFromScans(const std::vector<LocalizedRangeScan *> &scans)
        {
                cell_pass_cnt_->resize(getWidth(), getHeight());
                cell_pass_cnt_->getCoordinateConverter()->setOffset(getCoordinateConverter()->getOffset());
                cell_hit_cnt_->resize(getWidth(), getHeight());
                cell_hit_cnt_->getCoordinateConverter()->setOffset(getCoordinateConverter()->getOffset());

                for (const auto &scan_iter : scans)
                {
                        if (scan_iter == nullptr)
                        {
                                continue;
                        }

                        LocalizedRangeScan *scan = scan_iter;
                        addScan(scan);
                }

                update();
        }

        /**
         * Adds the scan's information to this grid's counters (optionally
         * update the grid's cells' occupancy status)
         * @param scan
         * @param do_update whether to update the grid's cell's occupancy status
         * @return returns false if an endpoint fell off the grid, otherwise true
         */
        bool addScan(LocalizedRangeScan *scan, bool do_update = false)
        {
                LaserRangeFinder *laser = scan->getLaserRangeFinder();
                double range_threshold = laser->getRangeThreshold();
                double max_range = laser->getMaximumRange();
                double min_range = laser->getMinimumRange();

                Vector2<double> scan_position = scan->getSensorPose().getPosition();
                // get scan point readings
                const std::vector<Vector2<double>> &point_readings = scan->getPointReadings(false);

                bool is_all_in_map = true;

                // draw lines from scan position to all point readings
                int point_index = 0;

                for (const auto &point_iter : point_readings)
                {
                        Vector2<double> point = point_iter;
                        double range_reading = scan->getRangeReadings()[point_index];
                        bool is_endpoint_valid = range_reading < range_threshold;

                        if (range_reading <= min_range || range_reading >= max_range || std::isnan(range_reading))
                        {
                                // ignore these readings
                                point_index++;
                                continue;
                        }
                        else if (range_reading >= range_threshold)
                        {
                                // clip range reading to be within trusted region
                                double ratio = range_threshold / range_reading;
                                double dx = point.getX() - scan_position.getX();
                                double dy = point.getY() - scan_position.getY();
                                point.setX(scan_position.getX() + ratio * dx);
                                point.setY(scan_position.getY() + ratio * dy);
                        }

                        bool is_in_map = traceRay(scan_position, point, is_endpoint_valid, do_update);

                        if (!is_in_map)
                        {
                                is_all_in_map = false;
                        }

                        point_index++;
                }

                return is_all_in_map;
        }

        /**
         * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
         */
        virtual void update()
        {
                assert(cell_pass_cnt_ != nullptr && cell_pass_cnt_ != nullptr);

                // clear grid
                clear();

                // set occupancy status of cells
                uint8_t *data_ptr = getDataPointer();
                uint32_t *cell_pass_cnt_ptr = cell_pass_cnt_->getDataPointer();
                uint32_t *cell_hit_cnt_ptr = cell_hit_cnt_->getDataPointer();

                uint32_t n_bytes = getDataSize();
                for (uint32_t i = 0; i < n_bytes; i++, data_ptr++, cell_pass_cnt_ptr++, cell_hit_cnt_ptr++)
                {
                        updateCell(data_ptr, *cell_pass_cnt_ptr, *cell_hit_cnt_ptr);
                }
        }

public:
        /**
         * Sets the minimum number of beams that must pass through a cell before it
         * will be considered to be occupied or unoccupied.
         * This prevents stray beams from messing up the map.
         */
        void setMinPassThrough(uint32_t count)
        {
                occupancy_threshold_ = count;
        }

        /**
         * Sets the minimum ratio of beams hitting cell to beams passing through
         * cell for cell to be marked as occupied.
         */
        void setOccupancyThreshold(double thresh)
        {
                occupancy_threshold_ = thresh;
        }

private:
        /**
         * Restrict the copy constructor
         */
        OccupancyGrid(const OccupancyGrid &);

        /**
         * Restrict the assignment operator
         */
        const OccupancyGrid &operator=(const OccupancyGrid &);

}; // OccupancyGrid

}
#endif // KARTO_SDK_KARTO_HPP