#ifndef KARTO_SDK_KARTO_HPP
#define KARTO_SDK_KARTO_HPP

#include <memory>
#include <vector>
#include <boost/thread.hpp>

#include <Eigen/Geometry>

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

typedef std::vector<Vector2<double>> PointVectorDouble;

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

//////////////////////////////
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
                m_Values[0] = 0;
                m_Values[1] = 0;
        }

        /**
         * Constructor initializing vector location
         * @param x
         * @param y
         */
        Vector2(T x, T y)
        {
                m_Values[0] = x;
                m_Values[1] = y;
        }

public:
        /**
         * Gets the x-coordinate of this vector2
         * @return the x-coordinate of the vector2
         */
        inline const T &GetX() const
        {
                return m_Values[0];
        }

        /**
         * Sets the x-coordinate of this vector2
         * @param x the x-coordinate of the vector2
         */
        inline void SetX(const T &x)
        {
                m_Values[0] = x;
        }

        /**
         * Gets the y-coordinate of this vector2
         * @return the y-coordinate of the vector2
         */
        inline const T &GetY() const
        {
                return m_Values[1];
        }

        /**
         * Sets the y-coordinate of this vector2
         * @param y the y-coordinate of the vector2
         */
        inline void SetY(const T &y)
        {
                m_Values[1] = y;
        }

        /**
         * Floor point operator
         * @param rOther
         */
        inline void MakeFloor(const Vector2 &rOther)
        {
                if (rOther.m_Values[0] < m_Values[0])
                {
                        m_Values[0] = rOther.m_Values[0];
                }
                if (rOther.m_Values[1] < m_Values[1])
                {
                        m_Values[1] = rOther.m_Values[1];
                }
        }

        /**
         * Ceiling point operator
         * @param rOther
         */
        inline void MakeCeil(const Vector2 &rOther)
        {
                if (rOther.m_Values[0] > m_Values[0])
                {
                        m_Values[0] = rOther.m_Values[0];
                }
                if (rOther.m_Values[1] > m_Values[1])
                {
                        m_Values[1] = rOther.m_Values[1];
                }
        }

        /**
         * Returns the square of the length of the vector
         * @return square of the length of the vector
         */
        inline double SquaredLength() const
        {
                return math::Square(m_Values[0]) + math::Square(m_Values[1]);
        }

        /**
         * Returns the length of the vector (x and y).
         * @return length of the vector
         */
        inline double Length() const
        {
                return sqrt(SquaredLength());
        }

        /**
         * Returns the square distance to the given vector
         * @returns square distance to the given vector
         */
        inline double SquaredDistance(const Vector2 &rOther) const
        {
                return (*this - rOther).SquaredLength();
        }

        /**
         * Gets the distance to the other vector2
         * @param rOther
         * @return distance to other vector2
         */
        inline double Distance(const Vector2 &rOther) const
        {
                return sqrt(SquaredDistance(rOther));
        }

public:
        /**
         * In place Vector2 addition.
         */
        inline void operator+=(const Vector2 &rOther)
        {
                m_Values[0] += rOther.m_Values[0];
                m_Values[1] += rOther.m_Values[1];
        }

        /**
         * In place Vector2 subtraction.
         */
        inline void operator-=(const Vector2 &rOther)
        {
                m_Values[0] -= rOther.m_Values[0];
                m_Values[1] -= rOther.m_Values[1];
        }

        /**
         * Addition operator
         * @param rOther
         * @return vector resulting from adding this vector with the given vector
         */
        inline const Vector2 operator+(const Vector2 &rOther) const
        {
                return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
        }

        /**
         * Subtraction operator
         * @param rOther
         * @return vector resulting from subtracting this vector from the given vector
         */
        inline const Vector2 operator-(const Vector2 &rOther) const
        {
                return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
        }

        /**
         * In place scalar division operator
         * @param scalar
         */
        inline void operator/=(T scalar)
        {
                m_Values[0] /= scalar;
                m_Values[1] /= scalar;
        }

        /**
         * Divides a Vector2
         * @param scalar
         * @return scalar product
         */
        inline const Vector2 operator/(T scalar) const
        {
                return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
        }

        /**
         * Computes the dot product between the two vectors
         * @param rOther
         * @return dot product
         */
        inline double operator*(const Vector2 &rOther) const
        {
                return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
        }

        /**
         * Scales the vector by the given scalar
         * @param scalar
         */
        inline const Vector2 operator*(T scalar) const
        {
                return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
        }

        /**
         * Subtract the vector by the given scalar
         * @param scalar
         */
        inline const Vector2 operator-(T scalar) const
        {
                return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
        }

        /**
         * In place scalar multiplication operator
         * @param scalar
         */
        inline void operator*=(T scalar)
        {
                m_Values[0] *= scalar;
                m_Values[1] *= scalar;
        }

        /**
         * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
         * @param rOther
         */
        inline bool operator==(const Vector2 &rOther) const
        {
                return m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1];
        }

        /**
         * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
         * @param rOther
         */
        inline bool operator!=(const Vector2 &rOther) const
        {
                return m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1];
        }

        /**
         * Less than operator
         * @param rOther
         * @return true if left vector is less than right vector
         */
        inline bool operator<(const Vector2 &rOther) const
        {
                if (m_Values[0] < rOther.m_Values[0])
                {
                        return true;
                }
                else if (m_Values[0] > rOther.m_Values[0])
                {
                        return false;
                }
                else
                {
                        return m_Values[1] < rOther.m_Values[1];
                }
        }

        /**
         * Write Vector2 onto output stream
         * @param rStream output stream
         * @param rVector to write
         */
        friend inline std::ostream &operator<<(std::ostream &rStream, const Vector2 &rVector)
        {
                rStream << rVector.GetX() << " " << rVector.GetY();
                return rStream;
        }

        /**
         * Read Vector2 from input stream
         * @param rStream input stream
         */
        friend inline std::istream &operator>>(std::istream &rStream, const Vector2 & /*rVector*/)
        {
                // Implement me!!  TODO(lucbettaieb): What the what?  Do I need to implement this?
                return rStream;
        }

        friend class boost::serialization::access;
        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
                ar &boost::serialization::make_nvp("m_Values_0", m_Values[0]);
                ar &boost::serialization::make_nvp("m_Values_1", m_Values[1]);
        }

private:
        T m_Values[2];
}; // Vector2<T>

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
            : m_Heading(0.0)
        {
        }

        /**
         * Constructor initializing pose parameters
         * @param rPosition position
         * @param heading heading
         **/
        Pose2(const Vector2<double> &rPosition, double heading)
            : m_Position(rPosition),
              m_Heading(heading)
        {
        }

        /**
         * Constructor initializing pose parameters
         * @param x x-coordinate
         * @param y y-coordinate
         * @param heading heading
         **/
        Pose2(double x, double y, double heading)
            : m_Position(x, y),
              m_Heading(heading)
        {
        }

        /**
         * Copy constructor
         */
        Pose2(const Pose2 &rOther)
            : m_Position(rOther.m_Position),
              m_Heading(rOther.m_Heading)
        {
        }

public:
        /**
         * Returns the x-coordinate
         * @return the x-coordinate of the pose
         */
        inline double GetX() const
        {
                return m_Position.GetX();
        }

        /**
         * Sets the x-coordinate
         * @param x the x-coordinate of the pose
         */
        inline void SetX(double x)
        {
                m_Position.SetX(x);
        }

        /**
         * Returns the y-coordinate
         * @return the y-coordinate of the pose
         */
        inline double GetY() const
        {
                return m_Position.GetY();
        }

        /**
         * Sets the y-coordinate
         * @param y the y-coordinate of the pose
         */
        inline void SetY(double y)
        {
                m_Position.SetY(y);
        }

        /**
         * Returns the position
         * @return the position of the pose
         */
        inline const Vector2<double> &GetPosition() const
        {
                return m_Position;
        }

        /**
         * Sets the position
         * @param rPosition of the pose
         */
        inline void SetPosition(const Vector2<double> &rPosition)
        {
                m_Position = rPosition;
        }

        /**
         * Returns the heading of the pose (in radians)
         * @return the heading of the pose
         */
        inline double GetHeading() const
        {
                return m_Heading;
        }

        /**
         * Sets the heading
         * @param heading of the pose
         */
        inline void SetHeading(double heading)
        {
                m_Heading = heading;
        }

        /**
         * Return the squared distance between two Pose2
         * @return squared distance
         */
        inline double SquaredDistance(const Pose2 &rOther) const
        {
                return m_Position.SquaredDistance(rOther.m_Position);
        }

public:
        /**
         * Assignment operator
         */
        inline Pose2 &operator=(const Pose2 &rOther)
        {
                m_Position = rOther.m_Position;
                m_Heading = rOther.m_Heading;

                return *this;
        }

        /**
         * Equality operator
         */
        inline bool operator==(const Pose2 &rOther) const
        {
                return m_Position == rOther.m_Position && m_Heading == rOther.m_Heading;
        }

        /**
         * Inequality operator
         */
        inline bool operator!=(const Pose2 &rOther) const
        {
                return m_Position != rOther.m_Position || m_Heading != rOther.m_Heading;
        }

        /**
         * In place Pose2 add.
         */
        inline void operator+=(const Pose2 &rOther)
        {
                m_Position += rOther.m_Position;
                m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
        }

        /**
         * Binary Pose2 add
         * @param rOther
         * @return Pose2 sum
         */
        inline Pose2 operator+(const Pose2 &rOther) const
        {
                return Pose2(m_Position + rOther.m_Position,
                             math::NormalizeAngle(m_Heading + rOther.m_Heading));
        }

        /**
         * Binary Pose2 subtract
         * @param rOther
         * @return Pose2 difference
         */
        inline Pose2 operator-(const Pose2 &rOther) const
        {
                return Pose2(m_Position - rOther.m_Position,
                             math::NormalizeAngle(m_Heading - rOther.m_Heading));
        }

        /**
         * Read pose from input stream
         * @param rStream input stream
         */
        friend inline std::istream &operator>>(std::istream &rStream, const Pose2 & /*rPose*/)
        {
                // Implement me!!
                return rStream;
        }

        /**
         * Write this pose onto output stream
         * @param rStream output stream
         * @param rPose to read
         */
        friend inline std::ostream &operator<<(std::ostream &rStream, const Pose2 &rPose)
        {
                rStream << rPose.m_Position.GetX() << " " << rPose.m_Position.GetY() << " " << rPose.m_Heading;
                return rStream;
        }

private:
        Vector2<double> m_Position;

        double m_Heading;
}; // Pose2

/**
 * Type declaration of Pose2 vector
 */
typedef std::vector<Pose2> Pose2Vector;


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
    Clear();
  }

  /**
   * Copy constructor
   */
  inline Matrix3(const Matrix3 & rOther)
  {
    memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(double));
  }

public:
  /**
   * Sets this matrix to identity matrix
   */
  void SetToIdentity()
  {
    memset(m_Matrix, 0, 9 * sizeof(double));

    for (int32_t i = 0; i < 3; i++) {
      m_Matrix[i][i] = 1.0;
    }
  }

  /**
   * Sets this matrix to zero matrix
   */
  void Clear()
  {
    memset(m_Matrix, 0, 9 * sizeof(double));
  }

  /**
   * Sets this matrix to be the rotation matrix of rotation around given axis
   * @param x x-coordinate of axis
   * @param y y-coordinate of axis
   * @param z z-coordinate of axis
   * @param radians amount of rotation
   */
  void FromAxisAngle(double x, double y, double z, const double radians)
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

    m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
    m_Matrix[0][1] = xyMCos - zSin;
    m_Matrix[0][2] = xzMCos + ySin;

    m_Matrix[1][0] = xyMCos + zSin;
    m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
    m_Matrix[1][2] = yzMCos - xSin;

    m_Matrix[2][0] = xzMCos - ySin;
    m_Matrix[2][1] = yzMCos + xSin;
    m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
  }

  /**
   * Returns transposed version of this matrix
   * @return transposed matrix
   */
  Matrix3 Transpose() const
  {
    Matrix3 transpose;

    for (uint32_t row = 0; row < 3; row++) {
      for (uint32_t col = 0; col < 3; col++) {
        transpose.m_Matrix[row][col] = m_Matrix[col][row];
      }
    }

    return transpose;
  }

  /**
   * Returns the inverse of the matrix
   */
  Matrix3 Inverse() const
  {
    Matrix3 kInverse = *this;
    bool haveInverse = InverseFast(kInverse, 1e-14);
    if (haveInverse == false) {
      assert(false);
    }
    return kInverse;
  }

  /**
   * Internal helper method for inverse matrix calculation
   * This code is lifted from the OgreMatrix3 class!!
   */
  bool InverseFast(Matrix3 & rkInverse, double fTolerance = KT_TOLERANCE) const
  {
    // Invert a 3x3 using cofactors.  This is about 8 times faster than
    // the Numerical Recipes code which uses Gaussian elimination.
    rkInverse.m_Matrix[0][0] = m_Matrix[1][1] * m_Matrix[2][2] - m_Matrix[1][2] * m_Matrix[2][1];
    rkInverse.m_Matrix[0][1] = m_Matrix[0][2] * m_Matrix[2][1] - m_Matrix[0][1] * m_Matrix[2][2];
    rkInverse.m_Matrix[0][2] = m_Matrix[0][1] * m_Matrix[1][2] - m_Matrix[0][2] * m_Matrix[1][1];
    rkInverse.m_Matrix[1][0] = m_Matrix[1][2] * m_Matrix[2][0] - m_Matrix[1][0] * m_Matrix[2][2];
    rkInverse.m_Matrix[1][1] = m_Matrix[0][0] * m_Matrix[2][2] - m_Matrix[0][2] * m_Matrix[2][0];
    rkInverse.m_Matrix[1][2] = m_Matrix[0][2] * m_Matrix[1][0] - m_Matrix[0][0] * m_Matrix[1][2];
    rkInverse.m_Matrix[2][0] = m_Matrix[1][0] * m_Matrix[2][1] - m_Matrix[1][1] * m_Matrix[2][0];
    rkInverse.m_Matrix[2][1] = m_Matrix[0][1] * m_Matrix[2][0] - m_Matrix[0][0] * m_Matrix[2][1];
    rkInverse.m_Matrix[2][2] = m_Matrix[0][0] * m_Matrix[1][1] - m_Matrix[0][1] * m_Matrix[1][0];

    double fDet = m_Matrix[0][0] * rkInverse.m_Matrix[0][0] +
      m_Matrix[0][1] * rkInverse.m_Matrix[1][0] +
      m_Matrix[0][2] * rkInverse.m_Matrix[2][0];

    if (fabs(fDet) <= fTolerance) {
      return false;
    }

    double fInvDet = 1.0 / fDet;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        rkInverse.m_Matrix[row][col] *= fInvDet;
      }
    }

    return true;
  }

  /**
   * Returns a string representation of this matrix
   * @return string representation of this matrix
   */
  inline std::string ToString() const
  {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        converter << m_Matrix[row][col] << " ";
      }
    }

    return converter.str();
  }

public:
  /**
   * Assignment operator
   */
  inline Matrix3 & operator=(const Matrix3 & rOther)
  {
    memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(double));
    return *this;
  }

  /**
   * Matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return reference to mat(r,c)
   */
  inline double & operator()(uint32_t row, uint32_t column)
  {
    return m_Matrix[row][column];
  }

  /**
   * Read-only matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return mat(r,c)
   */
  inline double operator()(uint32_t row, uint32_t column) const
  {
    return m_Matrix[row][column];
  }

  /**
   * Binary Matrix3 multiplication.
   * @param rOther
   * @return Matrix3 product
   */
  Matrix3 operator*(const Matrix3 & rOther) const
  {
    Matrix3 product;

    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        product.m_Matrix[row][col] = m_Matrix[row][0] * rOther.m_Matrix[0][col] +
          m_Matrix[row][1] * rOther.m_Matrix[1][col] +
          m_Matrix[row][2] * rOther.m_Matrix[2][col];
      }
    }

    return product;
  }

  /**
   * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
   * @param rPose2
   * @return Pose2 product
   */
  inline Pose2 operator*(const Pose2 & rPose2) const
  {
    Pose2 pose2;

    pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] *
      rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
    pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] *
      rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
    pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] *
      rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

    return pose2;
  }

  /**
   * In place Matrix3 add.
   * @param rkMatrix
   */
  inline void operator+=(const Matrix3 & rkMatrix)
  {
    for (uint32_t row = 0; row < 3; row++) {
      for (uint32_t col = 0; col < 3; col++) {
        m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
      }
    }
  }

  /**
   * Write Matrix3 onto output stream
   * @param rStream output stream
   * @param rMatrix to write
   */
  friend inline std::ostream & operator<<(std::ostream & rStream, const Matrix3 & rMatrix)
  {
    rStream << rMatrix.ToString();
    return rStream;
  }

private:
  double m_Matrix[3][3];
};

/**
 * Implementation of a Pose2 transform
 */
class Transform
{
public:
        /**
         * Constructs a transformation from the origin to the given pose
         * @param rPose pose
         */
        Transform(const Pose2 &rPose) // NOLINT
        {
                SetTransform(Pose2(), rPose);
        }

        /**
         * Constructs a transformation from the first pose to the second pose
         * @param rPose1 first pose
         * @param rPose2 second pose
         */
        Transform(const Pose2 &rPose1, const Pose2 &rPose2)
        {
                SetTransform(rPose1, rPose2);
        }

public:
        /**
         * Transforms the pose according to this transform
         * @param rSourcePose pose to transform from
         * @return transformed pose
         */
        inline Pose2 TransformPose(const Pose2 &rSourcePose)
        {
                Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
                double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

                return Pose2(newPosition.GetPosition(), angle);
        }

        /**
         * Inverse transformation of the pose according to this transform
         * @param rSourcePose pose to transform from
         * @return transformed pose
         */
        inline Pose2 InverseTransformPose(const Pose2 &rSourcePose)
        {
                Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
                double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

                // components of transform
                return Pose2(newPosition.GetPosition(), angle);
        }

private:
        /**
         * Sets this to be the transformation from the first pose to the second pose
         * @param rPose1 first pose
         * @param rPose2 second pose
         */
        void SetTransform(const Pose2 &rPose1, const Pose2 &rPose2)
        {
                if (rPose1 == rPose2)
                {
                        m_Rotation.SetToIdentity();
                        m_InverseRotation.SetToIdentity();
                        m_Transform = Pose2();
                        return;
                }

                // heading transformation
                m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
                m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

                // position transformation
                Pose2 newPosition;
                if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
                {
                        newPosition = rPose2 - m_Rotation * rPose1;
                }
                else
                {
                        newPosition = rPose2;
                }

                m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
        }

private:
        // pose transformation
        Pose2 m_Transform;

        Matrix3 m_Rotation;
        Matrix3 m_InverseRotation;
}; // Transform

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

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
        inline const Vector2<double> &getMinimum() const
        {
                return minimum_;
        }

        /**
         * Set bounding box minimum
         */
        inline void setMinimum(const Vector2<double> &minimum)
        {
                minimum_ = minimum;
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
        inline void setMaximum(const Vector2<double> &maximum)
        {
                maximum_ = maximum;
        }

        /**
         * Get the size of the bounding box
         */
        inline Size2<double> getSize() const
        {
                Vector2<double> size = maximum_ - minimum_;

                return Size2<double>(size.x(), size.y());
        }

        /**
         * Add vector to bounding box
         */
        inline void add(const Vector2<double> &point)
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
        Vector2<double> minimum_;
        Vector2<double> maximum_;
}; // BoundingBox2

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

template <typename T>
class Rectangle2
{
private:
        Vector2<T> position_;
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

public:
        inline T getWidth() const
        {
                return size_.getWidth();
        }

        inline T getHeight() const
        {
                return size_.getHeight();
        }

        /**
         * Gets the x-coordinate of the left edge of this rectangle
         * @return the x-coordinate of the left edge of this rectangle
         */
        inline T getX() const
        {
                return position_.x();
        }

        /**
         * Gets the y-coordinate of the bottom edge of this rectangle
         * @return the y-coordinate of the bottom edge of this rectangle
         */
        inline T getY() const
        {
                return position_.y();
        }

}; // Rectangle2

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

class LaserRangeFinder
{
private:
        double range_threshold_;

        double minimum_range_;
        double maximum_range_;

        double minimum_angle_;
        double maximum_angle_;

        double angular_resolution_;

        uint32_t number_of_range_readings_;

        Pose2 offset_pose_;

        std::string frame_id_;

public:
        LaserRangeFinder()
        {
        }

        inline void setFrameId(std::string frame_id)
        {
                frame_id_ = frame_id;
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
                range_threshold_ = range_threshold;
        }

        inline void setMaximumAngle(double maximum_angle)
        {
                maximum_angle_ = maximum_angle;
                updateNumberOfRangeReadings();
        }

        inline double getMaximumAngle()
        {
                return maximum_angle_;
        }

        inline void setMinimumAngle(double minimum_angle)
        {
                minimum_angle_ = minimum_angle;
                updateNumberOfRangeReadings();
        }

        inline double getMinimumAngle() const
        {
                return minimum_angle_;
        }

        inline void setAngularResolution(double angular_resolution)
        {
                angular_resolution_ = angular_resolution;
                updateNumberOfRangeReadings();
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

        void updateNumberOfRangeReadings()
        {
                number_of_range_readings_ = static_cast<uint32_t>(math::Round(
                    (getMaximumAngle() -
                     getMinimumAngle()) /
                    getAngularResolution()));
        }
}; // LaserRangeFinder

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

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
        int32_t scan_id_;
        Pose2 corrected_pose_;
        Pose2 odom_pose_;
        /**
         * Average of all the point readings
         */
        Pose2 barycenter_pose_;
        std::unique_ptr<double[]> range_readings_;
        uint32_t number_of_range_readings_;
        double time_;
        BoundingBox2 bounding_box_;
        bool is_dirty_;
        std::vector<Vector2<double>> point_readings_;
        std::vector<Vector2<double>> unfiltered_point_readings_;
        LaserRangeFinder *laser_;

        mutable boost::shared_mutex lock_;

public:
        LocalizedRangeScan()
        {
        }

        LocalizedRangeScan(LaserRangeFinder *laser, const RangeReadingsVector &range_readings)
            : is_dirty_(true), laser_(laser)
        {
                number_of_range_readings_ = range_readings.size();

                range_readings_ = std::make_unique<double[]>(number_of_range_readings_);
                std::copy(range_readings.begin(), range_readings.end(), range_readings_.get());
        }

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
                double offsetLength = deviceOffsetPose2.GetPosition().Length();
                double offsetHeading = deviceOffsetPose2.GetHeading();
                double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
                double correctedHeading = math::NormalizeAngle(sensor_pose.GetHeading());
                Pose2 worldSensorOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                                                offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                                                offsetHeading);

                return sensor_pose - worldSensorOffset;
        }

        inline void setTime(double time)
        {
                time_ = time;
        }

        /**
         * Gets sensor data time
         * @return time
         */
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
                if (is_dirty_)
                {
                        // throw away constness and do an update!
                        lock.unlock();
                        boost::unique_lock<boost::shared_mutex> uniqueLock(lock_);
                        const_cast<LocalizedRangeScan *>(this)->update();
                }

                if (want_filtered == true)
                {
                        return point_readings_;
                }
                else
                {
                        return unfiltered_point_readings_;
                }
        }

        /**
         * Computes the position of the sensor
         * @return scan pose
         */
        inline Pose2 getSensorPose() const
        {
                return GetSensorAt(corrected_pose_);
        }

        /**
         * Computes the position of the sensor if the robot were at the given pose
         * @param rPose
         * @return sensor pose
         */
        inline Pose2 GetSensorAt(const Pose2 &rPose) const
        {
                return Transform(rPose).TransformPose(getLaserRangeFinder()->getOffsetPose());
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

                if (is_dirty_)
                {
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
         * @param rPose new pose of the robot of this scan
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
                if (is_dirty_)
                {
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
                if (is_dirty_)
                {
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
                if (laser_ != nullptr)
                {
                        point_readings_.clear();
                        unfiltered_point_readings_.clear();

                        double range_threshold = laser_->getRangeThreshold();
                        double minimum_angle = laser_->getMinimumAngle();
                        double angular_resolution = laser_->getAngularResolution();
                        Pose2 scan_pose = getSensorPose();
                        // compute point readings
                        Vector2<double> range_points_sum(0, 0);
                        uint32_t beam_num = 0;

                        for (uint32_t i = 0; i < laser_->getNumberOfRangeReadings(); i++, beam_num++)
                        {
                                double range_reading = getRangeReadings()[i];
                                double angle = scan_pose.GetHeading() + minimum_angle + beam_num * angular_resolution;
                                Vector2<double> point;
                                point.x() = scan_pose.GetX() + (range_reading * cos(angle));
                                point.y() = scan_pose.GetY() + (range_reading * sin(angle));

                                if (!math::InRange(range_reading, laser_->getMinimumRange(), range_threshold))
                                {
                                        unfiltered_point_readings_.push_back(point);
                                        continue;
                                }

                                point_readings_.push_back(point);
                                unfiltered_point_readings_.push_back(point);
                                range_points_sum += point;
                        }

                        // compute barycenter
                        double n_points = static_cast<double>(point_readings_.size());
                        if (n_points != 0.0)
                        {
                                Vector2<double> average_position = Vector2<double>(range_points_sum / n_points);
                                barycenter_pose_ = Pose2(average_position, 0.0);
                        }
                        else
                        {
                                barycenter_pose_ = scan_pose;
                        }

                        // calculate bounding box of scan
                        bounding_box_ = BoundingBox2();
                        bounding_box_.add(scan_pose.GetPosition());

                        for (const auto &point_reading : point_readings_)
                        {
                                bounding_box_.add(point_reading);
                        }
                }

                is_dirty_ = false;
        }

}; // LocalizedRangeScan

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

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
                double grid_x = (world.GetX() - offset_.GetX()) * scale_;
                double grid_y = 0.0;

                if (flip_y == false)
                {
                        grid_y = (world.GetY() - offset_.GetY()) * scale_;
                }
                else
                {
                        grid_y = (size_.getHeight() / scale_ - world.GetY() + offset_.GetY()) * scale_;
                }

                return Vector2<int32_t>(
                    static_cast<int32_t>(math::Round(grid_x)),
                    static_cast<int32_t>(math::Round(grid_y)));
        }
};

/////////////////////////////////////////////////////////
class OccupancyGrid;

class CellUpdater
{
public:
        CellUpdater(OccupancyGrid *grid)
            : occupancy_grid_(grid)
        {
        }

        void operator()(uint32_t index);

private:
        OccupancyGrid *occupancy_grid_;
}; // CellUpdater

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

template <typename T>
class Grid
{
private:
        int32_t width_;                             // width of grid
        int32_t height_;                            // height of grid
        int32_t width_step_;                        // 8 bit aligned width of grid
        T *data_;                                   // grid data
        CoordinateConverter *coordinate_converter_; // utility to convert between world coordinates and grid coordinates

public:
        Grid()
        {
        }

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

        static std::unique_ptr<Grid<T>> createGrid(int32_t width, int32_t height, double resolution)
        {
                std::unique_ptr<Grid<T>> grid = std::make_unique<Grid<T>>(width, height);

                grid->getCoordinateConverter()->setScale(1.0 / resolution);

                return grid;
        }

        /**
         * Gets the grid data pointer
         * @return data pointer
         */
        inline T *getDataPointer()
        {
                return data_;
        }

        /**
         * Gets pointer to data at given grid coordinate
         * @param rGrid grid coordinate
         * @return grid point
         */
        T *getDataPointer(const Eigen::Matrix<int32_t, 2, 1> &grid)
        {
                int32_t index = getGridIndex(grid, true);
                return data_ + index;
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
        inline T getValue(const Eigen::Matrix<int32_t, 2, 1> &grid_coordinate) const
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
                if (steep)
                {
                        std::swap(x0, y0);
                        std::swap(x1, y1);
                }
                if (x0 > x1)
                {
                        std::swap(x0, x1);
                        std::swap(y0, y1);
                }

                int32_t delta_x = x1 - x0;
                int32_t delta_y = abs(y1 - y0);
                int32_t error = 0;
                int32_t y_step;
                int32_t y = y0;

                if (y0 < y1)
                {
                        y_step = 1;
                }
                else
                {
                        y_step = -1;
                }

                int32_t point_x;
                int32_t point_y;
                for (int32_t x = x0; x <= x1; x++)
                {
                        if (steep)
                        {
                                point_x = y;
                                point_y = x;
                        }
                        else
                        {
                                point_x = x;
                                point_y = y;
                        }

                        error += delta_y;

                        if (2 * error >= delta_x)
                        {
                                y += y_step;
                                error -= delta_x;
                        }

                        Eigen::Matrix<int32_t, 2, 1> grid_index(point_x, point_y);
                        if (isValidGridIndex(grid_index))
                        {
                                int32_t index = getGridIndex(grid_index, false);
                                T *grid_pointer = getDataPointer();
                                grid_pointer[index]++;

                                if (cell_updater != nullptr)
                                {
                                        (*cell_updater)(index);
                                }
                        }
                }
        }

        /**
         * Checks whether the given coordinates are valid grid indices
         * @param grid
         */
        inline bool isValidGridIndex(const Eigen::Matrix<int32_t, 2, 1> &grid) const
        {
                return (grid(0) < width_ && grid(1) < height_);
        }

        /**
         * Converts the point from world coordinates to grid coordinates
         * @param world world coordinate
         * @param flip_y
         * @return grid coordinate
         */
        inline Vector2i convertWorldToGrid(
            const Vector2d &world,
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
        virtual int32_t getGridIndex(const Vector2i &grid, bool boundary_check = true) const
        {
                if (boundary_check == true) {
                        if (isValidGridIndex(grid) == false) {
                                throw std::runtime_error("Index out of range");
                        }
                }

                int32_t index = grid.x() + (grid.y() * width_step_);

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
         * Clear out the grid data
         */
        void clear()
        {
                memset(data_, 0, getDataSize() * sizeof(T));
        }

        /**
         * Gets the coordinate converter for this grid
         * @return coordinate converter
         */
        inline CoordinateConverter *getCoordinateConverter() const
        {
                return coordinate_converter_;
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

                if (data_ != nullptr)
                {
                        delete[] data_;
                        data_ = nullptr;
                }

                try
                {
                        data_ = new T[getDataSize()];

                        if (coordinate_converter_ == nullptr) {
                                coordinate_converter_ = new CoordinateConverter();
                        }
                        coordinate_converter_->setSize(Size2<int32_t>(width, height));
                }
                catch (...)
                {
                        data_ = nullptr;
                        width_ = 0;
                        height_ = 0;
                        width_step_ = 0;
                }

                clear();
        }
}; // Grid

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////

/**
 * Create lookup tables for point readings at varying angles in grid.
 * For each angle, grid indexes are calculated for each range reading.
 * This is to speed up finding best angle/position for a localized range scan
 *
 * Used heavily in mapper and localizer.
 *
 * In the localizer, this is a huge speed up for calculating possible position.  For each particle,
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

        GridIndexLookup(Grid<T> *grid) // NOLINT
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
         * @param pScan the scan
         * @param angleCenter
         * @param angleOffset computes lookup arrays for the angles within this offset around angleStart
         * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
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

                Pose2Vector local_points;
                for (const auto &point : point_readings)
                {
                        // get points in local coordinates
                        Pose2 vec = Pose2::transformPose(scan->getSensorPose().inverse(), Pose2(point, 0.0));
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
         * @param angleIndex
         * @param angle
         * @param rLocalPoints
         */
        void computeOffsets(
            uint32_t angle_index, double angle, const Pose2Vector &local_points,
            LocalizedRangeScan *scan)
        {
                lookup_array_[angle_index]->setSize(static_cast<uint32_t>(local_points.size()));
                angles_.at(angle_index) = angle;

                // set up point array by computing relative offsets to points readings
                // when rotated by given angle

                const Vector2d &grid_offset = grid_->getCoordinateConverter()->getOffset();

                double cosine = cos(angle);
                double sine = sin(angle);

                uint32_t reading_index = 0;

                int32_t *angle_index_pointer = lookup_array_[angle_index]->getArrayPointer();

                double max_range = scan->getLaserRangeFinder()->getMaximumRange();

                for (const auto &point : local_points)
                {
                        const Vector2d &position = point.getPosition();
                        if (std::isnan(scan->getRangeReadings()[reading_index]) ||
                            std::isinf(scan->getRangeReadings()[reading_index]))
                        {
                                angle_index_pointer[reading_index] = INVALID_SCAN;
                                reading_index++;
                                continue;
                        }

                        // counterclockwise rotation and that rotation is about the origin (0, 0).
                        Vector2d offset;
                        offset.x() = cosine * position.x() - sine * position.y();
                        offset.y() = sine * position.x() + cosine * position.y();

                        // have to compensate for the grid offset when getting the grid index
                        Vector2i grid_point = grid_->convertWorldToGrid(offset + grid_offset);

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

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////



class OccupancyGrid : public Grid<uint8_t>
{

        friend class CellUpdater;

private:
        double occupancy_threshold_;
        uint32_t min_pass_through_;
        CellUpdater *cell_updater_;

        /**
         * Restrict the copy constructor
         */
        OccupancyGrid(const OccupancyGrid &);

        /**
         * Restrict the assignment operator
         */
        const OccupancyGrid &operator=(const OccupancyGrid &);

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
                cell_updater_ = new CellUpdater(this);
                if (math::DoubleEqual(resolution, 0.0)) {
                        throw std::invalid_argument("Resolution cannot be 0");
                }

                min_pass_through_ = 2;
                occupancy_threshold_ = 0.1;

                getCoordinateConverter()->setScale(1.0 / resolution);
                getCoordinateConverter()->setOffset(offset);
        }

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
         * @param rScans
         * @param resolution
         * @param rWidth
         * @param rHeight
         * @param rOffset
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
                        if (scan == nullptr)
                        {
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
        bool rayTrace(
            const Vector2<double> &world_from,
            const Vector2<double> &world_to,
            bool is_endpoint_valid,
            bool do_update = false)
        {
                assert(cell_pass_cnt_ != nullptr && cell_hit_cnt_ != nullptr);

                Eigen::Matrix<int32_t, 2, 1> grid_from = cell_pass_cnt_->convertWorldToGrid(world_from);
                Eigen::Matrix<int32_t, 2, 1> grid_to = cell_pass_cnt_->convertWorldToGrid(world_to);

                CellUpdater *cell_updater = do_update ? cell_updater_ : nullptr;
                cell_pass_cnt_->traceLine(grid_from(0), grid_from(1), grid_to(0),
                                          grid_to(1), cell_updater);

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

                Vector2<double> scan_position = scan->getSensorPose().GetPosition();
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
                                double dx = point.x() - scan_position.x();
                                double dy = point.y() - scan_position.y();
                                point.x() = scan_position.x() + ratio * dx;
                                point.y() = scan_position.y() + ratio * dy;
                        }

                        bool is_in_map = rayTrace(scan_position, point, is_endpoint_valid, do_update);

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
}; // OccupancyGrid

}
#endif // KARTO_SDK_KARTO_HPP