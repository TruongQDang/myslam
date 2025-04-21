#ifndef MATH_SLAM_HPP
#define MATH_SLAM_HPP

#include <cstddef>

namespace math
{

const double TOLERANCE = 1e-06;

const double PI_180 = 0.01745329251994329577;
const double PI = 3.14159265358979323846;
const double PI2 = 6.28318530717958647692;

const int32_t INVALID_SCAN = std::numeric_limits<int32_t>::max();

/**
 * Align a value to the alignValue.
 * The alignValue should be the power of two (2, 4, 8, 16, 32 and so on)
 * @param value
 * @param alignValue
 * @return aligned value
 */
template <class T>
inline T AlignValue(size_t value, size_t alignValue = 8)
{
        return static_cast<T>((value + (alignValue - 1)) & ~(alignValue - 1));
}

/**
 * Checks whether value is in the range [a;b]
 * @param value
 * @param a
 * @param b
 */
template <typename T>
inline bool InRange(const T &value, const T &a, const T &b)
{
        return value >= a && value <= b;
}

/**
 * Checks whether value is in the range [0;maximum)
 * @param value
 * @param maximum
 */
template <typename T>
inline bool IsUpTo(const T &value, const T &maximum)
{
        return value >= 0 && value < maximum;
}

template <typename T>
inline T Square(T value)
{
        return value * value;
}

/**
 * Checks whether two numbers are equal within a certain tolerance.
 * @param a
 * @param b
 * @return true if a and b differ by at most a certain tolerance.
 */
inline bool DoubleEqual(double a, double b)
{
        double delta = a - b;
        return delta < 0.0 ? delta >= -TOLERANCE : delta <= TOLERANCE;
}

/**
 * Converts degrees into radians
 * @param degrees
 * @return radian equivalent of degrees
 */
inline double DegreesToRadians(double degrees)
{
        return degrees * PI_180;
}

/**
 * Returns an equivalent angle to the first parameter such that the difference
 * when the second parameter is subtracted from this new value is an angle
 * in the normalized range of [-pi, pi], i.e. abs(minuend - subtrahend) <= pi.
 * @param minuend
 * @param subtrahend
 * @return normalized angle
 */
inline double NormalizeAngleDifference(double minuend, double subtrahend)
{
        while (minuend - subtrahend < -PI)
        {
                minuend += PI;
        }

        while (minuend - subtrahend > PI)
        {
                minuend -= PI;
        }

        return minuend;
}

/**
 * Normalizes angle to be in the range of [-pi, pi]
 * @param angle to be normalized
 * @return normalized angle
 */
inline double NormalizeAngle(double angle)
{
        while (angle < -math::PI)
        {
                if (angle < -PI2)
                {
                        angle += (u_int32_t)(angle / -PI2) * PI2;
                }
                else
                {
                        angle += PI2;
                }
        }

        while (angle > math::PI)
        {
                if (angle > PI2)
                {
                        angle -= (uint32_t)(angle / PI2) * PI2;
                }
                else
                {
                        angle -= PI2;
                }
        }

        assert(math::InRange(angle, -math::PI, math::PI));

        return angle;
}

} // namespace math

#endif // MATH_SLAM_HPP