#ifndef KARTO_SDK_MATH_HPP
#define KARTO_SDK_MATH_HPP

#include <limits>
#include <cstdint>
#include <cassert>
#include <cmath>

namespace karto
{

/**
 * Lets define a small number!
 */
const double KT_TOLERANCE = 1e-06;

/**
 * Platform independent pi definitions
 */
const double KT_PI_180 = 0.01745329251994329577;
const double KT_PI = 3.14159265358979323846;
const double KT_2PI = 6.28318530717958647692;

/**
 * Lets define max value of kt_int32s (int32_t) to use it to mark invalid scans
 */
const int32_t INVALID_SCAN = std::numeric_limits<int32_t>::max();


namespace math
{
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
        return delta < 0.0 ? delta >= -KT_TOLERANCE : delta <= KT_TOLERANCE;
}

/**
 * Converts degrees into radians
 * @param degrees
 * @return radian equivalent of degrees
 */
inline double DegreesToRadians(double degrees)
{
        return degrees * KT_PI_180;
}

/**
 * Normalizes angle to be in the range of [-pi, pi]
 * @param angle to be normalized
 * @return normalized angle
 */
inline double NormalizeAngle(double angle)
{
        while (angle < -KT_PI) {
                if (angle < -KT_2PI) {
                        angle += (uint32_t)(angle / -KT_2PI) * KT_2PI;
                }
                else {
                        angle += KT_2PI;
                }
        }

        while (angle > KT_PI) {
                if (angle > KT_2PI) {
                        angle -= (uint32_t)(angle / KT_2PI) * KT_2PI;
                } else {
                        angle -= KT_2PI;
                }
        }

        assert(karto::InRange(angle, -KT_PI, KT_PI));

        return angle;
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
        while (minuend - subtrahend < -KT_PI) {
                minuend += KT_2PI;
        }

        while (minuend - subtrahend > KT_PI) {
                minuend -= KT_2PI;
        }

        return minuend;
}

/**
 * Round function
 * @param value
 * @return rounds value to the nearest whole number (as double)
 */
inline double Round(double value)
{
        return value >= 0.0 ? floor(value + 0.5) : ceil(value - 0.5);
}

} // namespace math

} // namespace karto

#endif // MATH_SLAM_HPP