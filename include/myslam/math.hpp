#ifndef MATH_SLAM_HPP
#define MATH_SLAM_HPP

#include <cstddef>

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

} // namespace math

#endif // MATH_SLAM_HPP