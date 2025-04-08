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

} // namespace math

#endif // MATH_SLAM_HPP