#ifndef CERES_UTILS_HPP
#define CERES_UTILS_HPP

#include <ceres/ceres.h>
#include <ceres/autodiff_manifold.h>

/*****************************************************************************/
inline std::size_t GetHash(const int &x, const int &y)
/*****************************************************************************/
{
        return (std::hash<double>()(x) ^ (std::hash<double>()(y) << 1)) >> 1;
}


/*****************************************************************************/
// Normalizes the angle in radians between [-pi and pi).
template <typename T>
inline T NormalizeAngle(const T &angle_radians)
/*****************************************************************************/
{
        T two_pi(2.0 * M_PI);
        return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/


// Defines a manifold for updating the angle to be constrained in [-pi to pi).
class AngleManifold
{
public:
        template <typename T>
        bool Plus(const T *x_radians,
                  const T *delta_radians,
                  T *x_plus_delta_radians) const
        {
                *x_plus_delta_radians = NormalizeAngle(*x_radians + *delta_radians);
                return true;
        }

        template <typename T>
        bool Minus(const T *y_radians,
                   const T *x_radians,
                   T *y_minus_x_radians) const
        {
                *y_minus_x_radians = NormalizeAngle(*y_radians - *x_radians);
                return true;
        }

        static ceres::Manifold *Create()
        {
                return new ceres::AutoDiffManifold<AngleManifold, 1, 1>;
        }
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians)
/*****************************************************************************/
{
        const T cos_yaw = ceres::cos(yaw_radians);
        const T sin_yaw = ceres::sin(yaw_radians);
        Eigen::Matrix<T, 2, 2> rotation;
        rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
        return rotation;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

class PoseGraph2dErrorTerm
{
public:
        PoseGraph2dErrorTerm(
            double x_ab, double y_ab, double yaw_ab_radians,
            const Eigen::Matrix3d &sqrt_information)
            : p_ab_(x_ab, y_ab), yaw_ab_radians_(yaw_ab_radians), sqrt_information_(sqrt_information)
        {
        }

        template <typename T>
        bool operator()(
            const T *const x_a, const T *const y_a, const T *const yaw_a,
            const T *const x_b, const T *const y_b, const T *const yaw_b,
            T *residuals_ptr) const
        {
                const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
                const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);
                Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);
                residuals_map.template head<2>() = RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
                residuals_map(2) = NormalizeAngle((*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));
                // Scale the residuals by the square root information
                // matrix to account for the measurement uncertainty.
                residuals_map = sqrt_information_.template cast<T>() * residuals_map;
                return true;
        }

        static ceres::CostFunction *Create(
            double x_ab, double y_ab, double yaw_ab_radians,
            const Eigen::Matrix3d &sqrt_information)
        {
                return new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
                    new PoseGraph2dErrorTerm(
                        x_ab, y_ab, yaw_ab_radians,
                        sqrt_information));
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
        // The position of B relative to A in the A frame.
        const Eigen::Vector2d p_ab_;
        // The orientation of frame B relative to frame A.
        const double yaw_ab_radians_;
        // The inverse square root of the measurement covariance matrix.
        const Eigen::Matrix3d sqrt_information_;
};

#endif // CERES_UTILS_HPP