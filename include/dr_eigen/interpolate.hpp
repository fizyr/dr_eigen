#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dr {

/// Interpolate linearly between two vectors.
/**
 * At factor 0, the first vector is returned, at factor 1 the second.
 */
template<typename Derived1, typename Derived2>
auto interpolateVector(
	Eigen::MatrixBase<Derived1> const & a, ///< The first vector.
	Eigen::MatrixBase<Derived2> const & b, ///< The second vector.
	double factor                          ///< The interpolation factor.
) -> decltype(a + factor * (b - a)) {
	return a + factor * (b - a);
}

/// Interpolate spherical linearly between two rotations.
/**
 * At factor 0, the first rotation is returned, at factor 1 the second.
 */
template<typename T>
Eigen::AngleAxis<T> interpolateRotation(
	Eigen::AngleAxis<T> const & a, ///< The first rotation.
	Eigen::AngleAxis<T> const & b, ///< The second rotation.
	double factor                  ///< The interpolation factor.
) {
	Eigen::AngleAxis<T> difference = b * a.inverse();
	difference.angle() *= factor;
	return difference * a;
}

/// Interpolate spherical linearly between two rotations.
/**
 * At factor 0, the first rotation is returned, at factor 1 the second.
 */
template<typename T>
Eigen::Quaternion<T> interpolateRotation(
	Eigen::Quaternion<T> const & a, ///< The first rotation.
	Eigen::Quaternion<T> const & b, ///< The second rotation.
	double factor                   ///< The interpolation factor.
) {
	return a.slerp(factor, b);
}

/// Interpolate spherical linearly between two rotations.
/**
 * At factor 0, the first rotation is returned, at factor 1 the second.
 */
template<typename Derived1, typename Derived2>
Eigen::Quaternion<typename Eigen::MatrixBase<Derived1>::Scalar> interpolateRotation(
	Eigen::MatrixBase<Derived1> const & a, ///< The first rotation.
	Eigen::MatrixBase<Derived2> const & b, ///< The second rotation.
	double factor                          ///< The interpolation factor.
) {
	using Scalar1 = typename Eigen::MatrixBase<Derived1>::Scalar;
	using Scalar2 = typename Eigen::MatrixBase<Derived2>::Scalar;
	return interpolateRotation<Scalar1>(Eigen::Quaternion<Scalar1>(a), Eigen::Quaternion<Scalar2>(b), factor);
}

/// Interpolate spherical linearly between two isometries.
/**
 * At factor 0, the first isometry is returned, at factor 1 the second.
 * The translation will be interpolated linearly and the rotation spherial linearly.
 */
Eigen::Isometry3d interpolateIsometry(
	Eigen::Isometry3d const & a, ///< The first isometry.
	Eigen::Isometry3d const & b, ///< The second isometry.
	double factor                ///< The interpolation factor.
) {
	return Eigen::Translation3d{interpolateVector(a.translation(), b.translation(), factor)}
		* interpolateRotation(Eigen::Quaterniond{a.rotation()}, Eigen::Quaterniond{b.rotation()}, factor);
}

}
