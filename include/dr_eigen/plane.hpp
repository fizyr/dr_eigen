#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dr {

template<typename Scalar, int AmbientDim, int MatrixOptions, int PlaneOptions>
Eigen::Matrix<Scalar, AmbientDim, 1, MatrixOptions> rejection(
	Eigen::Matrix<Scalar, AmbientDim, 1, MatrixOptions> const & vector,
	Eigen::Hyperplane<Scalar, AmbientDim, PlaneOptions> const & plane
) {
	return vector - plane.projection(vector);
}

template<typename Scalar, int AmbientDim, int MatrixOptions, int PlaneOptions>
Eigen::Matrix<Scalar, AmbientDim, 1, MatrixOptions> reflection(
	Eigen::Matrix<Scalar, AmbientDim, 1, MatrixOptions> const & vector,
	Eigen::Hyperplane<Scalar, AmbientDim, PlaneOptions> const & plane
) {
	return vector - 2 * rejection(vector, plane);
}

}
