#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace dr {

/// Calculate average orientation using quaternions
template<typename DataType, typename ForwardIterator>
Eigen::Quaternion<DataType> averageQuaternions(ForwardIterator const & begin, ForwardIterator const & end) {

	if (begin == end) {
		throw std::logic_error("Cannot average orientations over an empty range.");
	}

	Eigen::Matrix<DataType, 4, 4> A = Eigen::Matrix<DataType, 4, 4>::Zero();
	uint sum(0);
	for (ForwardIterator it = begin; it != end; ++it) {
		Eigen::Matrix<DataType, 1, 4> q(1,4);
		q(0) = it->w();
		q(1) = it->x();
		q(2) = it->y();
		q(3) = it->z();
		A += q.transpose()*q;
		sum++;
	}
	A /= sum;

	Eigen::EigenSolver<Eigen::Matrix<DataType, 4, 4>> es(A);

	Eigen::Matrix<std::complex<DataType>, 4, 1> mat(es.eigenvalues());
	int index;
	mat.real().maxCoeff(&index);
	Eigen::Matrix<DataType, 4, 1> largest_ev(es.eigenvectors().real().block(0, index, 4, 1));

	return Eigen::Quaternion<DataType>(largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));
}

/// Overloaded function to calculate the average quaternion for some container types
template<typename DataType, typename Container>
typename Eigen::Quaternion<DataType> averageQuaternions(Container const & container) {
	return averageQuaternions<DataType>(container.begin(), container.end());
}

/// Calculate average position
template<typename DataType, typename ForwardIterator>
Eigen::Matrix<DataType, 3, 1> averagePositions(ForwardIterator const & begin, ForwardIterator const & end) {
	if (begin == end) {
		throw std::logic_error("Cannot average orientations over an empty range.");
	}

	DataType x(0), y(0), z(0);
	uint count(0);
	for (ForwardIterator it = begin; it!=end; ++it) {
		x += it->x();
		y += it->y();
		z += it->z();
		count++;
	}
	x /= count;
	y /= count;
	z /= count;

	return Eigen::Matrix<DataType, 3, 1>(x,y,z);
}

/// Overloaded function to calculate the average position for some container types
template<typename DataType, typename Container>
typename Eigen::Matrix<DataType, 3, 1> averagePositions(Container const & container) {
	return averagePositions<DataType>(container.begin(), container.end());
}

template<typename DataType, typename ForwardIterator>
Eigen::Transform<DataType, 3, Eigen::Isometry> averageIsometries(ForwardIterator const & begin, ForwardIterator const & end) {

	std::vector<Eigen::Vector3d> positions;
	std::vector<Eigen::Quaterniond> quaternions;

	for (auto it = begin; it != end; ++it) {
		positions.push_back(it->translation());
		quaternions.push_back(Eigen::Quaternion<DataType>(it->rotation()));
	}

	return Eigen::Translation<DataType, 3>(averagePositions<DataType>(positions)) * averageQuaternions<DataType>(quaternions);
}

template<typename DataType, typename Container>
Eigen::Transform<DataType, 3, Eigen::Isometry> averageIsometries(Container const & container) {
	return averageIsometries<DataType>(container.begin(), container.end());
}

}
