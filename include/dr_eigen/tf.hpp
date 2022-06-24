// Copyright 2014-2022, Fizyr B.V.

#pragma once
#include "eigen.hpp"

#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <string>


namespace dr {


/// Convert a TF vector to an Eigen vector.
inline Eigen::Vector3d toEigen(tf::Vector3 const & vector) {
	return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

/// Convert a TF quaternion to an Eigen Quaternion.
inline Eigen::Quaterniond toEigen(tf::Quaternion const & quaternion) {
	return Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}

/// Convert a TF transform to an Eigen isometry.
inline Eigen::Isometry3d toEigen(tf::Transform const & transform) {
	return Eigen::Isometry3d(translate(toEigen(transform.getOrigin())) * toEigen(transform.getRotation()));
}

/// Convert a TF transform to an Eigen isometry.
inline Eigen::Matrix3d toEigen(tf::Matrix3x3 const & matrix) {
	Eigen::Matrix3d result;
	result <<
		matrix[0][0], matrix[0][1], matrix[0][2],
		matrix[1][0], matrix[1][1], matrix[1][2],
		matrix[2][0], matrix[2][1], matrix[2][2];
	return result;
}

/// Convert a TF vector to an Eigen vector.
inline tf::Vector3 toTfVector3(Eigen::Vector3d const & vector) {
	return tf::Vector3(vector.x(), vector.y(), vector.z());
}

/// Convert a TF quaternion to an Eigen Quaternion.
inline tf::Quaternion toTfQuaternion(Eigen::Quaterniond const & quaternion) {
	return tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
}

/// Convert a TF quaternion to an Eigen Quaternion.
inline tf::Matrix3x3 toTfMatrix3x3(Eigen::Matrix3d const & matrix) {
	return tf::Matrix3x3(
		matrix(0, 0), matrix(0, 1), matrix(0, 2),
		matrix(1, 0), matrix(1, 1), matrix(1, 2),
		matrix(2, 0), matrix(2, 1), matrix(2, 2)
	);
}

/// Convert a TF transform to an Eigen isometry.
inline tf::Transform toTfTransform(Eigen::Isometry3d const & transform) {
	return tf::Transform(
		toTfMatrix3x3(transform.rotation()),
		toTfVector3(transform.translation())
	);
}

inline tf::StampedTransform toTfStampedTransform(
	Eigen::Isometry3d const & transform,
	std::string const & parent_frame,
	std::string const & child_frame,
	ros::Time const & time = ros::Time::now()
) {
	return tf::StampedTransform(toTfTransform(transform), time, parent_frame, child_frame);
}

}
