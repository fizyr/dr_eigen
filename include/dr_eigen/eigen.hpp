#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

namespace dr {

	/// A pose convention.
	struct PoseHeader {
		std::string parent_frame;
		std::string child_frame;
	};

	/// A pose with source and target frame information.
	struct Pose {
		PoseHeader header;
		Eigen::Isometry3d isometry;
	};

	/// Elementary axes.
	namespace axes {
		/// Get a vector representing the X axis.
		inline Eigen::Vector3d x() { return Eigen::Vector3d::UnitX(); }

		/// Get a vector representing the Y axis.
		inline Eigen::Vector3d y() { return Eigen::Vector3d::UnitY(); }

		/// Get a vector representing the Z axis.
		inline Eigen::Vector3d z() { return Eigen::Vector3d::UnitZ(); }
	}

	/// Create an aligned box with a center and dimensions.
	inline Eigen::AlignedBox3d makeCenteredBox(Eigen::Vector3d const & center, Eigen::Vector3d const & size) {
		return Eigen::AlignedBox3d(center - size / 2, center + size / 2);
	}

	/// Create a hyperplane with a position and normal.
	inline Eigen::Hyperplane<double, 3> makePlane(Eigen::Vector3d const & normal, Eigen::Vector3d const & point) {
		return Eigen::Hyperplane<double, 3>(normal, point);
	}

	/// Create a hyperplane from a pose.
	inline Eigen::Hyperplane<double, 3> makeXyPlane(Eigen::Isometry3d const & pose) {
		return Eigen::Hyperplane<double, 3>(pose.rotation() * axes::z(), pose.translation());
	}

	/// Create a translation from a vector.
	inline Eigen::Translation3d translate(Eigen::Vector3d translation) {
		return Eigen::Translation3d{translation};
	}

	/// Create a translation from X, Y and Z components.
	inline Eigen::Translation3d translate(double x, double y, double z) {
		return translate(Eigen::Vector3d{x, y, z});
	}

	/// Create a rotation with a given angle and axis.
	inline Eigen::AngleAxisd rotate(double angle, Eigen::Vector3d const & axis) {
		return Eigen::AngleAxisd{angle, axis};
	}

	/// Create a rotation with a given angle and axis and center of rotation.
	inline Eigen::Isometry3d rotate(double angle, Eigen::Vector3d const & axis, Eigen::Vector3d const & pivot_point) {
		return dr::translate(pivot_point) * Eigen::AngleAxisd{angle, axis} * dr::translate(-pivot_point);
	}

	/// Create a rotation around the X axis with a given angle.
	inline Eigen::AngleAxisd rotateX(double angle) {
		return dr::rotate(angle, axes::x());
	}

	/// Create a rotation around the X axis with a given angle and center of rotation.
	inline Eigen::Isometry3d rotateX(double angle, Eigen::Vector3d const & pivot_point) {
		return dr::rotate(angle, axes::x(), pivot_point);
	}

	/// Create a rotation around the Y axis with a given angle.
	inline Eigen::AngleAxisd rotateY(double angle) {
		return dr::rotate(angle, axes::y());
	}

	/// Create a rotation around the Y axis with a given angle and center of rotation.
	inline Eigen::Isometry3d rotateY(double angle, Eigen::Vector3d const & pivot_point) {
		return dr::rotate(angle, axes::y(), pivot_point);
	}

	/// Create a rotation around the Z axis with a given angle.
	inline Eigen::AngleAxisd rotateZ(double angle) {
		return dr::rotate(angle, axes::z());
	}

	/// Create a rotation around the Z axis with a given angle and center of rotation.
	inline Eigen::Isometry3d rotateZ(double angle, Eigen::Vector3d const & pivot_point) {
		return dr::rotate(angle, axes::z(), pivot_point);
	}

	/// Project vector a onto b.
	/**
	 * \return The projection of a onto b.
	 */
	template<typename A, typename B>
	auto projection(
		A const & a, ///< Vector a.
		B const & b  ///< Vector b.
	) -> decltype((a.dot(b) / b.dot(b)) * b) {
		return (a.dot(b) / b.dot(b)) * b;
	}

	/// Get the rejection of vector a onto b.
	/**
	 * \return The rejection of a onto b.
	 */
	template<typename A, typename B>
	auto rejection(
		A const & a, ///< Vector a.
		B const & b  ///< Vector b.
	) -> decltype(a - projection(a, b)) {
		return a - projection(a, b);
	}
}
