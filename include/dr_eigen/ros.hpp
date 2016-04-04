#pragma once
#include "eigen.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

namespace dr {

/// Convert a ROS Point to an Eigen vector.
inline Eigen::Vector3d toEigen(geometry_msgs::Point const & vector) {
	return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

/// Convert a ROS Point32 to an Eigen vector.
inline Eigen::Vector3f toEigen(geometry_msgs::Point32 const & vector) {
	return Eigen::Vector3f(vector.x, vector.y, vector.z);
}

/// Convert a ROS Vector3 to an Eigen vector.
inline Eigen::Vector3d toEigen(geometry_msgs::Vector3 const & vector) {
	return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

/// Convert a ROS Quaternion to an Eigen quaternion.
inline Eigen::Quaterniond toEigen(geometry_msgs::Quaternion const & quaternion) {
	return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

/// Convert a ROS Pose to an Eigen isometry.
inline Eigen::Isometry3d toEigen(geometry_msgs::Pose const & pose) {
	return translate(toEigen(pose.position)) * toEigen(pose.orientation);
}

/// Convert a ROS transform to an Eigen isometry.
inline Eigen::Isometry3d toEigen(geometry_msgs::Transform const & transform) {
	return translate(toEigen(transform.translation)) * toEigen(transform.rotation);
}

/// Convert an Eigen vector to a ROS Point.
inline geometry_msgs::Point toRosPoint(Eigen::Vector3d const & vector) {
	geometry_msgs::Point result;
	result.x = vector.x();
	result.y = vector.y();
	result.z = vector.z();
	return result;
}

/// Convert an Eigen vector to a ROS Point32.
inline geometry_msgs::Point32 toRosPoint32(Eigen::Vector3f const & vector) {
	geometry_msgs::Point32 result;
	result.x = vector.x();
	result.y = vector.y();
	result.z = vector.z();
	return result;
}

/// Convert an Eigen vector to a ROS Vector3.
inline geometry_msgs::Vector3 toRosVector3(Eigen::Vector3d const & vector) {
	geometry_msgs::Vector3 result;
	result.x = vector.x();
	result.y = vector.y();
	result.z = vector.z();
	return result;
}

/// Convert an Eigen quaternion to a ROS Quaternion.
inline geometry_msgs::Quaternion toRosQuaternion(Eigen::Quaterniond const & quaternion) {
	geometry_msgs::Quaternion result;
	result.w = quaternion.w();
	result.x = quaternion.x();
	result.y = quaternion.y();
	result.z = quaternion.z();
	return result;
}

/// Convert an Eigen angle axis to a ROS Quaternion.
inline geometry_msgs::Quaternion toRosQuaternion(Eigen::AngleAxisd const & angle_axis) {
	return toRosQuaternion(Eigen::Quaterniond(angle_axis));
}

/// Convert an Eigen isometry to a ROS Pose.
inline geometry_msgs::Pose toRosPose(Eigen::Isometry3d const & pose) {
	geometry_msgs::Pose result;
	result.position    = toRosPoint(pose.translation());
	result.orientation = toRosQuaternion(Eigen::Quaterniond(pose.rotation()));
	return result;
}

inline geometry_msgs::PoseStamped toRosPoseStamped(
	Eigen::Isometry3d const & pose, std::string frame_id, ros::Time const & time = ros::Time::now()) {
	geometry_msgs::PoseStamped result;
	result.header.frame_id  = frame_id;
	result.header.stamp     = time;
	result.pose.position    = toRosPoint(pose.translation());
	result.pose.orientation = toRosQuaternion(Eigen::Quaterniond(pose.rotation()));
	return result;
}

/// Convert an Eigen isometry to a ROS Transform.
inline geometry_msgs::Transform toRosTransform(Eigen::Isometry3d const & transform) {
	geometry_msgs::Transform result;
	result.translation = toRosVector3(transform.translation());
	result.rotation    = toRosQuaternion(Eigen::Quaterniond(transform.rotation()));
	return result;
}

}
