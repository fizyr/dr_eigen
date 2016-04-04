#include "ros.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

namespace {
	geometry_msgs::Point makePoint(double x, double y, double z) {
		geometry_msgs::Point result;
		result.x = x;
		result.y = y;
		result.z = z;
		return result;
	}

	geometry_msgs::Point32 makePoint32(float x, float y, float z) {
		geometry_msgs::Point32 result;
		result.x = x;
		result.y = y;
		result.z = z;
		return result;
	}

	geometry_msgs::Vector3 makeVector3(double x, double y, double z) {
		geometry_msgs::Vector3 result;
		result.x = x;
		result.y = y;
		result.z = z;
		return result;
	}

	geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w) {
		geometry_msgs::Quaternion result;
		result.x = x;
		result.y = y;
		result.z = z;
		result.w = w;
		return result;
	}

	geometry_msgs::Pose makePose(geometry_msgs::Point const & position, geometry_msgs::Quaternion orientation) {
		geometry_msgs::Pose result;
		result.position    = position;
		result.orientation = orientation;
		return result;
	}

	geometry_msgs::Transform makeTransform(geometry_msgs::Vector3 const & position, geometry_msgs::Quaternion orientation) {
		geometry_msgs::Transform result;
		result.translation = position;
		result.rotation    = orientation;
		return result;
	}
}

TEST(rosToEigen, point) {
	Eigen::Vector3d p1 = toEigen(makePoint(0, 1.5, 2));
	Eigen::Vector3d p2 = toEigen(makePoint(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);
}

TEST(rosToEigen, point32) {
	Eigen::Vector3f p1 = toEigen(makePoint32(0, 1.5, 2));
	Eigen::Vector3f p2 = toEigen(makePoint32(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);
}

TEST(rosToEigen, vector3) {
	Eigen::Vector3d p1 = toEigen(makeVector3(0, 1.5, 2));
	Eigen::Vector3d p2 = toEigen(makeVector3(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);
}

TEST(rosToEigen, quaternion) {
	Eigen::Quaterniond q1 = toEigen(makeQuaternion(0, 1.5, 2, 3.6));
	Eigen::Quaterniond q2 = toEigen(makeQuaternion(-1.5, -2.6, -3.7, 4.9));

	ASSERT_NEAR(0.0, q1.x(), 1e-5);
	ASSERT_NEAR(1.5, q1.y(), 1e-5);
	ASSERT_NEAR(2.0, q1.z(), 1e-5);
	ASSERT_NEAR(3.6, q1.w(), 1e-5);
	ASSERT_NEAR(-1.5, q2.x(), 1e-5);
	ASSERT_NEAR(-2.6, q2.y(), 1e-5);
	ASSERT_NEAR(-3.7, q2.z(), 1e-5);
	ASSERT_NEAR(4.9, q2.w(), 1e-5);
}

TEST(rosToEigen, pose) {
	Eigen::Isometry3d pose1 = toEigen(makePose(makePoint(0, 1.5, 2), makeQuaternion(1, 0, 0, 0)));
	Eigen::Isometry3d pose2 = toEigen(makePose(makePoint(-1.5, -2.6, -3.7), makeQuaternion(0, 0, 0, 1)));

	Eigen::Vector3d const & p1 = pose1.translation();
	Eigen::Vector3d const & p2 = pose2.translation();
	Eigen::Quaterniond const & q1 = Eigen::Quaterniond{pose1.rotation()};
	Eigen::Quaterniond const & q2 = Eigen::Quaterniond{pose2.rotation()};

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);

	ASSERT_NEAR(1, q1.x(), 1e-5);
	ASSERT_NEAR(0, q1.y(), 1e-5);
	ASSERT_NEAR(0, q1.z(), 1e-5);
	ASSERT_NEAR(0, q1.w(), 1e-5);
	ASSERT_NEAR(0, q2.x(), 1e-5);
	ASSERT_NEAR(0, q2.y(), 1e-5);
	ASSERT_NEAR(0, q2.z(), 1e-5);
	ASSERT_NEAR(1, q2.w(), 1e-5);
}

TEST(rosToEigen, transform) {
	Eigen::Isometry3d transform1 = toEigen(makeTransform(makeVector3(0, 1.5, 2), makeQuaternion(1, 0, 0, 0)));
	Eigen::Isometry3d transform2 = toEigen(makeTransform(makeVector3(-1.5, -2.6, -3.7), makeQuaternion(0, 0, 0, 1)));

	Eigen::Vector3d const & p1 = transform1.translation();
	Eigen::Vector3d const & p2 = transform2.translation();
	Eigen::Quaterniond const & q1 = Eigen::Quaterniond{transform1.rotation()};
	Eigen::Quaterniond const & q2 = Eigen::Quaterniond{transform2.rotation()};

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);

	ASSERT_NEAR(1, q1.x(), 1e-5);
	ASSERT_NEAR(0, q1.y(), 1e-5);
	ASSERT_NEAR(0, q1.z(), 1e-5);
	ASSERT_NEAR(0, q1.w(), 1e-5);
	ASSERT_NEAR(0, q2.x(), 1e-5);
	ASSERT_NEAR(0, q2.y(), 1e-5);
	ASSERT_NEAR(0, q2.z(), 1e-5);
	ASSERT_NEAR(1, q2.w(), 1e-5);
}


}
