#include "ros.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(eigenToRos, point) {
	geometry_msgs::Point p1 = toRosPoint(Eigen::Vector3d(0, 1.5, 2));
	geometry_msgs::Point p2 = toRosPoint(Eigen::Vector3d(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x, 1e-5);
	ASSERT_NEAR(1.5, p1.y, 1e-5);
	ASSERT_NEAR(2.0, p1.z, 1e-5);
	ASSERT_NEAR(-1.5, p2.x, 1e-5);
	ASSERT_NEAR(-2.6, p2.y, 1e-5);
	ASSERT_NEAR(-3.7, p2.z, 1e-5);
}

TEST(eigenToRos, point32) {
	geometry_msgs::Point32 p1 = toRosPoint32(Eigen::Vector3f(0, 1.5, 2));
	geometry_msgs::Point32 p2 = toRosPoint32(Eigen::Vector3f(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x, 1e-5);
	ASSERT_NEAR(1.5, p1.y, 1e-5);
	ASSERT_NEAR(2.0, p1.z, 1e-5);
	ASSERT_NEAR(-1.5, p2.x, 1e-5);
	ASSERT_NEAR(-2.6, p2.y, 1e-5);
	ASSERT_NEAR(-3.7, p2.z, 1e-5);
}

TEST(eigenToRos, vector3) {
	geometry_msgs::Vector3 p1 = toRosVector3(Eigen::Vector3d(0, 1.5, 2));
	geometry_msgs::Vector3 p2 = toRosVector3(Eigen::Vector3d(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x, 1e-5);
	ASSERT_NEAR(1.5, p1.y, 1e-5);
	ASSERT_NEAR(2.0, p1.z, 1e-5);
	ASSERT_NEAR(-1.5, p2.x, 1e-5);
	ASSERT_NEAR(-2.6, p2.y, 1e-5);
	ASSERT_NEAR(-3.7, p2.z, 1e-5);
}

TEST(eigenToRos, quaternion) {
	geometry_msgs::Quaternion q1 = toRosQuaternion(Eigen::Quaterniond(0, 1.5, 2, 3.6));
	geometry_msgs::Quaternion q2 = toRosQuaternion(Eigen::Quaterniond(-1.5, -2.6, -3.7, 4.9));

	ASSERT_NEAR(0.0, q1.w, 1e-5);
	ASSERT_NEAR(1.5, q1.x, 1e-5);
	ASSERT_NEAR(2.0, q1.y, 1e-5);
	ASSERT_NEAR(3.6, q1.z, 1e-5);
	ASSERT_NEAR(-1.5, q2.w, 1e-5);
	ASSERT_NEAR(-2.6, q2.x, 1e-5);
	ASSERT_NEAR(-3.7, q2.y, 1e-5);
	ASSERT_NEAR(4.9, q2.z, 1e-5);
}

TEST(eigenToRos, pose) {
	geometry_msgs::Pose pose1 = toRosPose(Eigen::Translation3d(0, 1.5, 2) * Eigen::Quaterniond(1, 0, 0, 0));
	geometry_msgs::Pose pose2 = toRosPose(Eigen::Translation3d(-1.5, -2.6, -3.7) * Eigen::Quaterniond(0, 0, 0, 1));

	geometry_msgs::Point const & p1 = pose1.position;
	geometry_msgs::Point const & p2 = pose2.position;
	geometry_msgs::Quaternion const & q1 = pose1.orientation;
	geometry_msgs::Quaternion const & q2 = pose2.orientation;

	ASSERT_NEAR(0.0, p1.x, 1e-5);
	ASSERT_NEAR(1.5, p1.y, 1e-5);
	ASSERT_NEAR(2.0, p1.z, 1e-5);
	ASSERT_NEAR(-1.5, p2.x, 1e-5);
	ASSERT_NEAR(-2.6, p2.y, 1e-5);
	ASSERT_NEAR(-3.7, p2.z, 1e-5);

	ASSERT_NEAR(1, q1.w, 1e-5);
	ASSERT_NEAR(0, q1.x, 1e-5);
	ASSERT_NEAR(0, q1.z, 1e-5);
	ASSERT_NEAR(0, q1.z, 1e-5);
	ASSERT_NEAR(0, q2.w, 1e-5);
	ASSERT_NEAR(0, q2.x, 1e-5);
	ASSERT_NEAR(0, q2.y, 1e-5);
	ASSERT_NEAR(1, q2.z, 1e-5);
}

TEST(eigenToRos, transform) {
	geometry_msgs::Transform transform1 = toRosTransform(Eigen::Translation3d(0, 1.5, 2) * Eigen::Quaterniond(1, 0, 0, 0));
	geometry_msgs::Transform transform2 = toRosTransform(Eigen::Translation3d(-1.5, -2.6, -3.7) * Eigen::Quaterniond(0, 0, 0, 1));

	geometry_msgs::Vector3 const & p1 = transform1.translation;
	geometry_msgs::Vector3 const & p2 = transform2.translation;
	geometry_msgs::Quaternion const & q1 = transform1.rotation;
	geometry_msgs::Quaternion const & q2 = transform2.rotation;

	ASSERT_NEAR(0.0, p1.x, 1e-5);
	ASSERT_NEAR(1.5, p1.y, 1e-5);
	ASSERT_NEAR(2.0, p1.z, 1e-5);
	ASSERT_NEAR(-1.5, p2.x, 1e-5);
	ASSERT_NEAR(-2.6, p2.y, 1e-5);
	ASSERT_NEAR(-3.7, p2.z, 1e-5);

	ASSERT_NEAR(1, q1.w, 1e-5);
	ASSERT_NEAR(0, q1.x, 1e-5);
	ASSERT_NEAR(0, q1.z, 1e-5);
	ASSERT_NEAR(0, q1.z, 1e-5);
	ASSERT_NEAR(0, q2.w, 1e-5);
	ASSERT_NEAR(0, q2.x, 1e-5);
	ASSERT_NEAR(0, q2.y, 1e-5);
	ASSERT_NEAR(1, q2.z, 1e-5);

}


}
