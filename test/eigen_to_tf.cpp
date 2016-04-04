#include "tf.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(eigenToTf, vector3) {
	tf::Vector3 p1 = toTfVector3(Eigen::Vector3d(0, 1.5, 2));
	tf::Vector3 p2 = toTfVector3(Eigen::Vector3d(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);
}

TEST(eigenToTf, quaternion) {
	tf::Quaternion q1 = toTfQuaternion(Eigen::Quaterniond(0, 1.5, 2, 3.6));
	tf::Quaternion q2 = toTfQuaternion(Eigen::Quaterniond(-1.5, -2.6, -3.7, 4.9));

	ASSERT_NEAR(0.0, q1.w(), 1e-5);
	ASSERT_NEAR(1.5, q1.x(), 1e-5);
	ASSERT_NEAR(2.0, q1.y(), 1e-5);
	ASSERT_NEAR(3.6, q1.z(), 1e-5);
	ASSERT_NEAR(-1.5, q2.w(), 1e-5);
	ASSERT_NEAR(-2.6, q2.x(), 1e-5);
	ASSERT_NEAR(-3.7, q2.y(), 1e-5);
	ASSERT_NEAR(4.9, q2.z(), 1e-5);
}

TEST(eigenToTf, matrix3x3) {
	tf::Matrix3x3 m1 = toTfMatrix3x3((Eigen::Matrix3d() << 0, 1, 2, 3, 4, 5, 6, 7, 8).finished());
	tf::Matrix3x3 m2 = toTfMatrix3x3((Eigen::Matrix3d() << -0.1, 1.3, -2, -3.5, -4.6, -5.7, 6.8, 7.9, 8.1).finished());

	ASSERT_NEAR(0, m1[0].x(), 1e-5);
	ASSERT_NEAR(1, m1[0].y(), 1e-5);
	ASSERT_NEAR(2, m1[0].z(), 1e-5);
	ASSERT_NEAR(3, m1[1].x(), 1e-5);
	ASSERT_NEAR(4, m1[1].y(), 1e-5);
	ASSERT_NEAR(5, m1[1].z(), 1e-5);
	ASSERT_NEAR(6, m1[2].x(), 1e-5);
	ASSERT_NEAR(7, m1[2].y(), 1e-5);
	ASSERT_NEAR(8, m1[2].z(), 1e-5);

	ASSERT_NEAR(-0.1, m2[0].x(), 1e-5);
	ASSERT_NEAR(1.3,  m2[0].y(), 1e-5);
	ASSERT_NEAR(-2,   m2[0].z(), 1e-5);
	ASSERT_NEAR(-3.5, m2[1].x(), 1e-5);
	ASSERT_NEAR(-4.6, m2[1].y(), 1e-5);
	ASSERT_NEAR(-5.7, m2[1].z(), 1e-5);
	ASSERT_NEAR(6.8,  m2[2].x(), 1e-5);
	ASSERT_NEAR(7.9,  m2[2].y(), 1e-5);
	ASSERT_NEAR(8.1,  m2[2].z(), 1e-5);
}

TEST(eigenToTf, transform) {
	tf::Transform transform1 = toTfTransform(Eigen::Translation3d(0, 1.5, 2) * Eigen::Quaterniond(1, 0, 0, 0));
	tf::Transform transform2 = toTfTransform(Eigen::Translation3d(-1.5, -2.6, -3.7) * Eigen::Quaterniond(0, 0, 0, 1));

	tf::Vector3 const & p1 = transform1.getOrigin();
	tf::Vector3 const & p2 = transform2.getOrigin();
	tf::Quaternion const & q1 = transform1.getRotation();
	tf::Quaternion const & q2 = transform2.getRotation();

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);

	ASSERT_NEAR(1, q1.w(), 1e-5);
	ASSERT_NEAR(0, q1.x(), 1e-5);
	ASSERT_NEAR(0, q1.z(), 1e-5);
	ASSERT_NEAR(0, q1.z(), 1e-5);
	ASSERT_NEAR(0, q2.w(), 1e-5);
	ASSERT_NEAR(0, q2.x(), 1e-5);
	ASSERT_NEAR(0, q2.y(), 1e-5);
	ASSERT_NEAR(1, q2.z(), 1e-5);
}


}
