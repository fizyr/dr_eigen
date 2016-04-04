#include "tf.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(tfToEigen, vector3) {
	Eigen::Vector3d p1 = toEigen(tf::Vector3(0, 1.5, 2));
	Eigen::Vector3d p2 = toEigen(tf::Vector3(-1.5, -2.6, -3.7));

	ASSERT_NEAR(0.0, p1.x(), 1e-5);
	ASSERT_NEAR(1.5, p1.y(), 1e-5);
	ASSERT_NEAR(2.0, p1.z(), 1e-5);
	ASSERT_NEAR(-1.5, p2.x(), 1e-5);
	ASSERT_NEAR(-2.6, p2.y(), 1e-5);
	ASSERT_NEAR(-3.7, p2.z(), 1e-5);
}

TEST(tfToEigen, quaternion) {
	Eigen::Quaterniond q1 = toEigen(tf::Quaternion(0, 1.5, 2, 3.6));
	Eigen::Quaterniond q2 = toEigen(tf::Quaternion(-1.5, -2.6, -3.7, 4.9));

	ASSERT_NEAR(0.0, q1.x(), 1e-5);
	ASSERT_NEAR(1.5, q1.y(), 1e-5);
	ASSERT_NEAR(2.0, q1.z(), 1e-5);
	ASSERT_NEAR(3.6, q1.w(), 1e-5);
	ASSERT_NEAR(-1.5, q2.x(), 1e-5);
	ASSERT_NEAR(-2.6, q2.y(), 1e-5);
	ASSERT_NEAR(-3.7, q2.z(), 1e-5);
	ASSERT_NEAR(4.9, q2.w(), 1e-5);
}

TEST(tfToEigen, matrix3x3) {
	Eigen::Matrix3d m1 = toEigen(tf::Matrix3x3(0, 1, 2, 3, 4, 5, 6, 7, 8));
	Eigen::Matrix3d m2 = toEigen(tf::Matrix3x3(-0.1, 1.3, -2, -3.5, -4.6, -5.7, 6.8, 7.9, 8.1));

	ASSERT_NEAR(0, m1(0, 0), 1e-5);
	ASSERT_NEAR(1, m1(0, 1), 1e-5);
	ASSERT_NEAR(2, m1(0, 2), 1e-5);
	ASSERT_NEAR(3, m1(1, 0), 1e-5);
	ASSERT_NEAR(4, m1(1, 1), 1e-5);
	ASSERT_NEAR(5, m1(1, 2), 1e-5);
	ASSERT_NEAR(6, m1(2, 0), 1e-5);
	ASSERT_NEAR(7, m1(2, 1), 1e-5);
	ASSERT_NEAR(8, m1(2, 2), 1e-5);

	ASSERT_NEAR(-0.1, m2(0, 0), 1e-5);
	ASSERT_NEAR(1.3, m2(0, 1), 1e-5);
	ASSERT_NEAR(-2, m2(0, 2), 1e-5);
	ASSERT_NEAR(-3.5, m2(1, 0), 1e-5);
	ASSERT_NEAR(-4.6, m2(1, 1), 1e-5);
	ASSERT_NEAR(-5.7, m2(1, 2), 1e-5);
	ASSERT_NEAR(6.8, m2(2, 0), 1e-5);
	ASSERT_NEAR(7.9, m2(2, 1), 1e-5);
	ASSERT_NEAR(8.1, m2(2, 2), 1e-5);
}

TEST(tfToEigen, transform) {
	Eigen::Isometry3d transform1 = toEigen(tf::Transform(tf::Quaternion(1, 0, 0, 0), tf::Vector3(0, 1.5, 2)));
	Eigen::Isometry3d transform2 = toEigen(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1.5, -2.6, -3.7)));

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
