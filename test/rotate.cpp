#include "eigen.hpp"

#include <gtest/gtest.h>

#include "test/compare.hpp"

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

// Test simple rotations.
TEST(RotationTest, angleAxis) {
	Eigen::AngleAxisd rotation;

	rotation = rotateX(1);
	ASSERT_EQ(1, rotation.angle());
	ASSERT_TRUE(testEqual(axes::x(), rotation.axis()));

	rotation = rotateY(2);
	ASSERT_EQ(2, rotation.angle());
	ASSERT_TRUE(testEqual(axes::y(), rotation.axis()));

	rotation = rotateZ(3);
	ASSERT_EQ(3, rotation.angle());
	ASSERT_TRUE(testEqual(axes::z(), rotation.axis()));

	rotation = rotate(4, {1, 2, 3});
	ASSERT_EQ(4, rotation.angle());
	ASSERT_TRUE(testEqual({1, 2, 3}, rotation.axis()));
}

// Test rotation with pivot point.
TEST(RotationTest, rotatePivot) {
	ASSERT_TRUE(testNear(Eigen::Vector3d{1, 2, 3}, rotate(0.3 * M_PI, axes::z(), Eigen::Vector3d{1, 2, 3}) * Eigen::Vector3d{1, 2, 3}));
	ASSERT_TRUE(testNear(Eigen::Vector3d{0, 1, 1}, rotate(0.5 * M_PI, axes::x(), Eigen::Vector3d{0, 1, 0}) * Eigen::Vector3d{0, 2, 0}));
	ASSERT_TRUE(testNear(Eigen::Vector3d{1, 0, 1}, rotate(0.5 * M_PI, axes::y(), Eigen::Vector3d{0, 0, 1}) * Eigen::Vector3d{0, 0, 2}));
	ASSERT_TRUE(testNear(Eigen::Vector3d{1, 1, 0}, rotate(0.5 * M_PI, axes::z(), Eigen::Vector3d{1, 0, 0}) * Eigen::Vector3d{2, 0, 0}));
}

TEST(RotationTest, rotateAroundAxis) {
	ASSERT_TRUE(testNear(Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()), rotateX(0.25 * M_PI), 0.001));
	ASSERT_TRUE(testNear(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY()), rotateY(-0.5 * M_PI), 0.001));
	ASSERT_TRUE(testNear(Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitZ()), rotateZ(0.75 * M_PI), 0.001));
}

}
