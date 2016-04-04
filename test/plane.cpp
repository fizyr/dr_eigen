#include "eigen.hpp"
#include "plane.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

namespace {
	double pi = 3.1415926536;
}

TEST(PlaneTest, makePlaneFromVectors) {
	Eigen::Hyperplane<double, 3> plane;

	plane = makePlane({0, 0, 1}, {0, 0, 2});
	ASSERT_TRUE(testEqual(axes::z(), plane.normal()));
	ASSERT_DOUBLE_EQ(-2, plane.offset());

	plane = makePlane({0, 1, 0}, {0, -2, 0});
	ASSERT_TRUE(testEqual(axes::y(), plane.normal()));
	ASSERT_DOUBLE_EQ(2, plane.offset());

	plane = makePlane({-1, 0, 0}, {5, 0, 0});
	ASSERT_TRUE(testEqual(-axes::x(), plane.normal()));
	ASSERT_DOUBLE_EQ(5, plane.offset());
}

TEST(PlaneTest, makeXyPlane) {
	Eigen::Hyperplane<double, 3> plane;

	plane = makeXyPlane(translate(1, 2, 3) * rotateX(0.5 * pi));
	ASSERT_TRUE(testNear(-axes::y(), plane.normal()));
	ASSERT_DOUBLE_EQ(0, plane.signedDistance(Eigen::Vector3d(1, 2, 3)));
}

TEST(PlaneTest, vectorRejection) {
	ASSERT_TRUE(testNear({5, 0, 0}, rejection(Eigen::Vector3d{4, 5, 6}, Eigen::Hyperplane<double, 3>{{1, 0, 0}, 1})));
	ASSERT_TRUE(testNear({0, -3, 0}, rejection(Eigen::Vector3d{4, 5, 6}, Eigen::Hyperplane<double, 3>{{0, -1, 0}, 8})));
	ASSERT_TRUE(testNear({0, 0, 9}, rejection(Eigen::Vector3d{4, 5, 6}, Eigen::Hyperplane<double, 3>{{0, 0, 1}, 3})));
}

TEST(PlaneTest, vectorReflection) {
	Eigen::Hyperplane<double, 3> plane;
	ASSERT_TRUE(testNear({-6, 5, 6}, reflection(Eigen::Vector3d{4, 5, 6}, Eigen::Hyperplane<double, 3>{{1, 0, 0}, 1})));
	ASSERT_TRUE(testNear({4, 11, 6}, reflection(Eigen::Vector3d{4, 5, 6}, Eigen::Hyperplane<double, 3>{{0, -1, 0}, 8})));
	ASSERT_TRUE(testNear({4, 5, -12}, reflection(Eigen::Vector3d{4, 5, 6}, Eigen::Hyperplane<double, 3>{{0, 0, 1}, 3})));
}

}
