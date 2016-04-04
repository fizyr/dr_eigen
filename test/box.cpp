#include "eigen.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(BoxTest, makeCenteredBox) {
	Eigen::AlignedBox3d box;
	box = makeCenteredBox({1, 2, 3}, {4, 5, 6});

	ASSERT_EQ(Eigen::Vector3d( 1,  2.0, 3), box.center());

	ASSERT_TRUE(testEqual(Eigen::Vector3d(-1, -0.5, 0), box.corner(Eigen::AlignedBox3d::BottomLeftFloor)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d(-1, -0.5, 6), box.corner(Eigen::AlignedBox3d::BottomLeftCeil)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d(-1,  4.5, 0), box.corner(Eigen::AlignedBox3d::TopLeftFloor)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d(-1,  4.5, 6), box.corner(Eigen::AlignedBox3d::TopLeftCeil)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d( 3, -0.5, 0), box.corner(Eigen::AlignedBox3d::BottomRightFloor)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d( 3, -0.5, 6), box.corner(Eigen::AlignedBox3d::BottomRightCeil)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d( 3,  4.5, 0), box.corner(Eigen::AlignedBox3d::TopRightFloor)));
	ASSERT_TRUE(testEqual(Eigen::Vector3d( 3,  4.5, 6), box.corner(Eigen::AlignedBox3d::TopRightCeil)));
}
