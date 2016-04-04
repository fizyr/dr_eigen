#include "eigen.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(AxesTest, unitAxes) {
	ASSERT_EQ(Eigen::Vector3d::UnitX(), axes::x());
	ASSERT_EQ(Eigen::Vector3d::UnitY(), axes::y());
	ASSERT_EQ(Eigen::Vector3d::UnitZ(), axes::z());

	ASSERT_EQ(Eigen::Vector3d(1, 0, 0), axes::x());
	ASSERT_EQ(Eigen::Vector3d(0, 1, 0), axes::y());
	ASSERT_EQ(Eigen::Vector3d(0, 0, 1), axes::z());
}

}
