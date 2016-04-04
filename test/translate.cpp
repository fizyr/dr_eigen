#include "eigen.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(TranslateTest, vectorRepresentation) {
	ASSERT_EQ(Eigen::Vector3d(0, 0, 0), translate(0, 0, 0).translation());
	ASSERT_EQ(Eigen::Vector3d(1, 0, 0), translate(1, 0, 0).translation());
	ASSERT_EQ(Eigen::Vector3d(0, 1, 0), translate(0, 1, 0).translation());
	ASSERT_EQ(Eigen::Vector3d(0, 0, 1), translate(0, 0, 1).translation());

	ASSERT_EQ(Eigen::Vector3d(-1, 0, 0), translate(-1, 0, 0).translation());
	ASSERT_EQ(Eigen::Vector3d(0, -1, 0), translate(0, -1, 0).translation());
	ASSERT_EQ(Eigen::Vector3d(0, 0, -1), translate(0, 0, -1).translation());
}

}
