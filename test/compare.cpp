#include "eigen.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

/// Test testEqual.
TEST(CompareTest, testEqual) {
	// Test equal is accepted.
	ASSERT_TRUE(testEqual(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1, 2, 3}));

	// Test small difference is rejected.
	ASSERT_FALSE(testEqual(Eigen::Vector3d{1.001, 2, 3}, Eigen::Vector3d{1, 2, 3}));

	// Test same elements different order is rejected.
	ASSERT_FALSE(testEqual(Eigen::Vector3d{3, 2, 1}, Eigen::Vector3d{1, 2, 3}));

	// Test single position differences are rejected.
	ASSERT_FALSE(testEqual(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1, 2, 0}));
	ASSERT_FALSE(testEqual(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1, 0, 3}));
	ASSERT_FALSE(testEqual(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{0, 2, 3}));
}

/// Test testNear without precision argument.
TEST(CompareTest, testNearDefaultPrecision) {
	// Test equal.
	ASSERT_TRUE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1, 2, 3}));

	// Test within precision.
	ASSERT_TRUE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1.0005, 2.0005, 3.0005}));
	ASSERT_TRUE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{0.9995, 1.9995, 2.9995}));
	ASSERT_FALSE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1.0015, 2.0015, 3.0015}));
	ASSERT_FALSE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{0.9985, 1.9985, 2.9985}));


	// Test same elements different order is rejected.
	ASSERT_FALSE(testNear(Eigen::Vector3d{3, 2, 1}, Eigen::Vector3d{1, 2, 3}));

	// Test single position differences are rejected.
	ASSERT_FALSE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1, 2, 0}));
	ASSERT_FALSE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{1, 0, 3}));
	ASSERT_FALSE(testNear(Eigen::Vector3d{1, 2, 3}, Eigen::Vector3d{0, 2, 3}));
}


/// Test testNear with precision argument.
TEST(CompareTest, testNearCustomPrecision) {
	// TODO
}

/// Test testNear for isometries
TEST(CompareTest, testNearDefaultPrecisionIsometry) {
	ASSERT_TRUE(
		testNear(
			Eigen::Isometry3d(Eigen::Translation3d{1, 2, 3} * Eigen::Quaterniond{0.707, 0.707, 0, 0}),
			Eigen::Isometry3d(Eigen::Translation3d{1, 2, 3} * Eigen::Quaterniond{0.707, 0.707, 0, 0})
		)
	);

	ASSERT_FALSE(
		testNear(
			Eigen::Isometry3d(Eigen::Translation3d{1, 2, 3} * Eigen::Quaterniond{1, 0, 0, 0}),
			Eigen::Isometry3d(Eigen::Translation3d{1, 2, 3} * Eigen::Quaterniond{0, 1, 0, 0})
		)
	);

	ASSERT_FALSE(
		testNear(
			Eigen::Isometry3d(Eigen::Translation3d{1, 2, 3} * Eigen::Quaterniond{0, 1, 0, 0}),
			Eigen::Isometry3d(Eigen::Translation3d{0, 2, 3} * Eigen::Quaterniond{0, 1, 0, 0})
		)
	);
}

}
