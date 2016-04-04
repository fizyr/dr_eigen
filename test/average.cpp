#include <gtest/gtest.h>

#include "average.hpp"
#include "test/compare.hpp"


using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(AverageQuaternionTest, averageOrientations) {
	// Define orientations
	Eigen::Matrix3d m1, m2;
	m1 = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitX());
	m2 = Eigen::AngleAxisd(0.75*M_PI, Eigen::Vector3d::UnitX());

	// Convert to quaternions
	std::vector<Eigen::Quaterniond> orientations;
	orientations.push_back( Eigen::Quaterniond(m1) );
	orientations.push_back( Eigen::Quaterniond(m2) );
	
	Eigen::Quaterniond actual = averageQuaternions<double>(orientations);
	Eigen::Quaterniond expected(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()));

	ASSERT_TRUE(testNear(Eigen::AngleAxisd(expected), Eigen::AngleAxisd(actual)));
}

TEST(AveragePositionTest, averagePositions) {
	// Define positions
	std::vector<Eigen::Vector3d> positions;
	positions.push_back(Eigen::Vector3d(0.1, 0.2, 0.3));
	positions.push_back(Eigen::Vector3d(-0.6, 0.6, 0.7));

	Eigen::Vector3d actual = averagePositions<double>(positions);
	Eigen::Vector3d expected(-0.25, 0.4, 0.5);

	ASSERT_TRUE(testNear(expected, actual));
}

TEST(AverageIsometryTest, averageIsometries) {
	std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> isometries;
	isometries.push_back(Eigen::Translation<double, 3>(0,  1, 2.5) * Eigen::Quaternion<double>(1, 0, 0, 0));
	isometries.push_back(Eigen::Translation<double, 3>(4, -2, 1  ) * Eigen::Quaternion<double>(0.70711, 0.70711, 0, 0));

	Eigen::Transform<double, 3, Eigen::Isometry> actual   = averageIsometries<double>(isometries);
	Eigen::Transform<double, 3, Eigen::Isometry> expected = Eigen::Translation<double, 3>(2, -0.5, 1.75) * Eigen::Quaternion<double>(0.92388, 0.38268, 0, 0);

	ASSERT_TRUE(testNear(expected, actual));
}

