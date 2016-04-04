#include "yaml.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>


using namespace dr;

int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

TEST(Yaml, vector) {
	ASSERT_EQ("{x: -0.100000, y: 1.200000, z: 2.300000}", toYaml(Eigen::Vector3d{-0.1, 1.2, 2.3}));
}

TEST(Yaml, quaternion) {
	ASSERT_EQ("{x: -0.100000, y: 1.200000, z: 2.300000, w: -1.900000}", toYaml(Eigen::Quaterniond{-1.9, -0.1, 1.2, 2.3}));
}

TEST(Yaml, pose) {
	std::string expected;
	expected += "position:    {x: -0.100000, y: 1.200000, z: 2.300000}\n";
	expected += "orientation: {x: 0.000000, y: 0.000000, z: 0.000000, w: 1.000000}";
	ASSERT_EQ(expected, toYaml(Eigen::Isometry3d{Eigen::Translation3d{-0.1, 1.2, 2.3} * Eigen::Quaterniond{1, 0, 0, 0}}));

	expected = "";
	expected += "\tposition:    {x: -0.100000, y: 1.200000, z: 2.300000}\n";
	expected += "\torientation: {x: 0.000000, y: 0.000000, z: 0.000000, w: 1.000000}";
	ASSERT_EQ(expected, toYaml(Eigen::Isometry3d{Eigen::Translation3d{-0.1, 1.2, 2.3} * Eigen::Quaterniond{1, 0, 0, 0}}, "\t"));
}

}
