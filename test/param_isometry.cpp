#include "param.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>
#include <exception>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

// Parse a quaternion from doubles.
TEST(EigenParamIsometry, validDouble) {
	XmlRpc::XmlRpcValue value;
	value["position"]["x"]    = double(1.0);
	value["position"]["y"]    = double(2.0);
	value["position"]["z"]    = double(3.0);
	value["orientation"]["w"] = double(-0.5);
	value["orientation"]["x"] = double(0.5);
	value["orientation"]["y"] = double(0.5);
	value["orientation"]["z"] = double(0.5);

	Eigen::Isometry3d converted = fromXmlRpc<Eigen::Isometry3d>(value);
	Eigen::Quaterniond orientation = Eigen::Quaterniond{converted.rotation()};

	ASSERT_EQ(Eigen::Vector3d(1, 2, 3), converted.translation());
	ASSERT_DOUBLE_EQ(-0.5, orientation.w());
	ASSERT_DOUBLE_EQ(0.5, orientation.x());
	ASSERT_DOUBLE_EQ(0.5, orientation.y());
	ASSERT_DOUBLE_EQ(0.5, orientation.z());
}

// Parse a value from ints.
TEST(EigenParamIsometry, validInt) {
	XmlRpc::XmlRpcValue value;
	value["position"]["x"]    = int(1);
	value["position"]["y"]    = int(2);
	value["position"]["z"]    = int(3);
	value["orientation"]["w"] = int(1);
	value["orientation"]["x"] = int(0);
	value["orientation"]["y"] = int(0);
	value["orientation"]["z"] = int(0);

	Eigen::Isometry3d converted = fromXmlRpc<Eigen::Isometry3d>(value);
	Eigen::Quaterniond orientation = Eigen::Quaterniond{converted.rotation()};

	ASSERT_EQ(Eigen::Vector3d(1, 2, 3), converted.translation());
	ASSERT_DOUBLE_EQ(1, orientation.w());
	ASSERT_DOUBLE_EQ(0, orientation.x());
	ASSERT_DOUBLE_EQ(0, orientation.y());
	ASSERT_DOUBLE_EQ(0, orientation.z());
}

// Test that parsing from a structure with too many components fails.
TEST(EigenParamIsometry, tooManyComponents) {
	XmlRpc::XmlRpcValue value;
	value["nonsense"]["a"]    = double(1.0);
	value["position"]["x"]    = double(1.0);
	value["position"]["y"]    = double(2.0);
	value["position"]["z"]    = double(3.0);
	value["orientation"]["w"] = double(-0.5);
	value["orientation"]["x"] = double(0.5);
	value["orientation"]["y"] = double(0.5);
	value["orientation"]["z"] = double(0.5);

	ASSERT_THROW(fromXmlRpc<Eigen::Isometry3d>(value), std::exception);
}

// Test that parsing from a structure with too few components fails.
TEST(EigenParamIsometry, missingComponents) {
	XmlRpc::XmlRpcValue value;
	value["position"]["x"]    = double(1.0);
	value["position"]["y"]    = double(2.0);
	value["position"]["z"]    = double(3.0);

	ASSERT_THROW(fromXmlRpc<Eigen::Isometry3d>(value), std::exception);
}

// Test that parsing from a structure with wrongly named components fails.
TEST(EigenParamIsometry, wrongComponents) {
	XmlRpc::XmlRpcValue value;
	value["nonsense"]["x"]      = double(1.0);
	value["nonsense"]["y"]      = double(2.0);
	value["nonsense"]["z"]      = double(3.0);
	value["more_nonsense"]["w"] = double(-0.5);
	value["more_nonsense"]["x"] = double(0.5);
	value["more_nonsense"]["y"] = double(0.5);
	value["more_nonsense"]["z"] = double(0.5);

	ASSERT_THROW(fromXmlRpc<Eigen::Isometry3d>(value), std::exception);
}

}
