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
TEST(EigenParamQuaternion, validDouble) {
	XmlRpc::XmlRpcValue quaterion;
	quaterion["w"] = double(1.0);
	quaterion["x"] = double(2.0);
	quaterion["y"] = double(3.0);
	quaterion["z"] = double(4.0);

	ASSERT_NEAR(0, fromXmlRpc<Eigen::Quaterniond>(quaterion).angularDistance(Eigen::Quaterniond{1, 2, 3, 4}), 1e-5);
}

// Parse a quaterion from ints.
TEST(EigenParamQuaternion, validInt) {
	XmlRpc::XmlRpcValue quaterion;
	quaterion["w"] = int(1);
	quaterion["x"] = int(2);
	quaterion["y"] = int(3);
	quaterion["z"] = int(4);

	ASSERT_NEAR(0, fromXmlRpc<Eigen::Quaterniond>(quaterion).angularDistance(Eigen::Quaterniond{1, 2, 3, 4}), 1e-5);
}

// Parse a quaterion from mixed doubles and ints.
TEST(EigenParamQuaternion, validMixed) {
	XmlRpc::XmlRpcValue quaterion;
	quaterion["w"] = double(1.0);
	quaterion["x"] = int(2);
	quaterion["y"] = double(3.0);
	quaterion["z"] = int(4);

	ASSERT_NEAR(0, fromXmlRpc<Eigen::Quaterniond>(quaterion).angularDistance(Eigen::Quaterniond{1, 2, 3, 4}), 1e-5);
}

// Test that parsing from a structure with too many components fails.
TEST(EigenParamQuaternion, tooManyComponents) {
	XmlRpc::XmlRpcValue quaterion;
	quaterion["v"] = 0.0;
	quaterion["w"] = 1.0;
	quaterion["x"] = 2.0;
	quaterion["y"] = 3.0;
	quaterion["z"] = 4.0;

	ASSERT_THROW(fromXmlRpc<Eigen::Quaterniond>(quaterion), std::exception);
}

// Test that parsing from a structure with too few components fails.
TEST(EigenParamQuaternion, missingComponents) {
	XmlRpc::XmlRpcValue quaterion;
	quaterion["x"] = 1.0;
	quaterion["y"] = 2.0;
	quaterion["z"] = 3.0;

	ASSERT_THROW(fromXmlRpc<Eigen::Quaterniond>(quaterion), std::exception);
}

// Test that parsing from a structure with wrongly named components fails.
TEST(EigenParamQuaternion, wrongComponents) {
	XmlRpc::XmlRpcValue quaterion;
	quaterion["aap"]  = 1.0;
	quaterion["noot"] = 2.0;
	quaterion["mies"] = 3.0;
	quaterion["wim"]  = 4.0;

	ASSERT_THROW(fromXmlRpc<Eigen::Quaterniond>(quaterion), std::exception);
}

}
