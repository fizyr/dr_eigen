#include "param.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>
#include <exception>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

// Parse a vector from doubles.
TEST(EigenParamVector, validDouble) {
	XmlRpc::XmlRpcValue vector;
	vector["x"] = double(1.0);
	vector["y"] = double(2.0);
	vector["z"] = double(3.0);

	ASSERT_TRUE(testEqual(Eigen::Vector3d{1, 2, 3}, fromXmlRpc<Eigen::Vector3d>(vector)));
}

// Parse a vector from ints.
TEST(EigenParamVector, validInt) {
	XmlRpc::XmlRpcValue vector;
	vector["x"] = int(1);
	vector["y"] = int(2);
	vector["z"] = int(3);

	ASSERT_TRUE(testEqual(Eigen::Vector3d{1, 2, 3}, fromXmlRpc<Eigen::Vector3d>(vector)));
}

// Parse a vector from mixed doubles and ints.
TEST(EigenParamVector, validMixed) {
	XmlRpc::XmlRpcValue vector;
	vector["x"] = double(1.0);
	vector["y"] = double(2.0);
	vector["z"] = int(3);

	ASSERT_TRUE(testEqual(Eigen::Vector3d{1, 2, 3}, fromXmlRpc<Eigen::Vector3d>(vector)));
}

// Test that parsing from a structure with too many components fails.
TEST(EigenParamVector, tooManyComponents) {
	XmlRpc::XmlRpcValue vector;
	vector["w"] = 0.0;
	vector["x"] = 1.0;
	vector["y"] = 2.0;
	vector["z"] = 3.0;

	ASSERT_THROW(fromXmlRpc<Eigen::Vector3d>(vector), std::exception);
}

// Test that parsing from a structure with too few components fails.
TEST(EigenParamVector, missingComponents) {
	XmlRpc::XmlRpcValue vector;
	vector["x"] = 1.0;
	vector["y"] = 2.0;

	ASSERT_THROW(fromXmlRpc<Eigen::Vector3d>(vector), std::exception);
}

// Test that parsing from a structure with wrongly named components fails.
TEST(EigenParamVector, wrongComponents) {
	XmlRpc::XmlRpcValue vector;
	vector["aap"]  = 1.0;
	vector["noot"] = 2.0;
	vector["mies"] = 3.0;

	ASSERT_THROW(fromXmlRpc<Eigen::Vector3d>(vector), std::exception);
}

}
