#include "param.hpp"
#include "eigen.hpp"
#include "test/compare.hpp"

#include <gtest/gtest.h>
#include <exception>


int main(int argc, char * * argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

namespace dr {

// Parse a quaternion from doubles.
TEST(EigenParameterAlignedBox, validDouble) {
	XmlRpc::XmlRpcValue value;
	value["center"]["x"]     = double(1.0);
	value["center"]["y"]     = double(2.0);
	value["center"]["z"]     = double(3.0);
	value["dimensions"]["x"] = double(4.0);
	value["dimensions"]["y"] = double(5.0);
	value["dimensions"]["z"] = double(6.0);

	ASSERT_TRUE(fromXmlRpc<Eigen::AlignedBox3d>(value).isApprox(makeCenteredBox({1, 2, 3}, {4, 5, 6})));
}

// Parse a value from ints.
TEST(EigenParameterAlignedBox, validInt) {
	XmlRpc::XmlRpcValue value;
	value["center"]["x"]     = int(1);
	value["center"]["y"]     = int(2);
	value["center"]["z"]     = int(3);
	value["dimensions"]["x"] = int(4);
	value["dimensions"]["y"] = int(5);
	value["dimensions"]["z"] = int(6);

	ASSERT_TRUE(fromXmlRpc<Eigen::AlignedBox3d>(value).isApprox(makeCenteredBox({1, 2, 3}, {4, 5, 6})));
}

// Test that parsing from a structure with too many components fails.
TEST(EigenParameterAlignedBox, tooManyComponents) {
	XmlRpc::XmlRpcValue value;
	value["center"]["x"]     = double(1.0);
	value["center"]["y"]     = double(2.0);
	value["center"]["z"]     = double(3.0);
	value["dimensions"]["x"] = double(4.0);
	value["dimensions"]["y"] = double(5.0);
	value["dimensions"]["z"] = double(6.0);
	value["nonsense"]["x"]   = double(7.0);

	ASSERT_THROW(fromXmlRpc<Eigen::AlignedBox3d>(value), std::exception);
}

// Test that parsing from a structure with too few components fails.
TEST(EigenParameterAlignedBox, missingComponents) {
	XmlRpc::XmlRpcValue value;
	value["center"]["x"]     = double(1.0);
	value["center"]["y"]     = double(2.0);
	value["center"]["z"]     = double(3.0);

	ASSERT_THROW(fromXmlRpc<Eigen::AlignedBox3d>(value), std::exception);
}

// Test that parsing from a structure with wrongly named components fails.
TEST(EigenParameterAlignedBox, wrongComponents) {
	XmlRpc::XmlRpcValue value;
	value["nonsense"]["x"]     = double(1.0);
	value["nonsense"]["y"]     = double(2.0);
	value["nonsense"]["z"]     = double(3.0);
	value["garbage"]["x"]      = double(4.0);
	value["garbage"]["y"]      = double(5.0);
	value["garbage"]["z"]      = double(6.0);

	ASSERT_THROW(fromXmlRpc<Eigen::AlignedBox3d>(value), std::exception);
}

}
