#include "param.hpp"
#include "eigen.hpp"

namespace dr {

template<> Eigen::Vector3d fromXmlRpc<Eigen::Vector3d>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Eigen::Vector3d");
	if (value.size() != 3) throw std::runtime_error("wrong number of components for Eigen::Vector3d: " + std::to_string(value.size()) + " (expected 3)");

	return {
		fromXmlRpc<double>(xmlRpcAt(value, "x")),
		fromXmlRpc<double>(xmlRpcAt(value, "y")),
		fromXmlRpc<double>(xmlRpcAt(value, "z")),
	};
}

template<> Eigen::Quaterniond fromXmlRpc<Eigen::Quaterniond>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Eigen::Quaterniond");
	if (value.size() != 4) throw std::runtime_error("wrong number of components for Eigen::Quaterniond: " + std::to_string(value.size()) + " (expected 4)");

	return {
		fromXmlRpc<double>(xmlRpcAt(value, "w")),
		fromXmlRpc<double>(xmlRpcAt(value, "x")),
		fromXmlRpc<double>(xmlRpcAt(value, "y")),
		fromXmlRpc<double>(xmlRpcAt(value, "z")),
	};
}

template<> Eigen::Isometry3d fromXmlRpc<Eigen::Isometry3d>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Eigen::Isometry3d");
	if (value.size() != 2) throw std::runtime_error("wrong number of components for Eigen::Isometry3d: " + std::to_string(value.size()) + " (expected 2)");

	return Eigen::Translation3d(fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value, "position"))) * fromXmlRpc<Eigen::Quaterniond>(xmlRpcAt(value, "orientation"));
}

template<> Eigen::AlignedBox3d fromXmlRpc<Eigen::AlignedBox3d>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Eigen::AlignedBox3d");
	if (value.size() != 2) throw std::runtime_error("wrong number of components for Eigen::AlignedBox3d: " + std::to_string(value.size()) + " (expected 2)");

	if (value.hasMember("center") && value.hasMember("dimensions")) {
		Eigen::Vector3d center     = fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value, "center"));
		Eigen::Vector3d dimensions = fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value, "dimensions"));
		return makeCenteredBox(center, dimensions);
	} else if (value.hasMember("min") && value.hasMember("max")) {
		Eigen::Vector3d min = fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value, "min"));
		Eigen::Vector3d max = fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value, "max"));
		return Eigen::AlignedBox3d{min, max};
	}

	throw std::runtime_error("wrong members for Eigen::AlignedBox3d: need either center and dimensions or min and max");
}

template<> PoseHeader fromXmlRpc<PoseHeader>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "PoseHeader");
	if (value.size() != 2) throw std::runtime_error("wrong number of components for PoseHeader: " + std::to_string(value.size()) + " (expected 2)");

	return PoseHeader{fromXmlRpc<std::string>(xmlRpcAt(value, "parent_frame")), fromXmlRpc<std::string>(xmlRpcAt(value, "child_frame"))};
}

template<> Pose fromXmlRpc<Pose>(XmlRpc::XmlRpcValue const & value) {
	ensureXmlRpcType(value, XmlRpc::XmlRpcValue::TypeStruct, "Pose");
	if (value.size() != 4) throw std::runtime_error("wrong number of components for Pose: " + std::to_string(value.size()) + " (expected 4)");

	PoseHeader header = PoseHeader{fromXmlRpc<std::string>(xmlRpcAt(value, "parent_frame")), fromXmlRpc<std::string>(xmlRpcAt(value, "child_frame"))};
	Eigen::Isometry3d isometry = Eigen::Translation3d(fromXmlRpc<Eigen::Vector3d>(xmlRpcAt(value, "position"))) * fromXmlRpc<Eigen::Quaterniond>(xmlRpcAt(value, "orientation"));
	return Pose{header, isometry};
}

}
