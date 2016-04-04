#include "yaml.hpp"

namespace dr {

/// Convert a vector to YAML.
std::string toYaml(const Eigen::Vector3d & vector) {
	std::string result;
	result.reserve(50);

	result += "{x: ";
	result += std::to_string(vector.x());

	result += ", y: ";
	result += std::to_string(vector.y());

	result += ", z: ";
	result += std::to_string(vector.z());

	result += "}";
	return result;
}

/// Convert a quaternion to YAML.
std::string toYaml(const Eigen::Quaterniond & quaternion) {
	std::string result;
	result.reserve(50);

	result += "{x: ";
	result += std::to_string(quaternion.x());

	result += ", y: ";
	result += std::to_string(quaternion.y());

	result += ", z: ";
	result += std::to_string(quaternion.z());

	result += ", w: ";
	result += std::to_string(quaternion.w());
	result += "}";
	return result;
}

/// Convert an isometry to YAML.
std::string toYaml(const Eigen::Isometry3d & pose, std::string const & indent) {
	std::string result;
	result.reserve(100);

	result.append(indent);
	result.append("position:    ");
	result.append(toYaml(pose.translation()));
	result.push_back('\n');
	result.append(indent);
	result.append("orientation: ");
	result.append(toYaml(Eigen::Quaterniond(pose.rotation())));
	return result;
}

std::string toYaml(PoseHeader const & header, std::string const & indent) {
	std::string result;
	result.reserve(100);

	result.append(indent);
	result.append("  ");
	result.append("parent_frame: ");
	result.append(header.parent_frame);
	result.push_back('\n');
	result.append(indent);
	result.append("  ");
	result.append("child_frame:  ");
	result.append(header.child_frame);
	return result;
}

std::string toYaml(Pose const & pose, std::string const & indent) {
	std::string result;
	result.reserve(100);

	result.append(toYaml(pose.header, indent));
	result.push_back('\n');
	result.append(toYaml(pose.isometry));
	return result;
}

}
