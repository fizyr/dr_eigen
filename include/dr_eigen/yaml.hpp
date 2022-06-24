// Copyright 2014-2022, Fizyr B.V.

#pragma once
#include "eigen.hpp"

#include <string>

namespace dr {

/// Convert a vector to YAML.
std::string toYaml(Eigen::Vector3d const & vector);

/// Convert a quaternion to YAML.
std::string toYaml(Eigen::Quaterniond const & quaternion);

/// Convert an isometry to YAML.
std::string toYaml(Eigen::Isometry3d const & pose, std::string const & indent = "");

/// Convert a pose header to YAML.
std::string toYaml(PoseHeader const & header, std::string const & indent = "");

/// Convert a pose to YAML.
std::string toYaml(Pose const & pose, std::string const & indent = "");

}
