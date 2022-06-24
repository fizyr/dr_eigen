// Copyright 2014-2022, Fizyr B.V.

#pragma once
#include <string>
#include <Eigen/Dense>

#include <gtest/gtest.h>

namespace dr {

/// Convert an eigen vector to a terser string.
std::string toString(Eigen::Vector3d const & v) {
	return "[" + std::to_string(v[0]) + ", " + std::to_string(v[1]) + ", " + std::to_string(v[2]) + "]";
}


std::string toString(Eigen::AngleAxisd const & a) {
	Eigen::Vector3d axis = a.axis();
	return "[" + std::to_string(axis[0]) + ", " + std::to_string(axis[1]) + ", " +
		std::to_string(axis[2]) + ", " + std::to_string(a.angle()) + "]";
}

std::string toString(Eigen::Isometry3d const & a) {
	return toString(a.translation()) + toString(Eigen::AngleAxisd(a.rotation()));
}

/// Test if two Eigen vectors are exactly equal.
testing::AssertionResult testEqual(Eigen::Vector3d const & expected, Eigen::Vector3d const & actual) {
	if (expected.x() == actual.x() && expected.y() == actual.y() && expected.z() == actual.z()) {
		return testing::AssertionSuccess();
	} else {
		return testing::AssertionFailure() << "actual (" << toString(actual) << ") does not equal expected (" << toString(expected) << ")";
	}
}

/// Test if two Eigen vectors are within tolerance of eachother.
testing::AssertionResult testNear(Eigen::Vector3d const & expected, Eigen::Vector3d const & actual, Eigen::Vector3d const & tolerance = {0.001, 0.001, 0.001}) {
	auto diff = (expected - actual).cwiseAbs();
	if ((diff.array() <= tolerance.array()).all()) {
		return testing::AssertionSuccess();
	} else {
		return testing::AssertionFailure() << "actual (" << toString(actual) << ") is not within tolerance (" << toString(tolerance) << ") of expected (" << toString(expected) << ")";
	}
}

/// Test if two Eigen rotation axes are within tolerance of eachother.
testing::AssertionResult testNear(Eigen::AngleAxisd const & expected, Eigen::AngleAxisd const & actual, float tolerance = 0.001) {
	if (actual.isApprox(expected, tolerance)) {
		return testing::AssertionSuccess();
	} else {
		return testing::AssertionFailure() << "actual (" << toString(actual) << ") is not within tolerance (" << std::to_string(tolerance) << ") of expected (" << toString(expected) << ")";
	}
}

/// Test if two Eigen isometries are within tolerance of each other.
testing::AssertionResult testNear(Eigen::Isometry3d const & expected, Eigen::Isometry3d const & actual, float tolerance = 0.001) {
	if (
		testNear(Eigen::Vector3d(expected.translation()), Eigen::Vector3d(actual.translation()), Eigen::Vector3d(tolerance, tolerance, tolerance)) &&
		testNear(Eigen::AngleAxisd(Eigen::Quaterniond(expected.rotation())), Eigen::AngleAxisd(Eigen::Quaterniond(actual.rotation())), tolerance)
	) {
		return testing::AssertionSuccess();
	} else {
		return testing::AssertionFailure() << "actual (" << toString(actual) << ") is not within tolerance (" << std::to_string(tolerance) << ") of expected (" << toString(expected) << ")";
	}
}

}
