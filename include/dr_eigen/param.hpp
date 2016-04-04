#include "eigen.hpp"
#include <dr_param/xmlrpc.hpp>

namespace dr {
	/// Convert an XmlRpcValue to an Eigen::Vector3d.
	template<> Eigen::Vector3d fromXmlRpc<Eigen::Vector3d>(XmlRpc::XmlRpcValue const & value);

	/// Convert an XmlRpcValue to an Eigen::Quaterniond.
	template<> Eigen::Quaterniond fromXmlRpc<Eigen::Quaterniond>(XmlRpc::XmlRpcValue const & value);

	/// Convert an XmlRpcValue to an Eigen::Isometry3d.
	template<> Eigen::Isometry3d fromXmlRpc<Eigen::Isometry3d>(XmlRpc::XmlRpcValue const & value);

	/// Convert an XmlRpcValue to an Eigen::AlignedBox3d.
	template<> Eigen::AlignedBox3d fromXmlRpc<Eigen::AlignedBox3d>(XmlRpc::XmlRpcValue const & value);

	/// Convert an XmlRpcValue to a PoseHeader.
	template<> PoseHeader fromXmlRpc<PoseHeader>(XmlRpc::XmlRpcValue const & value);

	/// Convert an XmlRpcValue to a Pose.
	template<> Pose fromXmlRpc<Pose>(XmlRpc::XmlRpcValue const & value);
}
