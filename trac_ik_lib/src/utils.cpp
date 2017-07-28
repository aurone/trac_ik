#include <trac_ik/utils.h>

#include <assert.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace TRAC_IK {

/// Loads a URDF model from ROS parameter. Additionally, the model may be
/// overridden by a custom model specified by the 'urdf_xml' ROS parameter in
/// the namespace defined by $nh. $nh/urdf_xml serves as a replacement for
/// $robot_description, if available.
///
/// \param nh Node handle to define a namespace to search from for the robot
///     description
/// \param robot_description The name of the robot model description parameter
/// \param[out] model
bool LoadModelOverride(
    const ros::NodeHandle& nh,
    const std::string& robot_description,
    urdf::Model& model)
{
    std::string xml_string;

    std::string urdf_xml;
    nh.param("urdf_xml", urdf_xml, robot_description);

    std::string full_urdf_xml;
    nh.searchParam(urdf_xml, full_urdf_xml);

    ROS_DEBUG_NAMED("trac_ik","Reading xml file from parameter server");
    if (!nh.getParam(full_urdf_xml, xml_string)) {
        ROS_FATAL_NAMED("trac_ik", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
    }

    nh.param(full_urdf_xml, xml_string, std::string());
    model.initString(xml_string);
    return true;
}

bool InitKDLChain(
    const urdf::Model& model,
    const std::string& base_name,
    const std::string& tip_name,
    KDL::Chain& chain,
    std::vector<std::string>& link_names,
    std::vector<std::string>& joint_names,
    KDL::JntArray& joint_min,
    KDL::JntArray& joint_max)
{
    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR_NAMED("trac_ik", "Failed to extract kdl tree from xml robot description");
        return false;
    }

    if (!tree.getChain(base_name, tip_name, chain)) {
        ROS_ERROR_NAMED("trac_ik", "Couldn't find chain %s to %s", base_name.c_str(), tip_name.c_str());
        return false;
    }

    joint_min.resize(chain.getNrOfJoints());
    joint_max.resize(chain.getNrOfJoints());

    unsigned int joint_num = 0;
    for (auto& segment : chain.segments) {
        link_names.push_back(segment.getName());
        auto joint = model.getJoint(segment.getJoint().getName());
        if (joint->type != urdf::Joint::UNKNOWN &&
            joint->type != urdf::Joint::FIXED)
        {
            joint_num++;
            assert(joint_num <= chain.getNrOfJoints());
            float lower, upper;
            int hasLimits;
            joint_names.push_back(joint->name);
            if (joint->type != urdf::Joint::CONTINUOUS) {
                if (joint->safety) {
                    lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
                    upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
                } else {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                }
                hasLimits = 1;
            } else {
                hasLimits = 0;
            }
            if (hasLimits) {
                joint_min(joint_num - 1) = lower;
                joint_max(joint_num - 1) = upper;
            } else {
                joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
                joint_max(joint_num - 1) = std::numeric_limits<float>::max();
            }
            ROS_INFO_STREAM("IK Using joint " << segment.getName() << " " << joint_min(joint_num - 1) << " " << joint_max(joint_num - 1));
        }
    }

    return true;
}

} // namespace TRAC_IK

namespace KDL {

std::ostream& operator<<(std::ostream& o, const JntArray& q)
{
    o << '[' << ' ';
    for (unsigned int i = 0; i < q.rows(); ++i) {
        o << q(i) << ' ';
    }
    o << ']';
    return o;
}

std::ostream& operator<<(std::ostream& o, const Vector& v)
{
    o << '(' << v.x() << ", " << v.y() << ", " << v.z() << ')';
    return o;
}

std::ostream& operator<<(std::ostream& o, const Rotation& R)
{
    double a, b, c;
    R.GetEulerZYX(a, b, c);
    o << "(" << a << ", " << b << ", " << c << ")";
    return o;
}

std::ostream& operator<<(std::ostream& o, const Frame& f)
{
    o << "{ p: " << f.p << ", M: " << f.M << " }";
    return o;
}

} // namespace KDL
