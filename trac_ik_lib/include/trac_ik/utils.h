#ifndef TRAC_IK_UTILS_H
#define TRAC_IK_UTILS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>

namespace TRAC_IK {

bool LoadModelOverride(
    const ros::NodeHandle& nh,
    const std::string& robot_description,
    urdf::Model& model);

bool InitKDLChain(
    const urdf::Model& model,
    const std::string& base_name,
    const std::string& tip_name,
    KDL::Chain& chain,
    std::vector<std::string>& link_names,
    std::vector<std::string>& joint_names,
    KDL::JntArray& joint_min,
    KDL::JntArray& joint_max);

} // namespace TRAC_IK

#endif
