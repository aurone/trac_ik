/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include "trac_ik_kinematics_plugin.h"

#include <algorithm>
#include <limits>

#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>
#include <trac_ik/trac_ik.hpp>
#include <trac_ik/utils.h>
#include <urdf/model.h>

namespace trac_ik_kinematics_plugin {

bool TRAC_IKKinematicsPlugin::initialize(
    const std::string& robot_description,
    const std::string& group_name,
    const std::string& base_name,
    const std::string& tip_name,
    double search_discretization)
{
    setValues(robot_description, group_name, base_name, tip_name, search_discretization);

    ros::NodeHandle node_handle("~");
    urdf::Model robot_model;

    if (!TRAC_IK::LoadModelOverride(node_handle, robot_description, robot_model)) {
        ROS_WARN_STREAM_NAMED("trac_ik", "Failed to load robot model");
        return false;
    }

    ROS_DEBUG_STREAM_NAMED("trac_ik","Reading joints and links from URDF");

    if (!TRAC_IK::InitKDLChain(
        robot_model, base_name, tip_name,
        chain_, link_names_, joint_names_, joint_min_, joint_max_))
    {
        ROS_WARN_STREAM_NAMED("trac_ik", "Failed to initialize KDL chain");
        return false;
    }

    tmp_in_.resize(chain_.getNrOfJoints());
    tmp_out_.resize(chain_.getNrOfJoints());

    ROS_DEBUG_NAMED("trac-ik plugin", "Looking in private handle: %s for param name: %s", node_handle.getNamespace().c_str(), (group_name + "/position_only_ik").c_str());

    node_handle.param(group_name + "/position_only_ik", position_ik_, false);

    bounds_ = KDL::Twist::Zero();

    if (position_ik_) {
        bounds_.rot.x(std::numeric_limits<float>::max());
        bounds_.rot.y(std::numeric_limits<float>::max());
        bounds_.rot.z(std::numeric_limits<float>::max());
    }

    ROS_DEBUG_NAMED("trac-ik plugin", "Looking in private handle: %s for param name: %s", node_handle.getNamespace().c_str(), (group_name + "/solve_type").c_str());

    std::string solve_type;
    node_handle.param(group_name + "/solve_type", solve_type, std::string("Speed"));
    ROS_DEBUG_NAMED("trac_ik plugin", "Using solve type %s", solve_type.c_str());

    if (solve_type == "Manipulation1") {
        solve_type_ = TRAC_IK::Manip1;
    }
    else if (solve_type == "Manipulation2") {
        solve_type_ = TRAC_IK::Manip2;
    }
    else if (solve_type == "Distance") {
        solve_type_ = TRAC_IK::Distance;
    }
    else {
        if (solve_type != "Speed") {
            ROS_WARN_STREAM_NAMED("trac_ik", solve_type << " is not a valid solve_type; setting to default: Speed");
        }
        solve_type_ = TRAC_IK::Speed;
    }

    // Same as MoveIt's KDL plugin
    const double epsilon = 1e-5;

    solver_.reset(new TRAC_IK::TRAC_IK(
            chain_, joint_min_, joint_max_, 1000, epsilon, solve_type_));

    active_ = true;
    return true;
}

int TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
    int i = 0;
    while (i < (int) chain_.getNrOfSegments()) {
        if (chain_.getSegment(i).getName() == name) {
            return i + 1;
        }
        i++;
    }
    return -1;
}

bool TRAC_IKKinematicsPlugin::getPositionFK(
    const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::Pose> &poses) const
{
    assert(active_);

    poses.resize(link_names.size());
    if (joint_angles.size() != chain_.getNrOfJoints()) {
        ROS_ERROR_NAMED("trac_ik", "Joint angles vector must have size: %d", chain_.getNrOfJoints());
        return false;
    }

    KDL::Frame p_out;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    auto& jnt_pos_in(tmp_in_);
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
        jnt_pos_in(i) = joint_angles[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(chain_);

    bool valid = true;
    for (unsigned int i = 0; i < poses.size(); i++) {
        ROS_DEBUG_NAMED("trac_ik","End effector index: %d", getKDLSegmentIndex(link_names[i]));
        if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0) {
            tf::poseKDLToMsg(p_out, poses[i]);
        } else {
            ROS_ERROR_NAMED("trac_ik", "Could not compute FK for %s", link_names[i].c_str());
            valid = false;
        }
    }

    return valid;
}

bool TRAC_IKKinematicsPlugin::getPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(
            ik_pose,
            ik_seed_state,
            default_timeout_,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(
            ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    const IKCallbackFn solution_callback = 0;
    return searchPositionIK(
            ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    std::vector<double> consistency_limits;
    return searchPositionIK(
            ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
    return searchPositionIK(
            ik_pose,
            ik_seed_state,
            timeout,
            solution,
            solution_callback,
            error_code,
            consistency_limits,
            options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const std::vector<double> &consistency_limits,
    const kinematics::KinematicsQueryOptions &options) const
{
    ROS_DEBUG_STREAM_NAMED("trac_ik","getPositionIK");

    assert(active_);

    if (ik_seed_state.size() != chain_.getNrOfJoints()) {
        ROS_ERROR_STREAM_NAMED("trac_ik", "Seed state must have size " << chain_.getNrOfJoints() << " instead of size " << ik_seed_state.size());
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    KDL::Frame frame;
    tf::poseMsgToKDL(ik_pose, frame);

    auto& in(tmp_in_);
    auto& out(tmp_out_);

    for (unsigned int z = 0; z < chain_.getNrOfJoints(); ++z) {
        in(z) = ik_seed_state[z];
    }

    solver_->setMaxIterations(timeout * iter_per_time_);

    int rc = solver_->CartToJnt(in, frame, out, bounds_);

    if (rc < 0) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    solution.resize(chain_.getNrOfJoints());

    for (unsigned int z = 0; z < chain_.getNrOfJoints(); z++) {
        solution[z] = out(z);
    }

    // check for collisions if a callback is provided
    if (solution_callback.empty()) {
        return true;
    }

    solution_callback(ik_pose, solution, error_code);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_DEBUG_STREAM_NAMED("trac_ik","Solution passes callback");
        return true;
    } else {
        ROS_DEBUG_STREAM_NAMED("trac_ik","Solution has error code " << error_code);
        return false;
    }
}

} // namespace trac_ik_kinematics_plugin

//register TRAC_IKKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin, kinematics::KinematicsBase);
