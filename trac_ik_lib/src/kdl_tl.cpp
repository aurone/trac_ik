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

#include <trac_ik/kdl_tl.hpp>

// standard includes
#include <limits>

// system includes
#include <ros/ros.h>

// project includes
#include <trac_ik/utils.h>

namespace KDL {

ChainIkSolverPos_TL::ChainIkSolverPos_TL(
    const Chain& chain,
    const JntArray& joint_min,
    const JntArray& joint_max,
    double eps,
    bool random_restart,
    bool try_jl_wrap)
:
    chain_(chain),
    joint_min_(joint_min),
    joint_max_(joint_max),
    joint_types_(),
    vik_solver_(chain),
    fk_solver_(chain),
    bounds_(KDL::Twist::Zero()),
    eps_(eps),
    rr_(random_restart),
    wrap_(try_jl_wrap),
    q_buff1_(chain_.getNrOfJoints()),
    q_buff2_(chain_.getNrOfJoints()),
    q_curr_(&q_buff1_),
    q_next_(&q_buff2_),
    delta_q_(chain.getNrOfJoints())
{
    assert(chain_.getNrOfJoints() == joint_min.data.size());
    assert(chain_.getNrOfJoints() == joint_max.data.size());

    for (size_t i = 0; i < chain_.segments.size(); i++) {
        std::string type = chain_.segments[i].getJoint().getTypeName();
        if (type.find("Rot") != std::string::npos) {
            if (joint_max(joint_types_.size()) >= std::numeric_limits<float>::max() &&
                joint_min(joint_types_.size()) <= std::numeric_limits<float>::lowest())
            {
                joint_types_.push_back(KDL::BasicJointType::Continuous);
            } else {
                joint_types_.push_back(KDL::BasicJointType::RotJoint);
            }
        } else if (type.find("Trans") != std::string::npos) {
            joint_types_.push_back(KDL::BasicJointType::TransJoint);
        }
    }

    assert(joint_types_.size() == joint_max.data.size());
}

int ChainIkSolverPos_TL::CartToJnt(
    const KDL::JntArray& q_init,
    const KDL::Frame& p_in,
    KDL::JntArray& q_out,
    const KDL::Twist& bounds)
{
    bounds_ = bounds;
    restart(q_init, p_in);

    const int max_iterations = 100;
    int res = 1;
    int i = 0;
    while (i < max_iterations && res != 0) {
        res = step();
        if (res == 2) { // local minima => random restart
            randomize(*q_curr_);
        }
        ++i;
    }

    if (res == 0) {
        q_out = qout();
    }

    return -res; // caller except negative values to mean errors
}

void ChainIkSolverPos_TL::restart(
    const KDL::JntArray& q_init,
    const KDL::Frame& p_in)
{
    *q_curr_ = q_init;
    f_target_ = p_in;
}

void ChainIkSolverPos_TL::restart(const KDL::JntArray& q_init)
{
    *q_curr_ = q_init;
}

int ChainIkSolverPos_TL::step()
{
    // update tip frame
    fk_solver_.JntToCart(*q_curr_, f_curr_);

    KDL::Twist delta_twist = diffRelative(f_target_, f_curr_);

    if (std::abs(delta_twist.vel.x()) <= std::abs(bounds_.vel.x())) {
        delta_twist.vel.x(0);
    }

    if (std::abs(delta_twist.vel.y()) <= std::abs(bounds_.vel.y())) {
        delta_twist.vel.y(0);
    }

    if (std::abs(delta_twist.vel.z()) <= std::abs(bounds_.vel.z())) {
        delta_twist.vel.z(0);
    }

    if (std::abs(delta_twist.rot.x()) <= std::abs(bounds_.rot.x())) {
        delta_twist.rot.x(0);
    }

    if (std::abs(delta_twist.rot.y()) <= std::abs(bounds_.rot.y())) {
        delta_twist.rot.y(0);
    }

    if (std::abs(delta_twist.rot.z()) <= std::abs(bounds_.rot.z())) {
        delta_twist.rot.z(0);
    }

    if (Equal(delta_twist, Twist::Zero(), eps_)) {
        return 0;
    }

    delta_twist = diff(f_curr_, f_target_);

    vik_solver_.CartToJnt(*q_curr_, delta_twist, delta_q_);

    // apply delta to get the next configuration
    Add(*q_curr_, delta_q_, *q_next_);

    // handle minimum variable bound
    for (size_t j = 0; j < joint_min_.data.size(); j++) {
        if (joint_types_[j] == KDL::BasicJointType::Continuous) {
            continue;
        }
        if ((*q_next_)(j) < joint_min_(j)) {
            if (!wrap_ || joint_types_[j] == KDL::BasicJointType::TransJoint) {
                // KDL's default
                (*q_next_)(j) = joint_min_(j);
            } else {
                // Find actual wrapped angle between limit and joint
                double diffangle = fmod(joint_min_(j) - (*q_next_)(j), 2 * M_PI);
                // Subtract that angle from limit and go into the range by a
                // revolution
                double curr_angle = joint_min_(j) - diffangle + 2 * M_PI;
                if (curr_angle > joint_max_(j)) {
                    (*q_next_)(j) = joint_min_(j);
                } else {
                    (*q_next_)(j) = curr_angle;
                }
            }
        }
    }

    // handle maximum variable bound
    for (size_t j = 0; j < joint_max_.data.size(); j++) {
        if (joint_types_[j] == KDL::BasicJointType::Continuous) {
            continue;
        }

        if ((*q_next_)(j) > joint_max_(j)) {
            if (!wrap_ || joint_types_[j] == KDL::BasicJointType::TransJoint) {
                // KDL's default
                (*q_next_)(j) = joint_max_(j);
            } else {
                // Find actual wrapped angle between limit and joint
                double diffangle = fmod((*q_next_)(j) - joint_max_(j), 2 * M_PI);
                // Add that angle to limit and go into the range by a revolution
                double curr_angle = joint_max_(j) + diffangle - 2 * M_PI;
                if (curr_angle < joint_min_(j)) {
                    (*q_next_)(j) = joint_max_(j);
                } else {
                    (*q_next_)(j) = curr_angle;
                }
            }
        }
    }

    // store the actually-moved delta in q_curr_
    Subtract(*q_curr_, *q_next_, *q_curr_);

    if (q_curr_->data.isZero(boost::math::tools::epsilon<float>())) {
        if (rr_) {
            std::swap(q_curr_, q_next_);
            return 2;
        }

        // Below would be an optimization to the normal KDL, where when it
        // gets stuck, it returns immediately.  Don't use to compare KDL with
        // random restarts or TRAC-IK to plain KDL.

        // else {
        //   q_out=q_curr;
        //   return -3;
        // }
    }

    // update the current configuration
    std::swap(q_curr_, q_next_);

    return 1;
}

void ChainIkSolverPos_TL::randomize(KDL::JntArray& q)
{
    for (size_t j = 0; j < q.data.size(); ++j) {
        if (joint_types_[j] == KDL::BasicJointType::Continuous) {
            q(j) = TRAC_IK::fRand(q(j) - 2 * M_PI, q(j) + 2 * M_PI);
        } else {
            q(j) = TRAC_IK::fRand(joint_min_(j), joint_max_(j));
        }
    }
}

} // namespace KDL
