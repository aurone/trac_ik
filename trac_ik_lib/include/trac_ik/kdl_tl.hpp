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

#ifndef KDLCHAINIKSOLVERPOS_TL_HPP
#define KDLCHAINIKSOLVERPOS_TL_HPP

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

namespace KDL {

enum BasicJointType {
    RotJoint, TransJoint, Continuous
};

/// An inverse kinematics algorithm that computes an inverse kinematics solution
/// via repeated application.
class ChainIkSolverPos_TL
{
public:

    ChainIkSolverPos_TL(
        const Chain& chain,
        const JntArray& q_min,
        const JntArray& q_max,
        double eps = 1e-3,
        bool random_restart = false,
        bool try_jl_wrap = false);

    /// \name Configuration
    ///@{
    void setBounds(const KDL::Twist& bounds) { bounds_ = bounds; }
    auto bounds() const -> const KDL::Twist& { return bounds_; }

    void setEps(double eps) { eps_ = eps; }
    double eps() const { return eps_; }
    ///@}

    /// \name Iterative Cart-to-Joint Interface
    ///@{

    /// Reset the current position and target frame of the solver.
    void restart(const KDL::JntArray& q_init, const KDL::Frame& p_in);

    /// Reset the current position, but NOT the target frame, of the solver.
    void restart(const KDL::JntArray& q_init);

    /// Step through a single iteration of the solver.
    ///
    /// Returns 0 if the solver has already converged to a solution and a
    /// non-zero value otherwise.
    int step(int steps = 1);

    ///@}

    /// Return the current configuration in the solver; the solution
    /// configuration if the solver has converged to a solution.
    const KDL::JntArray& qout() const { return *q_curr_; }

    int CartToJnt(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& bounds = KDL::Twist::Zero());

private:

    const Chain chain_;
    JntArray joint_min_;
    JntArray joint_max_;
    std::vector<KDL::BasicJointType> joint_types_;

    KDL::ChainIkSolverVel_pinv vik_solver_;
    KDL::ChainFkSolverPos_recursive fk_solver_;

    // step configuration
    KDL::Twist bounds_;
    double eps_;
    bool rr_;
    bool wrap_;

    // double-buffering for storing curr and next positions
    KDL::JntArray q_buff1_;
    KDL::JntArray q_buff2_;

    // current joint positions
    KDL::JntArray *q_curr_;

    // next joint positions
    KDL::JntArray *q_next_;

    bool done_;

    KDL::Frame f_curr_;
    KDL::JntArray delta_q_;

    KDL::Frame f_target_;

    void randomize(KDL::JntArray& q);
};

/**
 * determines the rotation axis necessary to rotate from frame b1 to the
 * orientation of frame b2 and the vector necessary to translate the origin
 * of b1 to the origin of b2, and stores the result in a Twist
 * datastructure.  The result is w.r.t. frame b1.
 * \param F_a_b1 frame b1 expressed with respect to some frame a.
 * \param F_a_b2 frame b2 expressed with respect to some frame a.
 * \warning The result is not a real Twist!
 * \warning In contrast to standard KDL diff methods, the result of
 * diffRelative is w.r.t. frame b1 instead of frame a.
 */
IMETHOD Twist diffRelative(
    const Frame & F_a_b1,
    const Frame & F_a_b2,
    double dt = 1)
{
    return Twist(
            F_a_b1.M.Inverse() * diff(F_a_b1.p, F_a_b2.p, dt),
            F_a_b1.M.Inverse() * diff(F_a_b1.M, F_a_b2.M, dt));
}

} // namespace KDL

#endif
