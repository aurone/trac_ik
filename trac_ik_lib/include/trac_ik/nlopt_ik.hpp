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

#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

// system includes
#include <nlopt.hpp>

#include <trac_ik/dual_quaternion.h>
#include <trac_ik/kdl_tl.hpp>

namespace NLOPT_IK {

enum OptType
{
    Joint,
    DualQuat,
    SumSq,
    L2
};

class NLOPT_IK
{
public:

    NLOPT_IK(
        const KDL::Chain& chain,
        const KDL::JntArray& q_min,
        const KDL::JntArray& q_max,
        double eps = 1e-3,
        OptType type = SumSq);

    /// \name Configuration
    ///@{
    void setBounds(const KDL::Twist& bounds) { bounds_ = bounds; }
    auto bounds() -> const KDL::Twist& { return bounds_; }

    void setEps(double eps) { eps_ = eps; }
    double eps() const { return eps_; }
    ///@}

    /// \name Iterative Cart-to-Joint Interface
    ///@{

    ///  Reset the current position and target frame of the solver.
    void restart(const KDL::JntArray& q_init, const KDL::Frame& p_in);

    /// Reset the current position, but NOT the target frame, of the solver.
    void restart(const KDL::JntArray& q_init);

    /// Step through a single iteration of the solver.
    ///
    /// Returns 0 if the solver finds a solution or has already converted to a
    /// solution and a non-zero value otherwise.
    int step();

    /// Return the current configuration in the solver; the solution
    /// configuration if the solver has converged to a solution.
    const KDL::JntArray& qout() const;
    ///@}

    int CartToJnt(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist bounds = KDL::Twist::Zero(),
        const KDL::JntArray& q_desired = KDL::JntArray());

    /// \name Minimization Objectives -- These functions flag the solver to
    /// to stop by setting the internal progress state and also record the value
    /// of the solver at the point where minimization succeeded

    // minimization objective for OptType::Joint.
    double minJoints(const std::vector<double>& x, std::vector<double>& grad);

    // minimization objective for OptType::SumSq and equality constraint of
    // OptType::Joint.
    void cartSumSquaredError(const std::vector<double>& x, double error[]);

    // minimization objective for OptType::DualQuat.
    void cartDQError(const std::vector<double>& x, double error[]);

    // minimization objective for OptType::L2.
    void cartL2NormError(const std::vector<double>& x, double error[]);

private:

    const KDL::Chain chain_;
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;
    std::vector<KDL::BasicJointType> types_;

    bool valid_; // whether this solver is valid for the given chain

    KDL::ChainFkSolverPos_recursive fk_solver_;

    // Problem Configuration
    KDL::Twist bounds_;
    double eps_;

    dual_quaternion target_dq_;         // the target duql quaternion for cost computation
    std::vector<double> target_pos_;    // the target pos for cost computation
    KDL::Frame f_target_;               // the target pose
    std::vector<double> x_min_;         // artificial max position limits for the solver
    std::vector<double> x_max_;         // artificial min position limits for the solver

    std::vector<double> best_x_;
    KDL::JntArray q_out_;

    // -3 for in-progress, reset in restart()
    // 1 for found solution
    // -1 for nans computed in one of the solver functions?
    int progress_;

    OptType opt_type_;

    nlopt::opt nlopt_;
};

} // namespace NLOPT_IK

#endif
