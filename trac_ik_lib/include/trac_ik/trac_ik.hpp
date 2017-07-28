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

#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

// standard includes
#include <random>

// system includes
#include <kdl/chainjnttojacsolver.hpp>

// project includes
#include <trac_ik/nlopt_ik.hpp>

namespace TRAC_IK {

enum SolveType
{
    Speed,
    Distance,
    Manip1,
    Manip2
};

class TRAC_IK
{
public:

    TRAC_IK(
        const KDL::Chain& _chain,
        const KDL::JntArray& _q_min,
        const KDL::JntArray& _q_max,
        int max_iters = 100,
        double _eps = 1e-5,
        SolveType _type = Speed);

    void setBounds(const KDL::Twist& bounds);
    const KDL::Twist& getBounds() const { return bounds_; }

    void setMaxIterations(int max_iters) { max_iters_ = max_iters; }

    const KDL::Chain& getKDLChain(KDL::Chain& chain) const { return chain_; }

    const KDL::JntArray& getLowerLimits() const { return joint_min_; }
    const KDL::JntArray& getUpperLimits() const { return joint_max_; }

    /// Return a negative value if an error was encountered.
    int CartToJnt(
        const KDL::JntArray &q_init,
        const KDL::Frame &p_in,
        KDL::JntArray &q_out,
        const KDL::Twist& bounds = KDL::Twist::Zero());

    void SetSolveType(SolveType _type) { solve_type_ = _type; }

private:

    KDL::Chain chain_;
    KDL::JntArray joint_min_;
    KDL::JntArray joint_max_;
    std::vector<KDL::BasicJointType> joint_types_;

    std::default_random_engine rng_;

    KDL::ChainJntToJacSolver jac_solver_;

    NLOPT_IK::NLOPT_IK nl_solver_;
    KDL::ChainIkSolverPos_TL ik_solver_;

    KDL::Twist bounds_;
    double eps_;
    SolveType solve_type_;
    int max_iters_;

    std::vector<KDL::JntArray> solutions_;
    std::vector<std::pair<double, size_t>> errors_;

    KDL::JntArray seed_;

    void randomize(KDL::JntArray& q, const KDL::JntArray& q_init);
    void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
    void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution);

    bool unique_solution(const KDL::JntArray& sol);

    /* @brief Manipulation metrics and penalties taken from "Workspace
     Geometric Characterization and Manipulability of Industrial Robots",
     Ming-June, Tsia, PhD Thesis, Ohio State University, 1986.
     https://etd.ohiolink.edu/!etd.send_file?accession=osu1260297835
     */
    double manipPenalty(const KDL::JntArray&);
    double ManipValue1(const KDL::JntArray&);
    double ManipValue2(const KDL::JntArray&);
};

} // namespace TRAC_IK

#endif
