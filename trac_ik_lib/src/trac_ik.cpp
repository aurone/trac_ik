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

#include <trac_ik/trac_ik.hpp>

// standard includes
#include <chrono>
#include <limits>

// system includes
#include <boost/date_time.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Geometry>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

// project includes
#include <trac_ik/utils.h>

namespace TRAC_IK {

inline double JointErr(
    const KDL::JntArray& q1,
    const KDL::JntArray& q2)
{
    double err = 0;
    for (uint i = 0; i < q1.data.size(); i++) {
        err += pow(q1(i) - q2(i), 2);
    }

    return err;
}

TRAC_IK::TRAC_IK(
    const KDL::Chain& chain,
    const KDL::JntArray& q_min,
    const KDL::JntArray& q_max,
    int max_iterations,
    double eps,
    SolveType type)
:
    chain_(chain),
    joint_min_(q_min),
    joint_max_(q_max),
    joint_types_(),
    jac_solver_(chain),
    nl_solver_(chain, q_min, q_max, eps, NLOPT_IK::SumSq),
    ik_solver_(chain, q_min, q_max, eps, true, true),
    bounds_(KDL::Twist::Zero()),
    eps_(eps),
    solve_type_(type),
    max_iters_(max_iterations),
    solutions_(),
    errors_(),
    seed_(chain.getNrOfJoints())
{
    assert(chain_.getNrOfJoints() == joint_min_.data.size());
    assert(chain_.getNrOfJoints() == joint_max_.data.size());

    for (uint i = 0; i < chain_.segments.size(); i++) {
        std::string type = chain_.segments[i].getJoint().getTypeName();
        if (type.find("Rot") != std::string::npos) {
            if (joint_max_(joint_types_.size()) >= std::numeric_limits<float>::max() &&
                joint_min_(joint_types_.size()) <= std::numeric_limits<float>::lowest())
            {
                joint_types_.push_back(KDL::BasicJointType::Continuous);
            } else {
                joint_types_.push_back(KDL::BasicJointType::RotJoint);
            }
        } else if (type.find("Trans") != std::string::npos) {
            joint_types_.push_back(KDL::BasicJointType::TransJoint);
        }
    }

    assert(joint_types_.size() == joint_min_.data.size());
}

void TRAC_IK::setBounds(const KDL::Twist& bounds)
{
    bounds_ = bounds;
    nl_solver_.setBounds(bounds);
    ik_solver_.setBounds(bounds);
}

bool TRAC_IK::unique_solution(const KDL::JntArray& sol)
{
    auto myEqual = [](const KDL::JntArray& a, const KDL::JntArray& b) {
        return (a.data - b.data).isZero(1e-4);
    };

    for (uint i = 0; i < solutions_.size(); i++) {
        if (myEqual(sol, solutions_[i])) {
            return false;
        }
    }
    return true;
}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
    if (val > max) {
        //Find actual angle offset
        double diffangle = fmod(val - max, 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        val = max + diffangle - 2 * M_PI;
    }

    if (val < min) {
        //Find actual angle offset
        double diffangle = fmod(min - val, 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        val = min - diffangle + 2 * M_PI;
    }
}

inline void normalizeAngle(double& val, const double& target)
{
    if (val > target + M_PI) {
        //Find actual angle offset
        double diffangle = fmod(val - target, 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        val = target + diffangle - 2 * M_PI;
    }

    if (val < target - M_PI) {
        //Find actual angle offset
        double diffangle = fmod(target - val, 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        val = target - diffangle + 2 * M_PI;
    }
}

void TRAC_IK::randomize(
    KDL::JntArray& q,
    const KDL::JntArray& q_init) const
{
    for (size_t j = 0; j < q.data.size(); ++j) {
        if (joint_types_[j] == KDL::BasicJointType::Continuous) {
            q(j) = fRand(q_init(j) - 2.0 * M_PI, q_init(j) + 2.0 * M_PI);
        }
        else {
            q(j) = fRand(joint_min_(j), joint_max_(j));
        }
    }
}

void TRAC_IK::normalize_seed(
    const KDL::JntArray& seed,
    KDL::JntArray& solution)
{
    // Make sure rotational joint values are within 1 revolution of seed; then
    // ensure joint limits are met.

    for (uint i = 0; i < joint_min_.data.size(); i++) {
        if (joint_types_[i] == KDL::BasicJointType::TransJoint) {
            continue;
        }

        double target = seed(i);
        double val = solution(i);

        normalizeAngle(val, target);

        if (joint_types_[i] == KDL::BasicJointType::Continuous) {
            solution(i) = val;
            continue;
        }

        normalizeAngle(val, joint_min_(i), joint_max_(i));

        solution(i) = val;
    }
}

void TRAC_IK::normalize_limits(
    const KDL::JntArray& seed,
    KDL::JntArray& solution)
{
    // Make sure rotational joint values are within 1 revolution of middle of
    // limits; then ensure joint limits are met.

    for (uint i = 0; i < joint_min_.data.size(); i++) {
        if (joint_types_[i] == KDL::BasicJointType::TransJoint) {
            continue;
        }

        double target = seed(i);

        if (joint_types_[i] == KDL::BasicJointType::RotJoint &&
            joint_types_[i] != KDL::BasicJointType::Continuous)
        {
            target = (joint_max_(i) + joint_min_(i)) / 2.0;
        }

        double val = solution(i);

        normalizeAngle(val, target);

        if (joint_types_[i] == KDL::BasicJointType::Continuous) {
            solution(i) = val;
            continue;
        }

        normalizeAngle(val, joint_min_(i), joint_max_(i));

        solution(i) = val;
    }
}

double TRAC_IK::manipPenalty(const KDL::JntArray& arr)
{
    double penalty = 1.0;
    for (uint i = 0; i < arr.data.size(); i++) {
        if (joint_types_[i] == KDL::BasicJointType::Continuous) {
            continue;
        }
        double range = joint_max_(i) - joint_min_(i);
        penalty *= ((arr(i) - joint_min_(i)) * (joint_max_(i) - arr(i)) / (range * range));
    }
    return std::max(0.0, 1.0 - exp(-1 * penalty));
}

double TRAC_IK::ManipValue1(const KDL::JntArray& arr)
{
    KDL::Jacobian jac(arr.data.size());

    jac_solver_.JntToJac(arr, jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();

    double error = 1.0;
    for (unsigned int i = 0; i < singular_values.rows(); ++i) {
        error *= singular_values(i, 0);
    }
    return error;
}

double TRAC_IK::ManipValue2(const KDL::JntArray& arr)
{
    KDL::Jacobian jac(arr.data.size());

    jac_solver_.JntToJac(arr, jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();

    return singular_values.minCoeff() / singular_values.maxCoeff();
}

int TRAC_IK::CartToJnt(
    const KDL::JntArray &q_init,
    const KDL::Frame &p_in,
    KDL::JntArray &q_out,
    const KDL::Twist& bounds)
{
    solutions_.clear();
    errors_.clear();

    ik_solver_.setBounds(bounds);
    nl_solver_.setBounds(bounds);

    for (unsigned int jidx = 0; jidx < chain_.getNrOfJoints(); ++jidx) {
        seed_(jidx) = q_init(jidx);
    }

    ik_solver_.restart(seed_, p_in);
    nl_solver_.restart(seed_, p_in);

    enum Solver {
        SOLVER_KDL,
        SOLVER_NLOPT,

        SOLVER_COUNT
    };

    int solver = SOLVER_KDL;
//    int solver = SOLVER_NLOPT;

    const int step_size = 50;
    const int max_iters = max_iters_ / step_size;
    for (int i = 0; i < max_iters; ++i) {
        // interleave iterations of kdl and nl opt
        switch (solver) {
        case SOLVER_KDL: {
            auto before = std::chrono::high_resolution_clock::now();
            int rc = ik_solver_.step(step_size);
            auto after = std::chrono::high_resolution_clock::now();
            ROS_DEBUG_THROTTLE_NAMED(1.0, "trac_ik", "kdl step took %f seconds", std::chrono::duration<double>(after - before).count());
            if (rc == 0) {
                ROS_DEBUG_NAMED("trac_ik", "KDL found solution on iteration %d", i);

                q_out = ik_solver_.qout();

                if (solve_type_ == Speed) {
                    return 0; // first solution returned
                }

                switch (solve_type_) {
                case Manip1:
                case Manip2:
                    normalize_limits(q_init, q_out);
                    break;
                default:
                    normalize_seed(q_init, q_out);
                    break;
                }

                if (unique_solution(q_out)) {
                    solutions_.push_back(q_out);
                    double err;
                    switch (solve_type_) {
                    case Manip1:
                        err = manipPenalty(q_out) * TRAC_IK::ManipValue1(q_out);
                        break;
                    case Manip2:
                        err = manipPenalty(q_out) * TRAC_IK::ManipValue2(q_out);
                        break;
                    default:
                        err = JointErr(q_init, q_out);
                        break;
                    }

                    errors_.emplace_back(err, solutions_.size() - 1);
                }

                // sample a new random seed to search for additional solutions
                // on successive iterations
                randomize(seed_, q_init);
                ik_solver_.restart(seed_);
            }

            solver = SOLVER_NLOPT;
        }   break;
        case SOLVER_NLOPT: {
            auto before = std::chrono::high_resolution_clock::now();
            int rc =  nl_solver_.step(step_size); // for some reason, 1 and 2 produce no solutions
            auto after = std::chrono::high_resolution_clock::now();
            ROS_DEBUG_THROTTLE_NAMED(1.0, "trac_ik", "nlopt step took %f seconds", std::chrono::duration<double>(after - before).count());
            if (rc == 0) {
                ROS_DEBUG_NAMED("trac_ik", "NLOPT found solution on iteration %d", i);

                q_out = nl_solver_.qout();

                if (solve_type_ == Speed) {
                    return 0; // first solution returned
                }

                switch (solve_type_) {
                case Manip1:
                case Manip2:
                    normalize_limits(q_init, q_out);
                    break;
                default:
                    normalize_seed(q_init, q_out);
                    break;
                }

                if (unique_solution(q_out)) {
                    solutions_.push_back(q_out);
                    double err;
                    switch (solve_type_) {
                    case Manip1:
                        err = manipPenalty(q_out) * TRAC_IK::ManipValue1(q_out);
                        break;
                    case Manip2:
                        err = manipPenalty(q_out) * TRAC_IK::ManipValue2(q_out);
                        break;
                    default:
                        err = JointErr(q_init, q_out);
                        break;
                    }

                    errors_.emplace_back(err, solutions_.size() - 1);
                }

                randomize(seed_, q_init);
                nl_solver_.restart(seed_);
            }

            solver = SOLVER_KDL;
        }   break;
        default: {
            ROS_ERROR_NAMED("trac_ik", "Unknown solver type");
            assert(0);
        }   break;
        }
    }

    if (solutions_.empty()) {
        ROS_DEBUG_NAMED("trac_ik", "Failed to find solution");
        return -3;
    }

    using solution_error = std::pair<double, unsigned int>;
    auto comp_error = [](const solution_error& p, const solution_error& q) {
        return p.first < q.first;
    };

    switch (solve_type_) {
    case Manip1:
    case Manip2:
        std::sort(errors_.rbegin(), errors_.rend(), comp_error); // rbegin/rend to sort by max
        break;
    default:
        std::sort(errors_.begin(), errors_.end(), comp_error);
        break;
    }

    q_out = solutions_[errors_[0].second];
    return solutions_.size();
}

} // namespace TRAC_IK
