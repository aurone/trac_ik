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

#include <trac_ik/nlopt_ik.hpp>

// standard includes
#include <cmath>
#include <limits>

// standard includes
#include <boost/date_time.hpp>
#include <ros/ros.h>

// project includes
#include <trac_ik/dual_quaternion.h>
#include <trac_ik/utils.h>

namespace NLOPT_IK {

double minfunc(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data);

double minfuncDQ(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data);

double minfuncSumSquared(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data);

double minfuncL2(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data);

void constrainfuncm(
    uint m,
    double* result,
    uint n,
    const double* x,
    double* grad,
    void* data);

// Auxiliary function to minimize (Sum of Squared joint angle error from the
// requested configuration).  Because we wanted a Class without static members,
// but NLOpt library does not support passing methods of Classes, we use these
// auxilary functions.
double minfunc(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data)
{
    NLOPT_IK* c = (NLOPT_IK*)data;
    return c->minJoints(x, grad);
}

// Auxiliary function to minimize (Sum of Squared joint angle error from the
// requested configuration).  Because we wanted a Class without static members,
// but NLOpt library does not support passing methods of Classes, we use these
// auxilary functions.
double minfuncDQ(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data)
{
    NLOPT_IK* c = (NLOPT_IK*)data;

    std::vector<double> vals(x);

    double jump = boost::math::tools::epsilon<float>();
    double result[1];
    c->cartDQError(vals, result);

    if (!grad.empty()) {
        double v1[1];
        for (uint i = 0; i < x.size(); i++) {
            double original = vals[i];

            vals[i] = original + jump;
            c->cartDQError(vals, v1);

            vals[i] = original;
            grad[i] = (v1[0] - result[0]) / (2 * jump);
        }
    }

    return result[0];
}

// Auxiliary function to minimize (Sum of Squared joint angle error from the
// requested configuration).
double minfuncSumSquared(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data)
{
    NLOPT_IK* c = (NLOPT_IK*)data;

    std::vector<double> vals(x);

    double jump = boost::math::tools::epsilon<float>();
    double result[1];
    c->cartSumSquaredError(vals, result);

    if (!grad.empty()) {
        double v1[1];
        for (uint i = 0; i < x.size(); i++) {
            double original = vals[i];

            vals[i] = original + jump;
            c->cartSumSquaredError(vals, v1);

            vals[i] = original;
            grad[i] = (v1[0] - result[0]) / (2.0 * jump);
        }
    }

    return result[0];
}

// Auxiliary function to minimize (Sum of Squared joint angle error from the
// requested configuration).  Because we wanted a Class without static members,
// but NLOpt library does not support passing methods of Classes, we use these
// auxilary functions.
double minfuncL2(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data)
{
    NLOPT_IK* c = (NLOPT_IK*)data;

    std::vector<double> vals(x);

    double jump = boost::math::tools::epsilon<float>();
    double result[1];
    c->cartL2NormError(vals, result);

    if (!grad.empty()) {
        double v1[1];
        for (uint i = 0; i < x.size(); i++) {
            double original = vals[i];

            vals[i] = original + jump;
            c->cartL2NormError(vals, v1);

            vals[i] = original;
            grad[i] = (v1[0] - result[0]) / (2.0 * jump);
        }
    }

    return result[0];
}

// Equality constraint auxilary function for Euclidean distance. This also uses
// a small walk to approximate the gradient of the constraint function at the
// current joint angles.
void constrainfuncm(
    uint m,
    double* result,
    uint n,
    const double* x,
    double* grad,
    void* data)
{
    NLOPT_IK* c = (NLOPT_IK*)data;

    std::vector<double> vals(n);

    for (uint i = 0; i < n; i++) {
        vals[i] = x[i];
    }

    double jump = boost::math::tools::epsilon<float>();

    c->cartSumSquaredError(vals, result);

    if (grad != NULL) {
        std::vector<double> v1(m);
        for (uint i = 0; i < n; i++) {
            double o = vals[i];
            vals[i] = o + jump;
            c->cartSumSquaredError(vals, v1.data());
            vals[i] = o;
            for (uint j = 0; j < m; j++) {
                grad[j * n + i] = (v1[j] - result[j]) / (2 * jump);
            }
        }
    }
}

// Constructor for an IK Class. Takes in a Chain to operate on, the min and max
// joint limits, an (optional) maximum number of iterations, and an (optional)
// desired error.
NLOPT_IK::NLOPT_IK(
    const KDL::Chain& chain,
    const KDL::JntArray& q_min,
    const KDL::JntArray& q_max,
    double eps,
    OptType _type)
:
    chain_(chain),
    joint_min_(),
    joint_max_(),
    types_(),
    valid_(true),
    fk_solver_(chain),
    eps_(std::abs(eps)),
    best_x_(chain.getNrOfJoints()),
    x_min_(chain.getNrOfJoints()),
    x_max_(chain.getNrOfJoints()),
    opt_type_(_type),
    q_out_(chain.getNrOfJoints())
{
    /////////////////////////////////////
    // Initialize KDL Chain Properties //
    /////////////////////////////////////

    assert(chain_.getNrOfJoints() == q_min.data.size());
    assert(chain_.getNrOfJoints() == q_max.data.size());

    if (chain_.getNrOfJoints() < 2) {
        ROS_WARN_THROTTLE(1.0, "NLOpt_IK can only be run for chains of length 2 or more");
        valid_ = false;
        return;
    }

    for (uint i = 0; i < chain_.getNrOfJoints(); i++) {
        joint_min_.push_back(q_min(i));
        joint_max_.push_back(q_max(i));
    }

    for (uint i = 0; i < chain_.segments.size(); i++) {
        std::string type = chain_.segments[i].getJoint().getTypeName();
        if (type.find("Rot") != std::string::npos) {
            if (q_max(types_.size()) >= std::numeric_limits<float>::max() && q_min(types_.size()) <= std::numeric_limits<float>::lowest()) {
                types_.push_back(KDL::BasicJointType::Continuous);
            }
            else {
                types_.push_back(KDL::BasicJointType::RotJoint);
            }
        } else if (type.find("Trans") != std::string::npos) {
            types_.push_back(KDL::BasicJointType::TransJoint);
        }
    }

    assert(types_.size() == joint_min_.size());

    ///////////////////////
    // Initialize solver //
    ///////////////////////

    nlopt_ = nlopt::opt(nlopt::LD_SLSQP, chain_.getNrOfJoints());

    std::vector<double> tolerance(1, boost::math::tools::epsilon<float>());
    nlopt_.set_xtol_abs(tolerance[0]);

    switch (opt_type_) {
    case Joint:
        nlopt_.set_min_objective(minfunc, this);
        nlopt_.add_equality_mconstraint(constrainfuncm, this, tolerance);
        break;
    case DualQuat:
        nlopt_.set_min_objective(minfuncDQ, this);
        break;
    case SumSq:
        nlopt_.set_min_objective(minfuncSumSquared, this);
        break;
    case L2:
        nlopt_.set_min_objective(minfuncL2, this);
        break;
    }
}

void NLOPT_IK::restart(
    const KDL::JntArray& q_init,
    const KDL::Frame& p_in)
{
    assert(q_init.data.size() == types_.size());

    progress_ = -3;

    f_target_ = p_in;

    //////////////////////////////////////////////////////////////////
    // pre-process initial state to be a valid state for the solver //
    //////////////////////////////////////////////////////////////////

    // 1. clamp prismatic joints
    // 2. find nearest in-bounds, 2*pi equivalent of revolute joint
    // 3. nothing for continuous joints
    for (size_t i = 0; i < best_x_.size(); ++i) {
        best_x_[i] = q_init(i);

        if (types_[i] == KDL::BasicJointType::Continuous) {
            continue;
        }

        if (types_[i] == KDL::BasicJointType::TransJoint) {
            best_x_[i] = std::min(best_x_[i], joint_max_[i]);
            best_x_[i] = std::max(best_x_[i], joint_min_[i]);
        } else {
            // Below is to handle bad seeds outside of limits

            if (best_x_[i] > joint_max_[i]) {
                // Find actual angle offset
                double diffangle = fmod(best_x_[i] - joint_max_[i], 2.0 * M_PI);
                // Add that to upper bound and go back a full rotation
                best_x_[i] = joint_max_[i] + diffangle - 2.0 * M_PI;
            }

            if (best_x_[i] < joint_min_[i]) {
                //Find actual angle offset
                double diffangle = fmod(joint_min_[i] - best_x_[i], 2.0 * M_PI);
                // Subtract that from lower bound and go forward a full rotation
                best_x_[i] = joint_min_[i] - diffangle + 2.0 * M_PI;
            }

            if (best_x_[i] > joint_max_[i]) {
                best_x_[i] = 0.5 * (joint_max_[i] + joint_min_[i]);
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    // determine reasonable upper and lower limits for the solver //
    ////////////////////////////////////////////////////////////////

    for (size_t i = 0; i < joint_min_.size(); i++) {
        if (types_[i] == KDL::BasicJointType::Continuous) {
            x_min_[i] = best_x_[i] - 2 * M_PI;
        } else if (types_[i] == KDL::BasicJointType::TransJoint) {
            x_min_[i] = joint_min_[i];
        } else {
            x_min_[i] = std::max(joint_min_[i], best_x_[i] - 2 * M_PI);
        }
    }

    for (size_t i = 0; i < joint_max_.size(); i++) {
        if (types_[i] == KDL::BasicJointType::Continuous) {
            x_max_[i] = best_x_[i] + 2 * M_PI;
        }
        else if (types_[i] == KDL::BasicJointType::TransJoint) {
            x_max_[i] = joint_max_[i];
        }
        else {
            x_max_[i] = std::min(joint_max_[i], best_x_[i] + 2 * M_PI);
        }
    }

    nlopt_.set_lower_bounds(x_min_);
    nlopt_.set_upper_bounds(x_max_);

    ///////////////////////////////////////////////////////////////////////
    // set a desired pose for the solver cost function (seed by default) //
    ///////////////////////////////////////////////////////////////////////

    switch (opt_type_) {
    case OptType::Joint: {
        target_pos_ = best_x_;

//        if (q_desired.data.size() == 0) {
//            target_pos_ = best_x_; // initial seed
//        } else {
//            target_pos_.resize(x.size());
//            for (uint i = 0; i < target_pos_.size(); i++) {
//                target_pos_[i] = q_desired(i);
//            }
//        }
    }   break;
    case OptType::DualQuat: {
        math3d::matrix3x3<double> targetRotationMatrix(f_target_.M.data);
        math3d::quaternion<double> targetQuaternion =
                math3d::rot_matrix_to_quaternion<double>(targetRotationMatrix);
        math3d::point3d targetTranslation(f_target_.p.data);
        target_dq_ = dual_quaternion::rigid_transformation(targetQuaternion, targetTranslation);
    }   break;
    case OptType::SumSq:
        break;
    case OptType::L2:
        break;
    default:
        break;
    }

    nlopt_.set_maxeval(10);
}

void NLOPT_IK::restart(const KDL::JntArray& q_init)
{
    restart(q_init, f_target_);
}

int NLOPT_IK::step()
{
    if (!valid_) {
        ROS_ERROR_THROTTLE(1.0, "NLOpt_IK can only be run for chains of length 2 or more");
        return -3;
    }

    if (progress_ == 1) {
        return 0;
    }

    double minf; // the minimum objective value, upon return

    try {
        nlopt_.optimize(best_x_, minf);
    } catch (...) {
    }

    if (progress_ == 1)  { // copy solution
        for (size_t i = 0; i < best_x_.size(); ++i) {
            q_out_(i) = best_x_[i];
        }
        return 0;
    } else {
        return 1;
    }
}

const KDL::JntArray& NLOPT_IK::qout() const
{
    return q_out_;
}

double NLOPT_IK::minJoints(
    const std::vector<double>& x,
    std::vector<double>& grad)
{
    // Actual function to compute the error between the current joint
    // configuration and the desired.  The SSE is easy to provide a
    // closed form gradient for.

    bool gradient = !grad.empty();

    double err = 0;
    for (uint i = 0; i < x.size(); i++) {
        err += pow(x[i] - target_pos_[i], 2);
        if (gradient) {
            grad[i] = 2.0 * (x[i] - target_pos_[i]);
        }
    }

    return err;

}

void NLOPT_IK::cartSumSquaredError(
    const std::vector<double>& x,
    double error[])
{
    // Actual function to compute Euclidean distance error.  This uses
    // the KDL Forward Kinematics solver to compute the Cartesian pose
    // of the current joint configuration and compares that to the
    // desired Cartesian pose for the IK solve.

    if (progress_ != -3) {
        nlopt_.force_stop();
        return;
    }

    KDL::JntArray q(x.size());

    for (uint i = 0; i < x.size(); i++) {
        q(i) = x[i];
    }

    KDL::Frame currentPose;
    int rc = fk_solver_.JntToCart(q, currentPose);

    if (rc < 0) {
        ROS_FATAL_STREAM("KDL FKSolver is failing: " << q.data);
    }

    KDL::Twist delta_twist = KDL::diffRelative(f_target_, currentPose);

    for (int i = 0; i < 6; i++) {
        if (std::abs(delta_twist[i]) <= std::abs(bounds_[i])) {
            delta_twist[i] = 0.0;
        }
    }

    error[0] = KDL::dot(delta_twist.vel, delta_twist.vel) +
            KDL::dot(delta_twist.rot, delta_twist.rot);

    if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps_)) {
        progress_ = 1;
        best_x_ = x;
    }
}

// Actual function to compute Euclidean distance error. This uses the KDL
// Forward Kinematics solver to compute the Cartesian pose of the current joint
// configuration and compares that to the desired Cartesian pose for the IK
// solve.
void NLOPT_IK::cartL2NormError(const std::vector<double>& x, double error[])
{
    if (progress_ != -3) {
        nlopt_.force_stop();
        return;
    }

    KDL::JntArray q(x.size());

    for (uint i = 0; i < x.size(); i++) {
        q(i) = x[i];
    }

    KDL::Frame currentPose;
    int rc = fk_solver_.JntToCart(q, currentPose);

    if (rc < 0) {
        ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q.data);
    }

    KDL::Twist delta_twist = KDL::diffRelative(f_target_, currentPose);

    for (int i = 0; i < 6; i++) {
        if (std::abs(delta_twist[i]) <= std::abs(bounds_[i])) {
            delta_twist[i] = 0.0;
        }
    }

    error[0] = std::sqrt(
            KDL::dot(delta_twist.vel, delta_twist.vel) +
            KDL::dot(delta_twist.rot, delta_twist.rot));

    if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps_)) {
        progress_ = 1;
        best_x_ = x;
    }
}

// Actual function to compute Euclidean distance error. This uses the KDL
// Forward Kinematics solver to compute the Cartesian pose of the current joint
// configuration and compares that to the desired Cartesian pose for the IK
// solve.
void NLOPT_IK::cartDQError(const std::vector<double>& x, double error[])
{
    if (progress_ != -3) {
        nlopt_.force_stop();
        return;
    }

    KDL::JntArray q(x.size());

    for (uint i = 0; i < x.size(); i++) {
        q(i) = x[i];
    }

    KDL::Frame currentPose;
    int rc = fk_solver_.JntToCart(q, currentPose);

    if (rc < 0) {
        ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q.data);
    }

    KDL::Twist delta_twist = KDL::diffRelative(f_target_, currentPose);

    for (int i = 0; i < 6; i++) {
        if (std::abs(delta_twist[i]) <= std::abs(bounds_[i])) {
            delta_twist[i] = 0.0;
        }
    }

    math3d::matrix3x3<double> currentRotationMatrix(currentPose.M.data);
    math3d::quaternion<double> currentQuaternion =
            math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
    math3d::point3d currentTranslation(currentPose.p.data);
    dual_quaternion currentDQ = dual_quaternion::rigid_transformation(
            currentQuaternion, currentTranslation);

    dual_quaternion errorDQ = (currentDQ * !target_dq_).normalize();
    errorDQ.log();
    error[0] = 4.0f * dot(errorDQ, errorDQ);

    if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps_)) {
        progress_ = 1;
        best_x_ = x;
    }
}

/// User command to start an IK solve. Takes in a seed configuration, a
/// Cartesian pose, and (optional) a desired configuration. If the desired is
/// not provided, the seed is used. Outputs the joint configuration found that
/// solves the IK.
///
/// Returns -3 if a configuration could not be found within the eps set up in
/// the constructor.
int NLOPT_IK::CartToJnt(
    const KDL::JntArray& q_init,
    const KDL::Frame& p_in,
    KDL::JntArray& q_out,
    const KDL::Twist _bounds,
    const KDL::JntArray& q_desired)
{
    bounds_ = _bounds;
    restart(q_init, p_in);

    const int max_iterations = 100;
    int res = 1;
    int i = 0;
    while (i < max_iterations && res != 0) {
        res = step();
        ++i;
    }

    if (res == 0) {
        q_out = qout();
    }

    return -res;

    return progress_;
}

} // namespace NLOPT_IK
