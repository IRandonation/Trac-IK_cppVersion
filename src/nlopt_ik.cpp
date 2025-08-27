#include "nlopt_ik.h"

#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>

namespace NLOPT_IK {

    NLOPT_IK::NLOPT_IK(
        const KDL::Chain& chain,
        const KDL::JntArray& q_min,
        const KDL::JntArray& q_max,
        double maxtime,
        double eps,
        OptType type)
    :
        chain_(chain),
        joint_min_(q_min),
        joint_max_(q_max),
        fk_solver_(chain),
        q_tmp_(chain.getNrOfJoints()),
        maxtime_(maxtime),
        eps_(eps),
        opt_type_(type),
        q_out_(chain.getNrOfJoints()),
        progress_(-3)
    {
        assert(chain_.getNrOfJoints() == joint_min_.data.size());
        assert(chain_.getNrOfJoints() == joint_max_.data.size());

        // Initialize joint types
        for (size_t i = 0; i < chain_.segments.size(); i++) {
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

        // Initialize bounds
        x_min_.resize(chain_.getNrOfJoints());
        x_max_.resize(chain_.getNrOfJoints());
        best_x_.resize(chain_.getNrOfJoints());

        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            x_min_[i] = joint_min_(i);
            x_max_[i] = joint_max_(i);
        }

        // 构造函数末尾
        nlopt_ = std::make_unique<nlopt::opt>(nlopt::LD_LBFGS, chain_.getNrOfJoints());
        nlopt_->set_lower_bounds(x_min_);
        nlopt_->set_upper_bounds(x_max_);
        nlopt_->set_ftol_rel(eps_);
        nlopt_->set_maxtime(maxtime_);

        // Initialize target position for joint minimization
        target_pos_.resize(chain_.getNrOfJoints(), 0.0);
    }

    void NLOPT_IK::restart(const KDL::JntArray& q_init, const KDL::Frame& p_in)
    {
        // Reset optimization state
        progress_ = -3;

        // Copy initial joint values
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            best_x_[i] = q_init(i);
        }

        // Set target frame
        f_target_ = p_in;

        // Initialize target dual quaternion
        math3d::matrix3x3<double> targetRotationMatrix(p_in.M.data);
        math3d::quaternion<double> targetQuaternion =
            math3d::rot_matrix_to_quaternion<double>(targetRotationMatrix);
        math3d::point3d targetTranslation(p_in.p.data);
        target_dq_ = dual_quaternion::rigid_transformation(
            targetQuaternion, targetTranslation);
    }

    void NLOPT_IK::restart(const KDL::JntArray& q_init)
    {
        // Reset optimization state but keep target frame
        progress_ = -3;

        // Copy initial joint values
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            best_x_[i] = q_init(i);
        }
    }

    int NLOPT_IK::step(int steps)
    {
        (void)steps; // Mark as used to avoid warning
        if (progress_ == 1) {
            return 0;
        }

        double minf; // the minimum objective value, upon return

        try {
            optimize(best_x_, minf);
        } catch (...) {
        }

        if (progress_ == 1) { // copy solution
            for (size_t i = 0; i < best_x_.size(); ++i) {
                q_out_(i) = best_x_[i];
            }
            return 0;
        } else {
            return 1;
        }
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
        for (size_t i = 0; i < x.size(); i++) {
            err += pow(x[i] - target_pos_[i], 2);
            if (gradient) {
                grad[i] = 2.0 * (x[i] - target_pos_[i]);
            }
        }

        return err;
    }

    // Actual function to compute Euclidean distance error.  This uses
    // the KDL Forward Kinematics solver to compute the Cartesian pose
    // of the current joint configuration and compares that to the
    // desired Cartesian pose for the IK solve.
    void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[])
    {
        if (progress_ != -3) {
            return;
        }

        auto& q = q_tmp_;

        for (size_t i = 0; i < chain_.getNrOfJoints(); i++) {
            q(i) = x[i];
        }

        KDL::Frame currentPose;
        int rc = fk_solver_.JntToCart(q, currentPose);

        if (rc < 0) {
            std::cerr << "KDL FKSolver is failing" << std::endl;
        }

        KDL::Twist delta_twist = KDL::diffRelative(f_target_, currentPose);

        // Check and zero out velocity components if within bounds
        if (std::abs(delta_twist.vel.x()) <= std::abs(bounds_.vel.x())) {
            delta_twist.vel.x() = 0.0;
        }
        if (std::abs(delta_twist.vel.y()) <= std::abs(bounds_.vel.y())) {
            delta_twist.vel.y() = 0.0;
        }
        if (std::abs(delta_twist.vel.z()) <= std::abs(bounds_.vel.z())) {
            delta_twist.vel.z() = 0.0;
        }
        
        // Check and zero out rotation components if within bounds
        if (std::abs(delta_twist.rot.x()) <= std::abs(bounds_.rot.x())) {
            delta_twist.rot.x() = 0.0;
        }
        if (std::abs(delta_twist.rot.y()) <= std::abs(bounds_.rot.y())) {
            delta_twist.rot.y() = 0.0;
        }
        if (std::abs(delta_twist.rot.z()) <= std::abs(bounds_.rot.z())) {
            delta_twist.rot.z() = 0.0;
        }

        error[0] = delta_twist.vel.dot(delta_twist.vel) +
                delta_twist.rot.dot(delta_twist.rot);

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
            return;
        }

        auto& q = q_tmp_;

        for (size_t i = 0; i < x.size(); i++) {
            q(i) = x[i];
        }

        KDL::Frame currentPose;
        int rc = fk_solver_.JntToCart(q, currentPose);

        if (rc < 0) {
            std::cerr << "KDL FKSolver is failing" << std::endl;
        }

        KDL::Twist delta_twist = KDL::diffRelative(f_target_, currentPose);

        // Check and zero out velocity components if within bounds
        if (std::abs(delta_twist.vel.x()) <= std::abs(bounds_.vel.x())) {
            delta_twist.vel.x() = 0.0;
        }
        if (std::abs(delta_twist.vel.y()) <= std::abs(bounds_.vel.y())) {
            delta_twist.vel.y() = 0.0;
        }
        if (std::abs(delta_twist.vel.z()) <= std::abs(bounds_.vel.z())) {
            delta_twist.vel.z() = 0.0;
        }
        
        // Check and zero out rotation components if within bounds
        if (std::abs(delta_twist.rot.x()) <= std::abs(bounds_.rot.x())) {
            delta_twist.rot.x() = 0.0;
        }
        if (std::abs(delta_twist.rot.y()) <= std::abs(bounds_.rot.y())) {
            delta_twist.rot.y() = 0.0;
        }
        if (std::abs(delta_twist.rot.z()) <= std::abs(bounds_.rot.z())) {
            delta_twist.rot.z() = 0.0;
        }

        error[0] = std::sqrt(
                delta_twist.vel.dot(delta_twist.vel) +
                delta_twist.rot.dot(delta_twist.rot));

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
            return;
        }

        auto& q = q_tmp_;

        for (size_t i = 0; i < x.size(); i++) {
            q(i) = x[i];
        }

        KDL::Frame currentPose;
        int rc = fk_solver_.JntToCart(q, currentPose);

        if (rc < 0) {
            std::cerr << "KDL FKSolver is failing" << std::endl;
        }

        KDL::Twist delta_twist = KDL::diffRelative(f_target_, currentPose);

        // Check and zero out velocity components if within bounds
        if (std::abs(delta_twist.vel.x()) <= std::abs(bounds_.vel.x())) {
            delta_twist.vel.x() = 0.0;
        }
        if (std::abs(delta_twist.vel.y()) <= std::abs(bounds_.vel.y())) {
            delta_twist.vel.y() = 0.0;
        }
        if (std::abs(delta_twist.vel.z()) <= std::abs(bounds_.vel.z())) {
            delta_twist.vel.z() = 0.0;
        }
        
        // Check and zero out rotation components if within bounds
        if (std::abs(delta_twist.rot.x()) <= std::abs(bounds_.rot.x())) {
            delta_twist.rot.x() = 0.0;
        }
        if (std::abs(delta_twist.rot.y()) <= std::abs(bounds_.rot.y())) {
            delta_twist.rot.y() = 0.0;
        }
        if (std::abs(delta_twist.rot.z()) <= std::abs(bounds_.rot.z())) {
            delta_twist.rot.z() = 0.0;
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

    int NLOPT_IK::CartToJnt(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& _bounds)
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

        return progress_;
    }

    bool NLOPT_IK::optimize(std::vector<double>& x, double& minf)
    {
        // 根据模式选择不同目标函数
        nlopt::opt& opt = *nlopt_;
        switch (opt_type_) {
            case Joint:
                opt.set_min_objective([](const std::vector<double>& v,
                                         std::vector<double>& g,
                                         void* d)->double {
                    return static_cast<NLOPT_IK*>(d)->minJoints(v, g);
                }, this);
                break;
            case SumSq:
            case L2:
                opt.set_min_objective([](const std::vector<double>& v,
                                         std::vector<double>& g,
                                         void* d)->double {
                    double err[1];
                    static_cast<NLOPT_IK*>(d)->cartSumSquaredError(v, err);
                    if (!g.empty()) {
                        // 数值梯度示例
                        const double h = 1e-6;
                        for (size_t i = 0; i < v.size(); ++i) {
                            std::vector<double> vh = v;
                            vh[i] += h;
                            double e2[1];
                            static_cast<NLOPT_IK*>(d)->cartSumSquaredError(vh, e2);
                            g[i] = (e2[0] - err[0]) / h;
                        }
                    }
                    return err[0];
                }, this);
                break;
            case DualQuat:
                opt.set_min_objective([](const std::vector<double>& v,
                                         std::vector<double>& g,
                                         void* d)->double {
                    double err[1];
                    static_cast<NLOPT_IK*>(d)->cartDQError(v, err);
                    /* 同理数值梯度 */
                    return err[0];
                }, this);
                break;
        }

        progress_ = -3;
        try {
            nlopt::result res = nlopt_->optimize(x, minf);
            return (res == nlopt::SUCCESS || res == nlopt::XTOL_REACHED || res == nlopt::MAXTIME_REACHED);
        } catch (const nlopt::forced_stop&) {
            return false;
        }
    }

    void NLOPT_IK::clipToJointLimits(std::vector<double>& x)
    {
        for (size_t i = 0; i < x.size(); i++) {
            if (joint_types_[i] != KDL::BasicJointType::Continuous) {
                if (x[i] < x_min_[i]) {
                    x[i] = x_min_[i];
                } else if (x[i] > x_max_[i]) {
                    x[i] = x_max_[i];
                }
            }
        }
    }

} // namespace NLOPT_IK