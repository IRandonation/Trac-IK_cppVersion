#include "nlopt_ik.h"

#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>
#include <memory>

#include <Eigen/Dense>

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
        progress_(-3)  // -3: 未开始, -1: 失败, 1: 成功
    {
        assert(chain_.getNrOfJoints() == joint_min_.data.size());
        assert(chain_.getNrOfJoints() == joint_max_.data.size());

        // 初始化关节类型（可选，如果你后续要用）
        for (size_t i = 0; i < chain_.segments.size(); i++) {
            std::string type = chain_.segments[i].getJoint().getTypeName();
            if (type.find("Rot") != std::string::npos) {
                if (joint_max_(joint_types_.size()) >= std::numeric_limits<float>::max() &&
                    joint_min_(joint_types_.size()) <= std::numeric_limits<float>::lowest()) {
                    joint_types_.push_back(BasicJointType::Continuous);
                } else {
                    joint_types_.push_back(BasicJointType::RotJoint);
                }
            } else if (type.find("Trans") != std::string::npos) {
                joint_types_.push_back(BasicJointType::TransJoint);
            }
        }

        // 初始化优化变量边界
        x_min_.resize(chain_.getNrOfJoints());
        x_max_.resize(chain_.getNrOfJoints());
        best_x_.resize(chain_.getNrOfJoints());
        target_pos_.resize(chain_.getNrOfJoints());

        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            x_min_[i] = joint_min_(i);
            x_max_[i] = joint_max_(i);
            target_pos_[i] = 0.0;  // 用于 Joint 最小化目标
        }

        // 构造 NLOPT 优化器
        nlopt_ = std::make_unique<nlopt::opt>(nlopt::LD_LBFGS, chain_.getNrOfJoints());
        nlopt_->set_lower_bounds(x_min_);
        nlopt_->set_upper_bounds(x_max_);
        nlopt_->set_ftol_rel(eps_);
        nlopt_->set_maxtime(maxtime_);

        // 根据优化类型设置目标函数
        switch (opt_type_) {
            case Joint:
                nlopt_->set_min_objective([](const std::vector<double>& v,
                                             std::vector<double>& g,
                                             void* d) -> double {
                    return static_cast<NLOPT_IK*>(d)->minJoints(v, g);
                }, this);
                break;
            case SumSq:
            case L2:
                nlopt_->set_min_objective([](const std::vector<double>& v,
                                             std::vector<double>& g,
                                             void* d) -> double {
                    double err[1] = {0};
                    static_cast<NLOPT_IK*>(d)->cartSumSquaredError(v, err);
                    if (!g.empty()) {
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
        }
    }

    void NLOPT_IK::restart(const KDL::JntArray& q_init, const KDL::Frame& p_in) {
        progress_ = -3;
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            best_x_[i] = q_init(i);
        }
        f_target_ = p_in;
    }

    void NLOPT_IK::restart(const KDL::JntArray& q_init) {
        progress_ = -3;
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            best_x_[i] = q_init(i);
        }
    }

    int NLOPT_IK::step(int steps) {
        (void)steps;
        if (progress_ == 1) return 0;

        double minf;
        try {
            bool res = optimize(best_x_, minf);
            if (res) {
                progress_ = 1;
                for (size_t i = 0; i < best_x_.size(); ++i) {
                    q_out_(i) = best_x_[i];
                }
                return 0;
            }
        } catch (...) {
        }
        return -1;
    }

    double NLOPT_IK::minJoints(const std::vector<double>& x, std::vector<double>& grad) {
        double err = 0.0;
        for (size_t i = 0; i < x.size(); i++) {
            err += (x[i] - target_pos_[i]) * (x[i] - target_pos_[i]);
            if (!grad.empty()) {
                grad[i] = 2.0 * (x[i] - target_pos_[i]);
            }
        }
        return err;
    }

    void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[]) {
        auto& q = q_tmp_;
        for (size_t i = 0; i < chain_.getNrOfJoints(); i++) {
            q(i) = x[i];
        }

        KDL::Frame currentPose;
        int rc = fk_solver_.JntToCart(q, currentPose);
        if (rc < 0) {
            std::cerr << "[NLOPT_IK] KDL FKSolver failed!" << std::endl;
            error[0] = 1e9;
            return;
        }

        KDL::Twist delta = KDL::diff(f_target_, currentPose);

        // 可选：剔除在误差范围内的分量
        if (std::abs(delta.vel.x()) <= std::abs(bounds_.vel.x())) delta.vel.x(0);
        if (std::abs(delta.vel.y()) <= std::abs(bounds_.vel.y())) delta.vel.y(0);
        if (std::abs(delta.vel.z()) <= std::abs(bounds_.vel.z())) delta.vel.z(0);

        if (std::abs(delta.rot.x()) <= std::abs(bounds_.rot.x())) delta.rot.x(0);
        if (std::abs(delta.rot.y()) <= std::abs(bounds_.rot.y())) delta.rot.y(0);
        if (std::abs(delta.rot.z()) <= std::abs(bounds_.rot.z())) delta.rot.z(0);

        error[0] = KDL::dot(delta.vel, delta.vel) + KDL::dot(delta.rot, delta.rot);

        if (KDL::Equal(delta, KDL::Twist::Zero(), eps_)) {
            progress_ = 1;
        }
    }

    void NLOPT_IK::cartL2NormError(const std::vector<double>& x, double error[]) {
        auto& q = q_tmp_;
        for (size_t i = 0; i < chain_.getNrOfJoints(); i++) {
            q(i) = x[i];
        }

        KDL::Frame currentPose;
        int rc = fk_solver_.JntToCart(q, currentPose);
        if (rc < 0) {
            std::cerr << "[NLOPT_IK] KDL FKSolver failed!" << std::endl;
            error[0] = 1e9;
            return;
        }

        KDL::Twist delta = KDL::diff(f_target_, currentPose);

        if (std::abs(delta.vel.x()) <= std::abs(bounds_.vel.x())) delta.vel.x(0);
        if (std::abs(delta.vel.y()) <= std::abs(bounds_.vel.y())) delta.vel.y(0);
        if (std::abs(delta.vel.z()) <= std::abs(bounds_.vel.z())) delta.vel.z(0);

        if (std::abs(delta.rot.x()) <= std::abs(bounds_.rot.x())) delta.rot.x(0);
        if (std::abs(delta.rot.y()) <= std::abs(bounds_.rot.y())) delta.rot.y(0);
        if (std::abs(delta.rot.z()) <= std::abs(bounds_.rot.z())) delta.rot.z(0);

        error[0] = std::sqrt(KDL::dot(delta.vel, delta.vel) + KDL::dot(delta.rot, delta.rot));

        if (KDL::Equal(delta, KDL::Twist::Zero(), eps_)) {
            progress_ = 1;
        }
    }

    bool NLOPT_IK::optimize(std::vector<double>& x, double& minf) {
        nlopt::opt& opt = *nlopt_;
        switch (opt_type_) {
            case Joint:
                opt.set_min_objective([](const std::vector<double>& v,
                                         std::vector<double>& g,
                                         void* d) -> double {
                    return static_cast<NLOPT_IK*>(d)->minJoints(v, g);
                }, this);
                break;
            case SumSq:
            case L2:
                opt.set_min_objective([](const std::vector<double>& v,
                                         std::vector<double>& g,
                                         void* d) -> double {
                    double err[1] = {0};
                    static_cast<NLOPT_IK*>(d)->cartSumSquaredError(v, err);
                    if (!g.empty()) {
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
        }

        try {
            nlopt::result res = opt.optimize(x, minf);
            return (res == nlopt::SUCCESS || res == nlopt::XTOL_REACHED || res == nlopt::MAXTIME_REACHED);
        } catch (const nlopt::forced_stop&) {
            return false;
        }
    }

    void NLOPT_IK::clipToJointLimits(std::vector<double>& x) {
        for (size_t i = 0; i < x.size(); i++) {
            if (joint_types_[i] != BasicJointType::Continuous) {
                if (x[i] < x_min_[i]) x[i] = x_min_[i];
                if (x[i] > x_max_[i]) x[i] = x_max_[i];
            }
        }
    }

    int NLOPT_IK::CartToJnt(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& bounds) {
        
        // 设置边界
        bounds_ = bounds;
        
        // 重置求解器状态
        restart(q_init, p_in);
        
        // 执行优化
        double minf;
        bool success = optimize(best_x_, minf);
        
        // 将结果复制到输出数组
        for (size_t i = 0; i < best_x_.size(); ++i) {
            q_out_(i) = best_x_[i];
        }
        
        // 确保解在关节限制内
        clipToJointLimits(best_x_);
        
        // 再次检查解是否满足要求
        if (success) {
            // 验证解
            auto& q = q_tmp_;
            for (size_t i = 0; i < chain_.getNrOfJoints(); i++) {
                q(i) = best_x_[i];
            }
            
            KDL::Frame currentPose;
            int rc = fk_solver_.JntToCart(q, currentPose);
            if (rc >= 0) {
                KDL::Twist delta = KDL::diff(f_target_, currentPose);
                
                // 检查是否在误差范围内
                if (std::abs(delta.vel.x()) <= std::abs(bounds_.vel.x()) &&
                    std::abs(delta.vel.y()) <= std::abs(bounds_.vel.y()) &&
                    std::abs(delta.vel.z()) <= std::abs(bounds_.vel.z()) &&
                    std::abs(delta.rot.x()) <= std::abs(bounds_.rot.x()) &&
                    std::abs(delta.rot.y()) <= std::abs(bounds_.rot.y()) &&
                    std::abs(delta.rot.z()) <= std::abs(bounds_.rot.z())) {
                    
                    // 解有效，复制到输出
                    q_out = q_out_;
                    return 0;
                }
            }
        }
        
        // 如果没有找到有效解，返回当前最佳解
        q_out = q_out_;
        return -1;
    }

} // namespace NLOPT_IK