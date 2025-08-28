#include "kdl_tl.h"

namespace KDL {

    // ======================
    // 构造函数
    // ======================
    ChainIkSolverPos_TL::ChainIkSolverPos_TL(
        const Chain& chain,
        const JntArray& q_min,
        const JntArray& q_max,
        double eps,
        bool random_restart,
        bool try_jl_wrap)
        : chain_(chain),
          joint_min_(q_min),
          joint_max_(q_max),
          fk_solver_(chain),
          vik_solver_(chain),
          q_curr_(chain.getNrOfJoints()),
          q_tmp_(chain.getNrOfJoints()),
          delta_q_(chain.getNrOfJoints()),
          eps_(eps),
          rr_(random_restart),
          wrap_(try_jl_wrap),
          done_(false)
    {
        rng_.seed(std::random_device{}());
        bounds_ = Twist::Zero();

        assert(chain_.getNrOfJoints() == joint_min_.data.size());
        assert(chain_.getNrOfJoints() == joint_max_.data.size());

        for (size_t i = 0; i < chain_.segments.size(); ++i) {
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
    }

    // ======================
    // 设置 / 获取
    // ======================
    void ChainIkSolverPos_TL::setBounds(const Twist& tol) { bounds_ = tol; }
    const Twist& ChainIkSolverPos_TL::bounds() const { return bounds_; }
    void ChainIkSolverPos_TL::setEps(double eps) { eps_ = eps; }
    double ChainIkSolverPos_TL::eps() const { return eps_; }

    // ======================
    // 重启
    // ======================
    void ChainIkSolverPos_TL::restart(const JntArray& q_init, const Frame& p_in) {
        q_curr_ = q_init;
        fk_solver_.JntToCart(q_curr_, f_curr_);
        f_target_ = p_in;
        done_ = false;
    }
    void ChainIkSolverPos_TL::restart(const JntArray& q_init) {
        q_curr_ = q_init;
        fk_solver_.JntToCart(q_curr_, f_curr_);
        done_ = false;
    }

    // ======================
    // 获取当前解
    // ======================
    const JntArray& ChainIkSolverPos_TL::qout() const { return q_curr_; }

    // ======================
    // 一次性求解
    // ======================
    int ChainIkSolverPos_TL::CartToJnt(const JntArray& q_init,
                                       const Frame& p_in,
                                       JntArray& q_out,
                                       const Twist& bounds) {
        setBounds(bounds);
        restart(q_init, p_in);

        const int max_iter = 200;
        for (int k = 0; k < max_iter; ++k) {
            if (step() == 0) {
                q_out = q_curr_;
                return 0;
            }
        }
        if (rr_) {
            randomize(q_curr_);
            for (int k = 0; k < max_iter; ++k) {
                if (step() == 0) {
                    q_out = q_curr_;
                    return 0;
                }
            }
        }
        return -1;
    }

    // ======================
    // 单步迭代
    // ======================
    int ChainIkSolverPos_TL::step(int steps) {
        if (done_) return 0;
        for (int s = 0; s < steps; ++s) {
            fk_solver_.JntToCart(q_curr_, f_curr_);
            Twist err = diff(f_curr_, f_target_);

            bool ok = std::fabs(err.vel.x()) < bounds_.vel.x() &&
                      std::fabs(err.vel.y()) < bounds_.vel.y() &&
                      std::fabs(err.vel.z()) < bounds_.vel.z() &&
                      std::fabs(err.rot.x()) < bounds_.rot.x() &&
                      std::fabs(err.rot.y()) < bounds_.rot.y() &&
                      std::fabs(err.rot.z()) < bounds_.rot.z();
            if (ok) {
                done_ = true;
                return 0;
            }

            vik_solver_.CartToJnt(q_curr_, err, delta_q_);
            Add(q_curr_, delta_q_, q_tmp_);
            clamp(q_tmp_);
            std::swap(q_curr_, q_tmp_);
        }
        return -1;
    }

    // ======================
    // 工具函数
    // ======================

    // 计算相对位姿误差：a * b^-1 -> Twist
    Twist ChainIkSolverPos_TL::diff(const Frame& a, const Frame& b) {
        return KDL::diff(a, b);
    }

    // q1 + dq -> q2
    void ChainIkSolverPos_TL::Add(const JntArray& q1, const JntArray& dq, JntArray& q2) {
        assert(q1.rows() == dq.rows() && dq.rows() == q2.rows());
        for (unsigned int i = 0; i < q1.rows(); i++) {
            q2(i) = q1(i) + dq(i);
        }
    }

    // 关节限位处理（clamping 或 wrapping）
    void ChainIkSolverPos_TL::clamp(JntArray& q) {
        for (unsigned int i = 0; i < q.rows(); ++i) {
            double& v = q(i);
            if (wrap_) {
                double range = joint_max_(i) - joint_min_(i);
                if (range > 0.0) {
                    while (v > joint_max_(i)) v -= range;
                    while (v < joint_min_(i)) v += range;
                }
            } else {
                v = std::max(joint_min_(i), std::min(joint_max_(i), v));
            }
        }
    }

    // 随机初始化（用于随机重启）
    void ChainIkSolverPos_TL::randomize(JntArray& q) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        for (unsigned int i = 0; i < q.rows(); ++i) {
            if (joint_types_[i] == BasicJointType::Continuous) {
                q(i) += dist(rng_) * 4.0 * M_PI - 2.0 * M_PI; // ±2π
            } else {
                q(i) = joint_min_(i) + dist(rng_) * (joint_max_(i) - joint_min_(i));
            }
        }
    }

} // namespace KDL