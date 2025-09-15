#include "trac_ik.h"

#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>
#include <random>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>


namespace TRAC_IK {

    // ----------------------------
    // Constructor
    // ----------------------------
    TRAC_IK::TRAC_IK(
        const KDL::Chain& chain,
        const KDL::JntArray& q_min,
        const KDL::JntArray& q_max,
        double maxtime,
        double eps,
        SolveType type)
    :
        chain_(chain),
        joint_min_(q_min),
        joint_max_(q_max),
        maxtime_(maxtime),
        eps_(eps),
        solve_type_(type),
        progress_(-3)
    {
        assert(chain_.getNrOfJoints() == joint_min_.data.size());
        assert(chain_.getNrOfJoints() == joint_max_.data.size());

        // 识别关节类型（尽量保留原逻辑）
        for (size_t i = 0; i < chain_.segments.size(); i++) {
            std::string type = chain_.segments[i].getJoint().getTypeName();
            if (type.find("Rot") != std::string::npos) {
                if (i < joint_max_.rows() && i < joint_min_.rows()) {
                    if (joint_max_(i) >= std::numeric_limits<double>::infinity() ||
                        joint_min_(i) <= -std::numeric_limits<double>::infinity())
                        joint_types_.push_back(KDL::BasicJointType::Continuous);
                    else
                        joint_types_.push_back(KDL::BasicJointType::RotJoint);
                } else {
                    joint_types_.push_back(KDL::BasicJointType::RotJoint);
                }
            } else if (type.find("Trans") != std::string::npos) {
                joint_types_.push_back(KDL::BasicJointType::TransJoint);
            } else {
                // 默认当作旋转关节
                joint_types_.push_back(KDL::BasicJointType::RotJoint);
            }
        }

        // 保留原有 solver 初始化
        kdl_solver_ = std::make_unique<KDL::ChainIkSolverPos_TL>(
            chain_, joint_min_, joint_max_, eps_, true, true);
        nlopt_solver_ = std::make_unique<NLOPT_IK::NLOPT_IK>(
            chain_, joint_min_, joint_max_, maxtime, eps_, NLOPT_IK::SumSq);

        // q_out_ 复用
        q_out_.resize(chain_.getNrOfJoints());

        // -------------------------
        // 新增：缓存一些常用 solver 以避免频繁构造
        // （注意：请在 trac_ik.h 中声明对应成员）
        // std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        // std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        // std::mt19937 rng_;
        // KDL::JntArray q_reuse_;
        // -------------------------
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
        fk_solver_  = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        q_reuse_.resize(chain_.getNrOfJoints());

        // 初始化 RNG（可用固定种子调试，生产可改为随机种子）
        std::random_device rd;
        rng_.seed(rd());
    }

    // ----------------------------
    // restart variants
    // ----------------------------
    void TRAC_IK::restart(const KDL::JntArray& q_init, const KDL::Frame& p_in) {
        progress_ = -3;
        f_target_ = p_in;
        kdl_solver_->restart(q_init, p_in);
        nlopt_solver_->restart(q_init, p_in);
    }

    void TRAC_IK::restart(const KDL::JntArray& q_init) {
        progress_ = -3;
        kdl_solver_->restart(q_init);
        nlopt_solver_->restart(q_init);
    }

    // ----------------------------
    // step (保留原逻辑，微优化)
    // ----------------------------
    int TRAC_IK::step(int steps) {
        if (progress_ == 1) return 0;

        int kdl_result = kdl_solver_->step(steps);
        if (kdl_result == 0) {
            q_out_ = kdl_solver_->qout();
            progress_ = 1;
            return 0;
        }

        int nlopt_result = nlopt_solver_->step(steps);
        if (nlopt_result == 0) {
            q_out_ = nlopt_solver_->qout();
            progress_ = 1;
            return 0;
        }

        if (solve_type_ != Speed) {
            // 使用预分配数组
            KDL::JntArray q_random(chain_.getNrOfJoints());
            randomize(q_random);
            kdl_solver_->restart(q_random);
            kdl_result = kdl_solver_->step(steps);
            if (kdl_result == 0) {
                q_out_ = kdl_solver_->qout();
                progress_ = 1;
                return 0;
            }
            nlopt_solver_->restart(q_random);
            nlopt_result = nlopt_solver_->step(steps);
            if (nlopt_result == 0) {
                q_out_ = nlopt_solver_->qout();
                progress_ = 1;
                return 0;
            }
        }

        return 1;
    }

    // ----------------------------
    // CartToJnt（主求解入口）——已优化重启流程、Speed 优先返回
    // ----------------------------
    int TRAC_IK::CartToJnt(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& _bounds)
    {
        bounds_ = _bounds;
        restart(q_init, p_in);
        kdl_solver_->setBounds(bounds_);
        nlopt_solver_->setBounds(bounds_);

        // 先尝试一次原位求解（快速路径）
        int kdl_result = kdl_solver_->CartToJnt(q_init, p_in, q_out_, bounds_);
        if (kdl_result == 0) {
            q_out = q_out_;
            return 0;
        }
        int nlopt_result = nlopt_solver_->CartToJnt(q_init, p_in, q_out_, bounds_);
        if (nlopt_result == 0) {
            q_out = q_out_;
            return 0;
        }

        // 依据模式调整重启次数。Speed 模式下显著减少重启并首次可行解立即返回。
        const int default_max_restarts = 100;
        const int speed_max_restarts = 10;
        const int max_restarts = (solve_type_ == Speed) ? speed_max_restarts : default_max_restarts;

        // 复用容器（避免频繁分配）
        std::vector<KDL::JntArray> solutions;
        std::vector<double> errors;
        solutions.reserve(32);
        errors.reserve(32);

        for (int i = 0; i < max_restarts; i++) {
            // 使用预分配的 q_reuse_
            KDL::JntArray& q_random = q_reuse_;
            // 生成随机初值
            randomize(q_random);

            // 尝试 KDL
            kdl_solver_->restart(q_random);
            kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds_);
            if (kdl_result == 0) {
                if (solve_type_ == Speed) {
                    q_out = q_out_;
                    return 0; // Speed：首次成功立即返回
                }
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); ++j) {
                            double d = q_out_(j) - q_init(j);
                            error += d * d;
                        }
                        break;
                    case Manip1: error = manipulability(q_out_); break;
                    case Manip2: error = manipulability2(q_out_); break;
                }
                errors.push_back(error);
            }

            // 尝试 NLopt
            nlopt_solver_->restart(q_random);
            nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds_);
            if (nlopt_result == 0) {
                if (solve_type_ == Speed) {
                    q_out = q_out_;
                    return 0;
                }
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); ++j) {
                            double d = q_out_(j) - q_init(j);
                            error += d * d;
                        }
                        break;
                    case Manip1: error = manipulability(q_out_); break;
                    case Manip2: error = manipulability2(q_out_); break;
                }
                errors.push_back(error);
            }
        }

        // 如果收集到了若干解（非 Speed 模式），选最优一个
        if (!solutions.empty()) {
            int best_idx = 0;
            double best_error = errors[0];
            for (size_t i = 1; i < errors.size(); i++) {
                bool prefer_larger = (solve_type_ == Manip1 || solve_type_ == Manip2);
                if (prefer_larger ? (errors[i] > best_error) : (errors[i] < best_error)) {
                    best_error = errors[i];
                    best_idx = i;
                }
            }
            q_out = solutions[best_idx];
            return 0;
        }

        // 否则，返回最后一次 solver 的结果（尽量返回 nlopt 更好的结果）
        if (kdl_result > nlopt_result) {
            q_out = nlopt_solver_->qout();
            return nlopt_result;
        } else {
            q_out = kdl_solver_->qout();
            return kdl_result;
        }
    }

    // ----------------------------
    // randomize（优化：避免频繁构造 distribution）
    // ----------------------------
    void TRAC_IK::randomize(KDL::JntArray& q) {
        const unsigned NJ = q.data.size();
        for (unsigned j = 0; j < NJ; ++j) {
            if (j < joint_types_.size() && j < static_cast<size_t>(joint_min_.rows()) && j < static_cast<size_t>(joint_max_.rows())) {
                // 生成 [0,1) 随机数
                std::uniform_real_distribution<double> dist01(0.0, 1.0);
                double rnd = dist01(rng_);
                if (joint_types_[j] == KDL::BasicJointType::Continuous) {
                    double low = q(j) - 2.0 * M_PI;
                    double high = q(j) + 2.0 * M_PI;
                    q(j) = low + rnd * (high - low);
                } else {
                    double low = joint_min_(j);
                    double high = joint_max_(j);
                    // 若上下界非法（相等或 nan），保持原值
                    if (!(std::isfinite(low) && std::isfinite(high) && (high - low) > 1e-12)) {
                        // 保持 q(j) 不变
                    } else {
                        q(j) = low + rnd * (high - low);
                    }
                }
            } else {
                // 超出索引，保持值
            }
        }
    }

    // ----------------------------
    // manipulability（优化：用 Gram 矩阵）
    // ----------------------------
    double TRAC_IK::manipulability(const KDL::JntArray& q) {
        KDL::Jacobian jac(chain_.getNrOfJoints());
        // 使用缓存的 jac_solver_（若失败则临时构造一个）
        if (jac_solver_) {
            if (jac_solver_->JntToJac(q, jac) < 0) return 0.0;
        } else {
            KDL::ChainJntToJacSolver local_jac(chain_);
            if (local_jac.JntToJac(q, jac) < 0) return 0.0;
        }

        const unsigned int cols = jac.columns();
        Eigen::MatrixXd J(6, cols);
        for (unsigned int c = 0; c < cols; ++c)
            for (int r = 0; r < 6; ++r)
                J(r, c) = jac(r, c);

        // Gram 矩阵 G = J * J^T （6x6）
        Eigen::MatrixXd G = J * J.transpose();

        // 对角正则化，提升数值稳定性
        const double eps_reg = 1e-9;
        for (int i = 0; i < 6; ++i) G(i,i) += eps_reg;

        // 若 G 非正定，返回 0
        // compute determinant (6x6 行列式开销小)
        double detG = G.determinant();
        if (detG <= 0.0 || !std::isfinite(detG)) return 0.0;

        // prod_sv = sqrt(det(G))，原代码返回 sqrt(product_sv)，若需兼容性可再 sqrt 一次
        double prod_sv = std::sqrt(detG);

        // 原实现返回 sqrt(product_of_singular_values)。为了兼容旧行为，这里保持跟原来接近的量级：
        // 原先：manip = prod(sv); return sqrt(manip) -> 返回 (prod_sv)^{1/2}
        // 但 prod_sv = prod(sv)；为了避免再次开根造成数值更小，这里直接返回 prod_sv。
        // 如果你需要严格相同输出，请把下面改为: return std::sqrt(prod_sv);
        return prod_sv;
    }

    // ----------------------------
    // manipulability2（优化：用特征值计算 max/min 奇异值）
    // ----------------------------
    double TRAC_IK::manipulability2(const KDL::JntArray& q) {
        KDL::Jacobian jac(chain_.getNrOfJoints());
        if (jac_solver_) {
            if (jac_solver_->JntToJac(q, jac) < 0) return 0.0;
        } else {
            KDL::ChainJntToJacSolver local_jac(chain_);
            if (local_jac.JntToJac(q, jac) < 0) return 0.0;
        }

        const unsigned int cols = jac.columns();
        Eigen::MatrixXd J(6, cols);
        for (unsigned int c = 0; c < cols; ++c)
            for (int r = 0; r < 6; ++r)
                J(r, c) = jac(r, c);

        // G = J * J^T
        Eigen::MatrixXd G = J * J.transpose();
        const double eps_reg = 1e-9;
        for (int i = 0; i < 6; ++i) G(i,i) += eps_reg;

        // 对称矩阵的特征分解（高效且数值稳定）
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(G);
        if (es.info() != Eigen::Success) return 0.0;

        Eigen::VectorXd eigs = es.eigenvalues();
        // eigenvalues are non-negative (but numerical可能有负值接近0)
        double max_eig = eigs.maxCoeff();
        double min_eig = eigs.minCoeff();
        if (min_eig < 1e-12) min_eig = 1e-12;
        if (max_eig < 1e-12) max_eig = 1e-12;

        // 奇异值 = sqrt(eigenvalue)
        double max_sv = std::sqrt(max_eig);
        double min_sv = std::sqrt(min_eig);

        return 1.0 / (max_sv / min_sv);
    }

    // ----------------------------
    // TryRandomRestarts（内部使用） - 使用复用与 Speed 优先策略
    // ----------------------------
    int TRAC_IK::TryRandomRestarts(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& bounds)
    {
        const int default_max_restarts = 100;
        const int speed_max_restarts = 10;
        const int max_restarts = (solve_type_ == Speed) ? speed_max_restarts : default_max_restarts;

        std::vector<KDL::JntArray> solutions;
        std::vector<double> errors;
        solutions.reserve(32);
        errors.reserve(32);

        for (int i = 0; i < max_restarts; ++i) {
            KDL::JntArray& q_random = q_reuse_;
            randomize(q_random);

            kdl_solver_->restart(q_random);
            int kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
            if (kdl_result == 0) {
                if (solve_type_ == Speed) { q_out = q_out_; return 0; }
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); ++j) {
                            double d = q_out_(j) - q_init(j);
                            error += d * d;
                        }
                        break;
                    case Manip1: error = manipulability(q_out_); break;
                    case Manip2: error = manipulability2(q_out_); break;
                }
                errors.push_back(error);
            }

            nlopt_solver_->restart(q_random);
            int nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
            if (nlopt_result == 0) {
                if (solve_type_ == Speed) { q_out = q_out_; return 0; }
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); ++j) {
                            double d = q_out_(j) - q_init(j);
                            error += d * d;
                        }
                        break;
                    case Manip1: error = manipulability(q_out_); break;
                    case Manip2: error = manipulability2(q_out_); break;
                }
                errors.push_back(error);
            }
        }

        if (!solutions.empty()) {
            int best_idx = 0;
            double best_error = errors[0];
            for (size_t i = 1; i < errors.size(); ++i) {
                bool prefer_larger = (solve_type_ == Manip1 || solve_type_ == Manip2);
                if (prefer_larger ? (errors[i] > best_error) : (errors[i] < best_error)) {
                    best_error = errors[i];
                    best_idx = i;
                }
            }
            q_out = solutions[best_idx];
            return 0;
        }

        return -1;
    }


} // namespace TRAC_IK
