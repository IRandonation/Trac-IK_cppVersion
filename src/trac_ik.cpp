#include "trac_ik.h"

#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>
#include <random>
#include <omp.h>

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
        dist01_ = std::uniform_real_distribution<double>(0.0, 1.0);
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
            std::cout << "KDL求解成功，迭代次数: " << kdl_solver_->getIterationCount() << std::endl;
            return 0;
        }

        int nlopt_result = nlopt_solver_->step(steps);
        if (nlopt_result == 0) {
            q_out_ = nlopt_solver_->qout();
            progress_ = 1;
            return 0;
        }

        if (solve_type_ != Speed) {
            // 使用预分配数组 - 已经优化
            KDL::JntArray& q_random = q_reuse_;  // 使用复用数组
            randomize(q_random);
            kdl_solver_->restart(q_random);
            kdl_result = kdl_solver_->step(steps);
            if (kdl_result == 0) {
                q_out_ = kdl_solver_->qout();
                progress_ = 1;
                std::cout << "KDL求解成功，迭代次数: " << kdl_solver_->getIterationCount() << std::endl;
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
            std::cout << "KDL求解成功，迭代次数: " << kdl_solver_->getIterationCount() << std::endl;
            return 0;
        }
        int nlopt_result = nlopt_solver_->CartToJnt(q_init, p_in, q_out_, bounds_);
        if (nlopt_result == 0) {
            q_out = q_out_;
            return 0;
        }

        return TryRandomRestarts(q_init, p_in, q_out, bounds_);
    }

    // ----------------------------
    // randomize（优化：避免频繁构造 distribution）
    // ----------------------------
    void TRAC_IK::randomize(KDL::JntArray& q) {
        // 批量生成随机数，减少函数调用开销
        static std::vector<double> random_cache(32);
        if (random_cache.size() < q.data.size()) {
            random_cache.resize(q.data.size());
        }
        
        for (size_t i = 0; i < q.data.size(); ++i) {
            random_cache[i] = dist01_(rng_);
        }
        
        // 使用缓存的随机数
        const unsigned NJ = q.data.size();
        for (unsigned j = 0; j < NJ; ++j) {
            if (j < joint_types_.size() && j < static_cast<size_t>(joint_min_.rows()) && j < static_cast<size_t>(joint_max_.rows())) {
                // 使用预生成的随机数
                double rnd = random_cache[j];
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
        // 时间预算开始
        const double time_limit = std::max(0.0, maxtime_);
        auto t0 = std::chrono::steady_clock::now();
    
        // Speed 模式：先 KDL-only，后 NLopt 兜底
        if (solve_type_ == Speed) {
            const int kdl_only_trials = 15;   // 可根据需要调整
            const int nlopt_trials    = 2;    // 极少数兜底尝试
    
            // 并行尝试多个随机初始化
            bool solution_found = false;
            KDL::JntArray best_solution(chain_.getNrOfJoints());
            
            #pragma omp parallel for num_threads(4)
            for (int i = 0; i < kdl_only_trials; ++i) {
                if (solution_found) continue;
                
                if (time_limit > 0.0) {
                    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                    if (elapsed >= time_limit) continue;
                }
    
                KDL::JntArray& q_random = q_reuse_;
                randomize(q_random);
    
                kdl_solver_->restart(q_random);
                int kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
                if (kdl_result == 0) { q_out = q_out_; return 0; }
            }
    
            // NLopt 兜底（极少数次）
            for (int i = 0; i < nlopt_trials; ++i) {
                if (time_limit > 0.0) {
                    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                    if (elapsed >= time_limit) return -1;
                }
    
                KDL::JntArray& q_random = q_reuse_;
                randomize(q_random);
    
                nlopt_solver_->restart(q_random);
                int nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
                if (nlopt_result == 0) { q_out = q_out_; return 0; }
            }
    
            return -1;
        }
    
        // 非 Speed 模式：保留原有收集最优解逻辑，加入时间预算检查
    
        const int default_max_restarts = 100;
        const int speed_max_restarts = 10; // 未使用（仅用于兼容变量存在）
        const int max_restarts = (solve_type_ == Speed) ? speed_max_restarts : default_max_restarts;
    
        std::vector<KDL::JntArray> solutions;
        std::vector<double> errors;
        solutions.reserve(max_restarts);
        errors.reserve(max_restarts);
    
        for (int i = 0; i < max_restarts; ++i) {
            if (time_limit > 0.0) {
                double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                if (elapsed >= time_limit) break;
            }
    
            KDL::JntArray& q_random = q_reuse_;
            randomize(q_random);
    
            kdl_solver_->restart(q_random);
            int kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
            if (kdl_result == 0) {
                // 非 Speed 模式：记录候选解
                std::cout << "KDL求解成功，迭代次数: " << kdl_solver_->getIterationCount() << std::endl;
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
                
                // Speed模式下找到第一个解就立即返回
                if (solve_type_ == Speed && solutions.size() >= 1) {
                    q_out = solutions[0];
                    return 0;
                }
            }
    
            if (time_limit > 0.0) {
                double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                if (elapsed >= time_limit) break;
            }
    
            nlopt_solver_->restart(q_random);
            int nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
            if (nlopt_result == 0) {
                // 非 Speed 模式：记录候选解
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
                
                // Speed模式下找到第一个解就立即返回
                if (solve_type_ == Speed && solutions.size() >= 1) {
                    q_out = solutions[0];
                    return 0;
                }
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
