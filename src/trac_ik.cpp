#include "trac_ik.h"

#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>
#include <future>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace TRAC_IK {

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

        // Initialize joint types
        for (size_t i = 0; i < chain_.segments.size(); i++) {
            std::string type = chain_.segments[i].getJoint().getTypeName();
            if (type.find("Rot") != std::string::npos) {
                if (joint_max_(i) >= std::numeric_limits<double>::infinity() ||
                    joint_min_(i) <= -std::numeric_limits<double>::infinity())
                {
                    joint_types_.push_back(KDL::BasicJointType::Continuous);
                } else {
                    joint_types_.push_back(KDL::BasicJointType::RotJoint);
                }
            } else if (type.find("Trans") != std::string::npos) {
                joint_types_.push_back(KDL::BasicJointType::TransJoint);
            }
        }

        // Initialize solvers
        kdl_solver_ = std::make_unique<KDL::ChainIkSolverPos_TL>(
            chain_, joint_min_, joint_max_, eps_, true, true);
            
        nlopt_solver_ = std::make_unique<NLOPT_IK::NLOPT_IK>(
            chain_, joint_min_, joint_max_, maxtime, eps_, NLOPT_IK::SumSq);

        // Initialize output joint array
        q_out_.resize(chain_.getNrOfJoints());
    }

    void TRAC_IK::restart(const KDL::JntArray& q_init, const KDL::Frame& p_in)
    {
        // Reset optimization state
        progress_ = -3;

        // Set target frame
        f_target_ = p_in;

        // Reset both solvers
        kdl_solver_->restart(q_init, p_in);
        nlopt_solver_->restart(q_init, p_in);
    }

    void TRAC_IK::restart(const KDL::JntArray& q_init)
    {
        // Reset optimization state but keep target frame
        progress_ = -3;

        // Reset both solvers
        kdl_solver_->restart(q_init);
        nlopt_solver_->restart(q_init);
    }

    int TRAC_IK::step(int steps)
    {
        if (progress_ == 1) {
            return 0;
        }

        // Try KDL solver first
        int kdl_result = kdl_solver_->step(steps);
        if (kdl_result == 0) {
            q_out_ = kdl_solver_->qout();
            progress_ = 1;
            return 0;
        }

        // If KDL fails, try NLOPT solver
        int nlopt_result = nlopt_solver_->step(steps);
        if (nlopt_result == 0) {
            q_out_ = nlopt_solver_->qout();
            progress_ = 1;
            return 0;
        }

        // If both fail, try random restarts
        if (solve_type_ != Speed) {
            KDL::JntArray q_random(chain_.getNrOfJoints());
            randomize(q_random);
            
            // Try KDL with random restart
            kdl_solver_->restart(q_random);
            kdl_result = kdl_solver_->step(steps);
            if (kdl_result == 0) {
                q_out_ = kdl_solver_->qout();
                progress_ = 1;
                return 0;
            }
            
            // Try NLOPT with random restart
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

    // const KDL::JntArray& TRAC_IK::qout() const
    // {
    //     return q_out_;
    // }

    int TRAC_IK::CartToJnt(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& _bounds)
    {
        bounds_ = _bounds;
        restart(q_init, p_in);

        // Set bounds for both solvers
        kdl_solver_->setBounds(bounds_);
        nlopt_solver_->setBounds(bounds_);

        // Try KDL solver first
        int kdl_result = kdl_solver_->CartToJnt(q_init, p_in, q_out_, bounds_);
        if (kdl_result == 0) {
            q_out = q_out_;
            return 0;
        }

        // If KDL fails, try NLOPT solver
        int nlopt_result = nlopt_solver_->CartToJnt(q_init, p_in, q_out_, bounds_);
        if (nlopt_result == 0) {
            q_out = q_out_;
            return 0;
        }

        // If both fail, try multiple random restarts
        const int max_restarts = 100;
        std::vector<KDL::JntArray> solutions;
        std::vector<double> errors;

        for (int i = 0; i < max_restarts; i++) {
            KDL::JntArray q_random(chain_.getNrOfJoints());
            randomize(q_random);

            // Try KDL with random restart
            kdl_solver_->restart(q_random);
            kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds_);
            if (kdl_result == 0) {
                solutions.push_back(q_out_);
                
                // Calculate error based on solve type
                double error = 0.0;
                switch (solve_type_) {
                    case Speed:
                        error = 1.0; // All solutions are equally good
                        break;
                    case Distance:
                        // Distance from seed
                        for (unsigned int j = 0; j < q_init.rows(); j++) {
                            error += pow(q_out_(j) - q_init(j), 2);
                        }
                        break;
                    case Manip1:
                        error = manipulability(q_out_);
                        break;
                    case Manip2:
                        error = manipulability2(q_out_);
                        break;
                }
                errors.push_back(error);
            }

            // Try NLOPT with random restart
            nlopt_solver_->restart(q_random);
            nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds_);
            if (nlopt_result == 0) {
                solutions.push_back(q_out_);
                
                // Calculate error based on solve type
                double error = 0.0;
                switch (solve_type_) {
                    case Speed:
                        error = 1.0; // All solutions are equally good
                        break;
                    case Distance:
                        // Distance from seed
                        for (unsigned int j = 0; j < q_init.rows(); j++) {
                            error += pow(q_out_(j) - q_init(j), 2);
                        }
                        break;
                    case Manip1:
                        error = manipulability(q_out_);
                        break;
                    case Manip2:
                        error = manipulability2(q_out_);
                        break;
                }
                errors.push_back(error);
            }
        }

        // If we found solutions, select the best one based on solve type
        if (!solutions.empty()) {
            int best_idx = 0;
            double best_error = errors[0];
            
            for (size_t i = 1; i < errors.size(); i++) {
                // For manipulability, higher is better
                if (solve_type_ == Manip1 || solve_type_ == Manip2) {
                    if (errors[i] > best_error) {
                        best_error = errors[i];
                        best_idx = i;
                    }
                } else {
                    // For distance and speed, lower is better
                    if (errors[i] < best_error) {
                        best_error = errors[i];
                        best_idx = i;
                    }
                }
            }
            
            q_out = solutions[best_idx];
            return 0;
        }

        // If all attempts fail, return the best solution from KDL or NLOPT
        if (kdl_result > nlopt_result) {
            q_out = nlopt_solver_->qout();
            return nlopt_result;
        } else {
            q_out = kdl_solver_->qout();
            return kdl_result;
        }
    }

    void TRAC_IK::randomize(KDL::JntArray& q)
    {
        for (size_t j = 0; j < q.data.size(); ++j) {
            if (joint_types_[j] == KDL::BasicJointType::Continuous) {
                std::uniform_real_distribution<double> dist(
                        q(j) - 2.0 * M_PI, q(j) + 2.0 * M_PI);
                q(j) = dist(rng_);
            } else {
                std::uniform_real_distribution<double> dist(
                        joint_min_(j), joint_max_(j));
                q(j) = dist(rng_);
            }
        }
    }

    double TRAC_IK::manipulability(const KDL::JntArray& q)
    {
        KDL::Jacobian jac(chain_.getNrOfJoints());
        KDL::ChainJntToJacSolver jac_solver(chain_);
        if (jac_solver.JntToJac(q, jac) < 0) return 0.0;

        Eigen::MatrixXd J(6, jac.columns());
        for (unsigned int col = 0; col < jac.columns(); ++col)
            for (int row = 0; row < 6; ++row)
                J(row, col) = jac(row, col);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd sv = svd.singularValues();

        double manip = 1.0;
        for (int i = 0; i < sv.size(); ++i)
            manip *= sv(i);
        return std::sqrt(manip);  // 或者直接 return manip; （奇异值乘积）
    }

    double TRAC_IK::manipulability2(const KDL::JntArray& q)
    {
        // Compute Jacobian using KDL
        KDL::Jacobian jac(chain_.getNrOfJoints());
        KDL::ChainJntToJacSolver jac_solver(chain_);
        if (jac_solver.JntToJac(q, jac) < 0) return 0.0;
    
        // Convert to Eigen
        Eigen::MatrixXd J(6, jac.columns());
        for (unsigned int col = 0; col < jac.columns(); ++col)
            for (int row = 0; row < 6; ++row)
                J(row, col) = jac(row, col);

        // Perform SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd sv = svd.singularValues();

        if (sv.size() == 0) return 0.0;

        double max_sv = sv(0);             // 最大奇异值
        double min_sv = sv(sv.size() - 1); // 最小奇异值

        if (min_sv < 1e-10) min_sv = 1e-10; // 避免除以 0

        // 条件数 = λ_max / λ_min
        double cond = max_sv / min_sv;

        // 可选：返回条件数的倒数，作为可操作性的一种度量
        return 1.0 / cond;
    }

    int TRAC_IK::CartToJntParallel(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& bounds,
        double timeout_ms)
    {
        using namespace std::chrono;

        // 1. 初始化结果变量
        int best_result = -1;
        KDL::JntArray best_q_out;
        double best_error = std::numeric_limits<double>::max();

        // 2. 启动异步任务：KDL 求解器
        auto future_kdl = std::async(std::launch::async, [&]() -> std::pair<int, KDL::JntArray> {
            KDL::JntArray q_kdl(q_init.rows());
            int res = kdl_solver_->CartToJnt(q_init, p_in, q_kdl, bounds);
            if (res == 0) {
                return {res, q_kdl};
            } else {
                return {res, KDL::JntArray(q_init.rows())};  // 返回无效解
            }
        });

        // 3. 启动异步任务：NLOPT 求解器
        auto future_nlopt = std::async(std::launch::async, [&]() -> std::pair<int, KDL::JntArray> {
            KDL::JntArray q_nlopt(q_init.rows());
            int res = nlopt_solver_->CartToJnt(q_init, p_in, q_nlopt, bounds);
            if (res == 0) {
                return {res, q_nlopt};
            } else {
                return {res, KDL::JntArray(q_init.rows())};  // 返回无效解
            }
        });

        // 4. 设定超时时间
        auto start = high_resolution_clock::now();
        bool kdl_ready = false, nlopt_ready = false;
        std::pair<int, KDL::JntArray> result_kdl, result_nlopt;

        // 5. 尝试获取 KDL 结果（带超时逻辑）
        if (future_kdl.valid()) {
            if (future_kdl.wait_for(milliseconds(static_cast<int>(timeout_ms))) == std::future_status::ready) {
                result_kdl = future_kdl.get();
                kdl_ready = true;
                if (result_kdl.first == 0) {
                    // 可以在这里计算误差，暂时假设 KDL 返回的就是可接受的
                    best_result = result_kdl.first;
                    best_q_out = result_kdl.second;
                    return best_result;  // 直接返回 KDL 的解
                }
            }
        }

        // 6. 尝试获取 NLOPT 结果（带超时逻辑）
        if (future_nlopt.valid()) {
            if (future_nlopt.wait_for(milliseconds(static_cast<int>(timeout_ms))) == std::future_status::ready) {
                result_nlopt = future_nlopt.get();
                nlopt_ready = true;
                if (result_nlopt.first == 0) {
                    best_result = result_nlopt.first;
                    best_q_out = result_nlopt.second;
                    return best_result;  // 直接返回 NLOPT 的解
                }
            }
        }

        // 7. 如果两者都返回了，但都失败，则你可以选择：
        //    - 继续走原来的随机重启逻辑
        //    - 或者直接返回失败

        // 8. （可选）如果你想进一步比较 KDL / NLOPT 的结果（比如谁快谁慢，谁误差小），可以在这做
        //    但目前我们只返回最先成功的那个

        // 9. 如果到这里还没有返回，说明两者都未成功，你可以选择：
        return -1;  // 表示都失败了，你也可以调用原来的随机重启逻辑
    }

} // namespace TRAC_IK