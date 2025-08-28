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

        for (size_t i = 0; i < chain_.segments.size(); i++) {
            std::string type = chain_.segments[i].getJoint().getTypeName();
            if (type.find("Rot") != std::string::npos) {
                if (joint_max_(i) >= std::numeric_limits<double>::infinity() ||
                    joint_min_(i) <= -std::numeric_limits<double>::infinity())
                    joint_types_.push_back(KDL::BasicJointType::Continuous);
                else
                    joint_types_.push_back(KDL::BasicJointType::RotJoint);
            } else if (type.find("Trans") != std::string::npos) {
                joint_types_.push_back(KDL::BasicJointType::TransJoint);
            }
        }

        kdl_solver_ = std::make_unique<KDL::ChainIkSolverPos_TL>(
            chain_, joint_min_, joint_max_, eps_, true, true);
        nlopt_solver_ = std::make_unique<NLOPT_IK::NLOPT_IK>(
            chain_, joint_min_, joint_max_, maxtime, eps_, NLOPT_IK::SumSq);

        q_out_.resize(chain_.getNrOfJoints());
    }

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

        const int max_restarts = 100;
        std::vector<KDL::JntArray> solutions;
        std::vector<double> errors;

        for (int i = 0; i < max_restarts; i++) {
            KDL::JntArray q_random(chain_.getNrOfJoints());
            randomize(q_random);

            kdl_solver_->restart(q_random);
            kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds_);
            if (kdl_result == 0) {
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); j++)
                            error += pow(q_out_(j) - q_init(j), 2);
                        break;
                    case Manip1: error = manipulability(q_out_); break;
                    case Manip2: error = manipulability2(q_out_); break;
                }
                errors.push_back(error);
            }

            nlopt_solver_->restart(q_random);
            nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds_);
            if (nlopt_result == 0) {
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); j++)
                            error += pow(q_out_(j) - q_init(j), 2);
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
            for (size_t i = 1; i < errors.size(); i++) {
                if ((solve_type_ == Manip1 || solve_type_ == Manip2) ? (errors[i] > best_error) : (errors[i] < best_error)) {
                    best_error = errors[i];
                    best_idx = i;
                }
            }
            q_out = solutions[best_idx];
            return 0;
        }

        if (kdl_result > nlopt_result) {
            q_out = nlopt_solver_->qout();
            return nlopt_result;
        } else {
            q_out = kdl_solver_->qout();
            return kdl_result;
        }
    }

    void TRAC_IK::randomize(KDL::JntArray& q) {
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

    double TRAC_IK::manipulability(const KDL::JntArray& q) {
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
        return std::sqrt(manip);
    }

    double TRAC_IK::manipulability2(const KDL::JntArray& q) {
        KDL::Jacobian jac(chain_.getNrOfJoints());
        KDL::ChainJntToJacSolver jac_solver(chain_);
        if (jac_solver.JntToJac(q, jac) < 0) return 0.0;
        Eigen::MatrixXd J(6, jac.columns());
        for (unsigned int col = 0; col < jac.columns(); ++col)
            for (int row = 0; row < 6; ++row)
                J(row, col) = jac(row, col);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd sv = svd.singularValues();
        if (sv.size() == 0) return 0.0;
        double max_sv = sv(0);
        double min_sv = sv(sv.size() - 1);
        if (min_sv < 1e-10) min_sv = 1e-10;
        return 1.0 / (max_sv / min_sv);
    }

    // ======================
    // ✅ 新增函数：随机重启（内部使用）
    // ======================
    int TRAC_IK::TryRandomRestarts(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& bounds)
    {
        const int max_restarts = 100;
        std::vector<KDL::JntArray> solutions;
        std::vector<double> errors;

        for (int i = 0; i < max_restarts; i++) {
            KDL::JntArray q_random(chain_.getNrOfJoints());
            randomize(q_random);

            kdl_solver_->restart(q_random);
            int kdl_result = kdl_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
            if (kdl_result == 0) {
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); j++)
                            error += pow(q_out_(j) - q_init(j), 2);
                        break;
                    case Manip1: error = manipulability(q_out_); break;
                    case Manip2: error = manipulability2(q_out_); break;
                }
                errors.push_back(error);
            }

            nlopt_solver_->restart(q_random);
            int nlopt_result = nlopt_solver_->CartToJnt(q_random, p_in, q_out_, bounds);
            if (nlopt_result == 0) {
                solutions.push_back(q_out_);
                double error = 0.0;
                switch (solve_type_) {
                    case Speed: error = 1.0; break;
                    case Distance:
                        for (unsigned int j = 0; j < q_init.rows(); j++)
                            error += pow(q_out_(j) - q_init(j), 2);
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
            for (size_t i = 1; i < errors.size(); i++) {
                if ((solve_type_ == Manip1 || solve_type_ == Manip2) ? (errors[i] > best_error) : (errors[i] < best_error)) {
                    best_error = errors[i];
                    best_idx = i;
                }
            }
            q_out = solutions[best_idx];
            return 0;
        }

        return -1;
    }

    // ======================
    // ✅ 最终优化版：并行求解（带误差验证 + 回退随机重启）
    // ======================
    int TRAC_IK::CartToJntParallel(
        const KDL::JntArray& q_init,
        const KDL::Frame& p_in,
        KDL::JntArray& q_out,
        const KDL::Twist& bounds,
        double timeout_ms)
    {
        using namespace std::chrono;

        const double MAX_POS_ERR = 0.01;     // 1 cm
        const double MAX_ORI_ERR = 0.1;      // ~5.7°
        const int    NJ = q_init.rows();

        int best_result = -1;
        KDL::JntArray best_q_out(NJ);
        double best_pos_err = std::numeric_limits<double>::max();
        double best_ori_err = std::numeric_limits<double>::max();

        auto future_kdl = std::async(std::launch::async, [&]() {
            KDL::JntArray q_kdl(NJ);
            int res = kdl_solver_->CartToJnt(q_init, p_in, q_kdl, bounds);
            return std::make_pair(res, q_kdl);
        });

        auto future_nlopt = std::async(std::launch::async, [&]() {
            KDL::JntArray q_nlopt(NJ);
            int res = nlopt_solver_->CartToJnt(q_init, p_in, q_nlopt, bounds);
            return std::make_pair(res, q_nlopt);
        });

        auto start = high_resolution_clock::now();

        std::pair<int, KDL::JntArray> result_kdl, result_nlopt;
        bool kdl_valid = false, nlopt_valid = false;

        if (future_kdl.valid()) {
            if (future_kdl.wait_for(milliseconds(static_cast<int>(timeout_ms))) == std::future_status::ready) {
                result_kdl = future_kdl.get();
                kdl_valid = true;
                if (result_kdl.first == 0) {
                    KDL::Frame computed_pose;
                    KDL::ChainFkSolverPos_recursive fk_solver(chain_);
                    if (fk_solver.JntToCart(result_kdl.second, computed_pose) >= 0) {
                        KDL::Twist err = KDL::diff(p_in, computed_pose);
                        double pos_err = err.vel.Norm();
                        double ori_err = err.rot.Norm();
                        if (pos_err < MAX_POS_ERR && ori_err < MAX_ORI_ERR) {
                            bool valid = true;
                            for (int i = 0; i < NJ; ++i) {
                                if (result_kdl.second(i) < joint_min_(i) || result_kdl.second(i) > joint_max_(i)) {
                                    valid = false;
                                    break;
                                }
                            }
                            if (valid) {
                                // std::cout << "[KDL] return valid solution" << std::endl;
                                q_out = result_kdl.second;
                                return result_kdl.first;
                            }
                        }
                    }
                }
            }
        }

        if (future_nlopt.valid()) {
            if (future_nlopt.wait_for(milliseconds(static_cast<int>(timeout_ms))) == std::future_status::ready) {
                result_nlopt = future_nlopt.get();
                nlopt_valid = true;
                if (result_nlopt.first == 0) {
                    KDL::Frame computed_pose;
                    KDL::ChainFkSolverPos_recursive fk_solver(chain_);
                    if (fk_solver.JntToCart(result_nlopt.second, computed_pose) >= 0) {
                        KDL::Twist err = KDL::diff(p_in, computed_pose);
                        double pos_err = err.vel.Norm();
                        double ori_err = err.rot.Norm();
                        if (pos_err < MAX_POS_ERR && ori_err < MAX_ORI_ERR) {
                            bool valid = true;
                            for (int i = 0; i < NJ; ++i) {
                                if (result_nlopt.second(i) < joint_min_(i) || result_nlopt.second(i) > joint_max_(i)) {
                                    valid = false;
                                    break;
                                }
                            }
                            if (valid) {
                                // std::cout << "[NLOPT] return valid solution" << std::endl;
                                q_out = result_nlopt.second;
                                return result_nlopt.first;
                            }
                        }
                    }
                }
            }
        }

        // std::cout << "no valid soluntion, random restart" << std::endl;
        int random_result = TryRandomRestarts(q_init, p_in, q_out, bounds);
        if (random_result == 0) {
            // std::cout << "random start success" << std::endl;
            return 0;
        } else {
            // std::cout << "no valid solution" << std::endl;
            return -1;
        }
    }

} // namespace TRAC_IK