#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

#include <vector>
#include <string>
#include <random>
#include <nlopt.h>   // 先包含 C 接口（有时需要，视 nlopt.hpp 实现而定）
#include <nlopt.hpp> // 然后包含 C++ 封装
#include <memory>

#include "kdl_types.h"
#include "chainfksolverpos_recursive.h"
#include "dual_quaternion.h"

namespace NLOPT_IK {

    enum OptType {
        Joint,         // Minimize distance to joint configuration
        DualQuat,      // Minimize dual quaternion error
        SumSq,         // Minimize sum of squared errors
        L2             // Minimize L2 norm of error
    };

    /// A nonlinear optimization based inverse kinematics solver.
    class NLOPT_IK {
    public:
        NLOPT_IK(
            const KDL::Chain& chain,
            const KDL::JntArray& q_min,
            const KDL::JntArray& q_max,
            double maxtime = 0.005,
            double eps = 1e-3,
            OptType type = SumSq);

        /// \name Configuration
        ///@{
        void setBounds(const KDL::Twist& bounds) { bounds_ = bounds; }
        auto bounds() const -> const KDL::Twist& { return bounds_; }

        void setEps(double eps) { eps_ = eps; }
        double eps() const { return eps_; }

        void setMaxTime(double maxtime) { maxtime_ = maxtime; }
        double maxTime() const { return maxtime_; }

        void setOptType(OptType type) { opt_type_ = type; }
        OptType optType() const { return opt_type_; }
        ///@}

        /// \name Iterative Cart-to-Joint Interface
        ///@{

        /// Reset the current position and target frame of the solver.
        void restart(const KDL::JntArray& q_init, const KDL::Frame& p_in);

        /// Reset the current position, but NOT the target frame, of the solver.
        void restart(const KDL::JntArray& q_init);

        /// Step through a single iteration of the solver.
        ///
        /// Returns 0 if the solver has already converged to a solution and a
        /// non-zero value otherwise.
        int step(int steps = 1);

        ///@}

        /// Return the current configuration in the solver; the solution
        /// configuration if the solver has converged to a solution.
        const KDL::JntArray& qout() const { return q_out_; }

        int CartToJnt(
            const KDL::JntArray& q_init,
            const KDL::Frame& p_in,
            KDL::JntArray& q_out,
            const KDL::Twist& bounds = KDL::Twist::Zero());

    private:
        std::unique_ptr<nlopt::opt> nlopt_;

        const KDL::Chain& chain_;
        KDL::JntArray joint_min_, joint_max_;
        std::vector<KDL::BasicJointType> joint_types_;
        KDL::ChainFkSolverPos_recursive fk_solver_;
        KDL::JntArray q_tmp_;

        double maxtime_;
        double eps_;
        OptType opt_type_;

        std::vector<double> x_min_, x_max_, best_x_;
        KDL::JntArray q_out_;
        KDL::Frame f_target_;
        KDL::Twist bounds_;
        int progress_;

        // Optimization variables
        std::vector<double> target_pos_;
        dual_quaternion target_dq_;

        // Optimization functions
        double minJoints(const std::vector<double>& x, std::vector<double>& grad);
        void cartSumSquaredError(const std::vector<double>& x, double error[]);
        void cartL2NormError(const std::vector<double>& x, double error[]);
        void cartDQError(const std::vector<double>& x, double error[]);

        // Simplified optimization algorithm
        bool optimize(std::vector<double>& x, double& minf);

        // Helper functions
        void clipToJointLimits(std::vector<double>& x);
    };

} // namespace NLOPT_IK

#endif