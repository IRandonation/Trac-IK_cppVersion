#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

#include <vector>
#include <string>
#include <memory>
#include <random>

#include "kdl_types.h"
#include "kdl_tl.h"
#include "nlopt_ik.h"

namespace TRAC_IK {

    /// A hybrid inverse kinematics solver that combines the KDL-TL and NLOPT-IK
    /// algorithms to provide fast and robust inverse kinematics solutions.
    class TRAC_IK {
    public:
        enum SolveType {
            Speed,    // Prioritize speed over accuracy
            Distance, // Prioritize solution closest to seed
            Manip1,   // Prioritize manipulability
            Manip2    // Prioritize secondary manipulability measure
        };

        TRAC_IK(
            const KDL::Chain& chain,
            const KDL::JntArray& q_min,
            const KDL::JntArray& q_max,
            double maxtime = 0.005,
            double eps = 1e-3,
            SolveType type = Speed);

        /// \name Configuration
        ///@{
        void setBounds(const KDL::Twist& bounds) { bounds_ = bounds; }
        auto bounds() const -> const KDL::Twist& { return bounds_; }

        void setEps(double eps) { eps_ = eps; }
        double eps() const { return eps_; }

        void setMaxTime(double maxtime) { maxtime_ = maxtime; }
        double maxTime() const { return maxtime_; }

        void setSolveType(SolveType type) { solve_type_ = type; }
        SolveType solveType() const { return solve_type_; }
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
            const KDL::Twist&    _bounds);

        /// Return the KDL solver used by TRAC-IK
        const std::unique_ptr<KDL::ChainIkSolverPos_TL>& getKDL() const { return kdl_solver_; }

        /// Return the NLOPT solver used by TRAC-IK
        const std::unique_ptr<NLOPT_IK::NLOPT_IK>& getNLOPT() const { return nlopt_solver_; }

    private:

        const KDL::Chain& chain_;
        KDL::JntArray joint_min_, joint_max_;
        std::vector<KDL::BasicJointType> joint_types_;

        double maxtime_;
        double eps_;
        SolveType solve_type_;

        std::unique_ptr<KDL::ChainIkSolverPos_TL> kdl_solver_;
        std::unique_ptr<NLOPT_IK::NLOPT_IK> nlopt_solver_;

        KDL::JntArray q_out_;
        KDL::Frame f_target_;
        KDL::Twist bounds_;
        int progress_;

        // Random number generator for random restarts
        std::default_random_engine rng_;

        // Helper functions
        void randomize(KDL::JntArray& q);
        double manipulability(const KDL::JntArray& q);
        double manipulability2(const KDL::JntArray& q);
    };

} // namespace TRAC_IK

#endif