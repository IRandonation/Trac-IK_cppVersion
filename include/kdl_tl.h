#ifndef CHAIN_IK_SOLVER_POS_TL_H
#define CHAIN_IK_SOLVER_POS_TL_H

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <random>
#include <cmath>
#include <cassert>
#include <vector>

// 可选：引入 Eigen（推荐）
#include <Eigen/Dense>

namespace KDL {

    // 关节类型枚举
    enum BasicJointType {
        RotJoint, TransJoint, Continuous
    };

    class ChainIkSolverPos_TL {
    public:
        ChainIkSolverPos_TL(
            const Chain& chain,
            const JntArray& q_min,
            const JntArray& q_max,
            double eps = 1e-5,
            bool random_restart = true,
            bool try_jl_wrap = true);

        void setBounds(const Twist& tol);
        const Twist& bounds() const;
        void setEps(double eps);
        double eps() const;

        void restart(const JntArray& q_init, const Frame& p_in);
        void restart(const JntArray& q_init);

        const JntArray& qout() const;

        int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out, const Twist& bounds = Twist::Zero());
        int step(int steps = 1);

    private:
        Twist diff(const Frame& a, const Frame& b); // 计算相对位姿误差 Twist
        static void Add(const JntArray& q1, const JntArray& dq, JntArray& q2); // q1 + dq -> q2
        void clamp(JntArray& q); // 关节限位处理
        void randomize(JntArray& q); // 随机初始化（用于随机重启）

        // 成员变量
        const Chain& chain_;
        const JntArray joint_min_;
        const JntArray joint_max_;
        ChainFkSolverPos_recursive fk_solver_;
        ChainIkSolverVel_pinv vik_solver_;

        JntArray q_curr_;
        JntArray q_tmp_;
        JntArray delta_q_;

        double eps_;
        bool rr_;   // random restart
        bool wrap_; // wrap around joint limits
        bool done_;

        JntArray q_out_;

        Twist bounds_;
        Frame f_curr_;
        Frame f_target_;

        std::mt19937 rng_;
        std::vector<BasicJointType> joint_types_;
    };

} // namespace KDL

#endif // CHAIN_IK_SOLVER_POS_TL_H