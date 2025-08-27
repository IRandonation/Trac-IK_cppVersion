#ifndef KDLCHAINIKSOLVERPOS_TL_HPP
#define KDLCHAINIKSOLVERPOS_TL_HPP

#include <random>
#include "kdl_types.h"
#include "chainfksolverpos_recursive.h"
#include "chainjnttojacsolver.h"
#include "chainiksolvervel_pinv.h"

namespace KDL {

class ChainIkSolverPos_TL {
public:
    ChainIkSolverPos_TL(const Chain& chain,
                        const JntArray& q_min,
                        const JntArray& q_max,
                        double eps = 1e-3,
                        bool random_restart = false,
                        bool try_jl_wrap = false);

    /*--------- 配置 ---------*/
    void setBounds(const Twist& tol);
    const Twist& bounds() const;
    void setEps(double eps);
    double eps() const;

    /*--------- 迭代接口 ---------*/
    void restart(const JntArray& q_init, const Frame& p_in);
    void restart(const JntArray& q_init);
    int step(int steps = 1);
    const JntArray& qout() const;

    /*--------- 一次性求解 ---------*/
    int CartToJnt(const JntArray& q_init,
                  const Frame& p_in,
                  JntArray& q_out,
                  const Twist& bounds = Twist::Zero());

    bool isConverged() const;

private:
    void randomize(JntArray& q);
    void clamp(JntArray& q);

    /*--------- 成员 ---------*/
    const Chain chain_;
    JntArray joint_min_, joint_max_;
    std::vector<KDL::BasicJointType> joint_types_;
    std::default_random_engine rng_;
    ChainIkSolverVel_pinv vik_solver_;
    ChainFkSolverPos_recursive fk_solver_;

    Twist bounds_;
    double eps_;
    bool rr_, wrap_;
    JntArray q_curr_, q_tmp_, delta_q_;
    Frame f_target_, f_curr_;
    bool done_;
};

} // namespace KDL
#endif