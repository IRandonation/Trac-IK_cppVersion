#ifndef CHAIN_JNT_TO_JAC_SOLVER_H
#define CHAIN_JNT_TO_JAC_SOLVER_H

#include "kdl_types.h"

namespace KDL {

    class ChainJntToJacSolver {
    public:
        explicit ChainJntToJacSolver(const Chain& _chain) : chain(_chain) {}

        int JntToJac(const JntArray& q_in, Jacobian& jac, int segmentNr = -1) {
            if (segmentNr < 0) segmentNr = chain.getNrOfSegments();
            if (segmentNr > (int)chain.getNrOfSegments()) return -1;
            if (q_in.rows() != chain.getNrOfJoints()) return -2;
                
            jac.setZero();
            Frame T_total = Frame::Identity();
            unsigned int j = 0;  // 活动关节索引
                
            for (int i = 0; i < segmentNr; ++i) {
                const Segment& seg = chain.getSegment(i);
                const Joint&     jnt = seg.getJoint();
            
                // 局部变换
                Frame T_local = Frame::Identity();
                if (jnt.type != Joint::None) {
                    T_local = jnt.pose(q_in(j));
                    ++j;
                }
                T_local = T_local * seg.getFrameToTip();
            
                // 计算该关节的 twist（仅活动关节）
                if (jnt.type != Joint::None) {
                    Twist t_local = jnt.twist(q_in(j - 1), 1.0);
                    Twist t_base;
                    t_base.vel = T_total.M * t_local.vel;
                    t_base.rot = T_total.M * t_local.rot;
                
                    for (unsigned int k = 0; k < 3; ++k) {
                        jac(k, j - 1)     = t_base.vel(k);
                        jac(k + 3, j - 1) = t_base.rot(k);
                    }
                }
            
                T_total = T_total * T_local;
            }
            return 0;
        }

    private:
        const Chain& chain;
    };

} // namespace KDL

#endif