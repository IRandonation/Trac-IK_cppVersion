#ifndef CHAIN_FK_SOLVER_POS_RECURSIVE_H
#define CHAIN_FK_SOLVER_POS_RECURSIVE_H

#include "kdl_types.h"

namespace KDL {

    class ChainFkSolverPos_recursive {
    public:
        explicit ChainFkSolverPos_recursive(const Chain& _chain) : chain(_chain) {}

        int JntToCart(const JntArray& q_in, Frame& p_out, int segmentNr = -1) {
            if (segmentNr < 0)
                segmentNr = chain.getNrOfSegments();

            if (segmentNr > (int)chain.getNrOfSegments())
                return -1;

            if (q_in.rows() != chain.getNrOfJoints())
                return -2;  // 关节数不匹配
                
            p_out = Frame::Identity();
            int j = 0;
            for (int i = 0; i < segmentNr; i++) {
                if (chain.getSegment(i).getJoint().getTypeName() != "None") {
                    p_out = p_out * chain.getSegment(i).getJoint().pose(q_in(j));
                    j++;
                }
                else {
                    p_out = p_out * chain.getSegment(i).getJoint().pose(0.0);
                }
                p_out = p_out * chain.getSegment(i).getFrameToTip();
            }
            return 0;
        }

    private:
        const Chain& chain;
    };

} // namespace KDL

#endif