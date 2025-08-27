#ifndef CHAIN_IK_SOLVER_VEL_PINV_H
#define CHAIN_IK_SOLVER_VEL_PINV_H

#include "kdl_types.h"
#include <vector>
#include <cmath>
#include <limits>
#include "tiny_svd.hpp"


namespace KDL {

    class ChainIkSolverVel_pinv {
    public:
        explicit ChainIkSolverVel_pinv(const Chain& _chain) : chain(_chain), jac_solver(_chain) {
            nj = chain.getNrOfJoints();
            // Allocate memory for SVD computation
            int m = 6;
            int n = nj;
            U.resize(m * n);
            V.resize(n * n);
            S.resize(std::min(m, n));
            work.resize(std::max(m, n));
            tmp.resize(n);
        }

        int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out) {
            // Check dimensions
            if (q_in.rows() != nj || qdot_out.rows() != nj)
                return -1;

            // Compute the Jacobian
            Jacobian jac(nj);
            jac_solver.JntToJac(q_in, jac);

            // Convert twist to vector
            std::vector<double> v(6);
            v[0] = v_in.vel.x();
            v[1] = v_in.vel.y();
            v[2] = v_in.vel.z();
            v[3] = v_in.rot.x();
            v[4] = v_in.rot.y();
            v[5] = v_in.rot.z();

            // Copy Jacobian to U
            for (unsigned int i = 0; i < 6; i++) {
                for (unsigned int j = 0; j < nj; j++) {
                    U[i * nj + j] = jac(i, j);
                }
            }

            // Compute SVD
            computeSVD(6, nj, U.data(), S.data(), V.data());

            // Compute pseudo-inverse
            double max_singular_value = S[0];
            double tolerance = std::max(6, (int)nj) * max_singular_value * std::numeric_limits<double>::epsilon();

            for (unsigned int i = 0; i < S.size(); i++) {
                if (S[i] > tolerance) {
                    tmp[i] = 1.0 / S[i];
                }
                else {
                    tmp[i] = 0.0;
                }
            }

            // Compute qdot_out = V * S+ * U^T * v
            // First compute U^T * v
            std::vector<double> Utv(S.size(), 0.0);
            for (unsigned int i = 0; i < S.size(); i++) {
                for (unsigned int j = 0; j < 6; j++) {
                    Utv[i] += U[j * nj + i] * v[j];
                }
            }

            // Then compute S+ * U^T * v
            std::vector<double> SplusUtv(S.size(), 0.0);
            for (unsigned int i = 0; i < S.size(); i++) {
                SplusUtv[i] = tmp[i] * Utv[i];
            }

            // Finally compute V * S+ * U^T * v
            for (unsigned int i = 0; i < nj; i++) {
                qdot_out(i) = 0.0;
                for (unsigned int j = 0; j < S.size(); j++) {
                    qdot_out(i) += V[i * nj + j] * SplusUtv[j];
                }
            }

            return 0;
        }

    private:
        const Chain& chain;
        unsigned int nj;
        ChainJntToJacSolver jac_solver;
        std::vector<double> U, V, S, work, tmp;

        void computeSVD(int m, int n, double* A, double* S, double* V) {
            tiny::jacobiSVD(m, n, A, S, V);
}
    };

} // namespace KDL

#endif