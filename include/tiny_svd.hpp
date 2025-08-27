// tiny_svd.hpp
#pragma once
#include <cmath>
#include <algorithm>
#include <limits>

namespace tiny {

// 仅支持 m >= n, A[m][n]
inline void jacobiSVD(int m, int n, double* A, double* S, double* V)
{
    // 初始化 V = I
    for (int i = 0; i < n * n; ++i) V[i] = 0.0;
    for (int i = 0; i < n; ++i) V[i * n + i] = 1.0;

    double eps = 1e-12;
    int max_iter = 50;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        bool converged = true;
        for (int p = 0; p < n; ++p)
        {
            for (int q = p + 1; q < n; ++q)
            {
                double app = 0.0, aqq = 0.0, apq = 0.0;
                for (int i = 0; i < m; ++i)
                {
                    int idx_i_p = i * n + p;
                    int idx_i_q = i * n + q;
                    app += A[idx_i_p] * A[idx_i_p];
                    aqq += A[idx_i_q] * A[idx_i_q];
                    apq += A[idx_i_p] * A[idx_i_q];
                }
                if (std::fabs(apq) < eps * std::sqrt(app * aqq)) continue;

                converged = false;
                double tau = (aqq - app) / (2.0 * apq);
                double t = (tau >= 0 ? 1.0 : -1.0) / (std::fabs(tau) + std::sqrt(1.0 + tau * tau));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;

                // 旋转 A 的列 p,q
                for (int i = 0; i < m; ++i)
                {
                    int ip = i * n + p;
                    int iq = i * n + q;
                    double tmp_p = A[ip] * c - A[iq] * s;
                    double tmp_q = A[ip] * s + A[iq] * c;
                    A[ip] = tmp_p;
                    A[iq] = tmp_q;
                }
                // 旋转 V 的列 p,q
                for (int i = 0; i < n; ++i)
                {
                    int ip = i * n + p;
                    int iq = i * n + q;
                    double tmp_p = V[ip] * c - V[iq] * s;
                    double tmp_q = V[ip] * s + V[iq] * c;
                    V[ip] = tmp_p;
                    V[iq] = tmp_q;
                }
            }
        }
        if (converged) break;
    }

    // 计算奇异值（列向量的 2-范数）
    for (int j = 0; j < n; ++j)
    {
        double norm = 0.0;
        for (int i = 0; i < m; ++i) norm += A[i * n + j] * A[i * n + j];
        S[j] = std::sqrt(norm);
    }

    // 排序（降序）
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            if (S[i] < S[j])
            {
                std::swap(S[i], S[j]);
                for (int k = 0; k < m; ++k) std::swap(A[k * n + i], A[k * n + j]);
                for (int k = 0; k < n; ++k) std::swap(V[k * n + i], V[k * n + j]);
            }
        }
    }
}

} // namespace tiny