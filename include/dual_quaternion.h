#ifndef DUAL_QUATERNION_HPP
#define DUAL_QUATERNION_HPP

#include "math3d.h"

using math3d::point3d;
using math3d::quaternion;

template <typename T>
inline int sign(T v)
{
    return (v < 0) ? -1 : 1;
}

struct dual_quaternion
{
    quaternion<double> R;
    quaternion<double> tR_2;

    dual_quaternion(double v = 1.0) : R(v), tR_2(0) { }

    static constexpr double dq_epsilon = 1e-8;

    static
        dual_quaternion rigid_transformation(
            const quaternion<double>& r,
            const point3d& t)
    {
        dual_quaternion result;
        result.R = r;
        result.tR_2 = (quaternion<double>::convert(t) * r) *= 0.5;
        return result;
    }

    static
        dual_quaternion convert(const double* p)
    {
        dual_quaternion result;
        result.R = quaternion<double>::convert(p);
        result.tR_2 = quaternion<double>::convert(p + 4);
        return result;
    }

    dual_quaternion& normalize()
    {
        double n = math3d::norm(R) * sign(R.w);
        R *= 1.0 / n;
        tR_2 *= 1.0 / n;
        double d = math3d::dot(R, tR_2);
        //tR_2 += (-d)*R;
        quaternion<double> r2 = R;
        r2 *= -d;
        tR_2 += r2;
        return *this;
    }

    point3d get_translation()
    {
        quaternion<double> t = tR_2 * ~R;
        point3d result;
        result.x = 2 * t.i;
        result.y = 2 * t.j;
        result.z = 2 * t.k;
        return result;
    }

    void to_vector(double* p)
    {
        R.to_vector(p); tR_2.to_vector(p + 4);
    }

    dual_quaternion& operator+=(const dual_quaternion& a)
    {
        R += a.R;
        tR_2 += a.tR_2;
        return *this;
    }

    dual_quaternion& operator*=(double a)
    {
        R *= a;
        tR_2 *= a;
        return *this;
    }

    // computes log map tangent at identity
    // assumes qual_quaternion is unitary
    dual_quaternion& log()
    {
        const double h0 = std::acos(R.w);
        //small angle approximation: sin(h0)=h0, cos(h0)=1
        if (h0 * h0 < dq_epsilon) {
            R.w = 0.0;
            R *= 0.5;
            tR_2.w = 0.0;
            tR_2 *= 0.5;
        }
        else {
            R.w = 0.0;
            const double ish0 = 1.0 / math3d::norm(R);
            // R *= ish0;
            math3d::normalize(R); // R = s0
            const double he = -tR_2.w * ish0;
            tR_2.w = 0.0;

            quaternion<double> Rp(R);
            Rp *= -math3d::dot(R, tR_2) / math3d::dot(R, R);
            tR_2 += Rp;
            tR_2 *= ish0; // tR_2 = se

            tR_2 *= h0;
            Rp = R;
            Rp *= he;
            tR_2 += Rp;
            tR_2 *= 0.5;
            R *= h0 * 0.5;
        }

        return *this;
    }

    // computes exp map tangent at identity
    // assumes qual_quaternion is on tangent space
    dual_quaternion& exp()
    {
        const double h0 = 2.0 * math3d::norm(R);

        // small angle approximation: sin(h0)=h0, cos(h0)=1
        if (h0 * h0 < dq_epsilon) {
            R *= 2.0;
            R.w = 1.0;
            tR_2 *= 2.0;
            // normalize();
        }
        else {
            const double he = 4.0 * math3d::dot(tR_2, R) / h0;
            const double sh0 = sin(h0);
            const double ch0 = cos(h0);
            quaternion<double> Rp(R);
            Rp *= -math3d::dot(R, tR_2) / math3d::dot(R, R);
            tR_2 += Rp;
            tR_2 *= 2.0 / h0; //tR_2=se

            tR_2 *= sh0;
            Rp = R;
            Rp *= he * ch0 * 2.0 / h0;
            tR_2 += Rp;
            tR_2.w = -he * sh0;

            R *= sh0 * 2.0 / h0;
            R.w = ch0;
        }

        normalize();
        return *this;
    }
};

inline
dual_quaternion operator*(const dual_quaternion& a, const dual_quaternion& b)
{
    dual_quaternion result;
    result.R = a.R * b.R;
    result.tR_2 = a.R * b.tR_2 + a.tR_2 * b.R;
    return result;
}

inline
dual_quaternion operator~(const dual_quaternion& a)
{
    dual_quaternion result;
    result.R = ~a.R;
    result.tR_2 = ((~a.tR_2) *= -1);
    return result;
}

inline
dual_quaternion operator!(const dual_quaternion& a)
{
    dual_quaternion result;
    result.R = ~a.R;
    result.tR_2 = ~a.tR_2;
    return result;
}

inline
double dot(const dual_quaternion& a, const dual_quaternion& b)
{
    return math3d::dot(a.R, b.R) + math3d::dot(a.tR_2, b.tR_2);
}

inline dual_quaternion log(dual_quaternion a) { return a.log(); }
inline dual_quaternion exp(dual_quaternion a) { return a.exp(); }

inline
std::ostream& operator<<(std::ostream& out, const dual_quaternion& dq)
{
    return out << "( " << dq.R.w << ", " << dq.R.i << ", " << dq.R.j << ", " <<
        dq.R.k << ",  " << dq.tR_2.w << ", " << dq.tR_2.i << ", " <<
        dq.tR_2.j << ", " << dq.tR_2.k << " )";
}

#endif