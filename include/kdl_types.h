#ifndef KDL_TYPES_H
#define KDL_TYPES_H

#include <vector>
#include <cmath>
#include <iostream>
#include <random>
#include <cassert>
#include "math3d.h"

namespace KDL {

    // Basic joint types
    enum BasicJointType {
        RotJoint, TransJoint, Continuous
    };

    // Vector class for 3D vectors
    class Vector {
    public:
        double data[3];

        Vector() {
            data[0] = 0.0;
            data[1] = 0.0;
            data[2] = 0.0;
        }

        Vector(double x, double y, double z) {
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        double x() const { return data[0]; }
        double y() const { return data[1]; }
        double z() const { return data[2]; }

        double& x() { return data[0]; }
        double& y() { return data[1]; }
        double& z() { return data[2]; }

        double operator()(int i) const { return data[i]; }
        double& operator()(int i) { return data[i]; }

        Vector operator+(const Vector& v) const {
            return Vector(data[0] + v.data[0], data[1] + v.data[1], data[2] + v.data[2]);
        }

        Vector operator-(const Vector& v) const {
            return Vector(data[0] - v.data[0], data[1] - v.data[1], data[2] - v.data[2]);
        }

        Vector operator-() const {
            return Vector(-data[0], -data[1], -data[2]);
        }

        Vector operator*(double s) const {
            return Vector(data[0] * s, data[1] * s, data[2] * s);
        }

        Vector operator/(double s) const {
            return Vector(data[0] / s, data[1] / s, data[2] / s);
        }

        double dot(const Vector& v) const {
            return data[0] * v.data[0] + data[1] * v.data[1] + data[2] * v.data[2];
        }

        Vector cross(const Vector& v) const {
            return Vector(
                data[1] * v.data[2] - data[2] * v.data[1],
                data[2] * v.data[0] - data[0] * v.data[2],
                data[0] * v.data[1] - data[1] * v.data[0]
            );
        }

        double Norm() const {
            return std::sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
        }

        void Normalize() {
            double n = Norm();
            if (n > 0.0) {
                data[0] /= n;
                data[1] /= n;
                data[2] /= n;
            }
        }

        friend std::ostream& operator<<(std::ostream& os, const Vector& v) {
            os << "[" << v.data[0] << ", " << v.data[1] << ", " << v.data[2] << "]";
            return os;
        }
    };

    // Rotation class using rotation matrix
    class Rotation {
    public:
        double data[9];  // Row-major: 0,1,2 = first row

        Rotation() {
            // Identity matrix
            for (int i = 0; i < 9; i++) {
                data[i] = 0.0;
            }
            data[0] = 1.0;
            data[4] = 1.0;
            data[8] = 1.0;
        }

        Rotation(double r00, double r01, double r02,
            double r10, double r11, double r12,
            double r20, double r21, double r22) {
            data[0] = r00; data[1] = r01; data[2] = r02;
            data[3] = r10; data[4] = r11; data[5] = r12;
            data[6] = r20; data[7] = r21; data[8] = r22;
        }

        double operator()(int i, int j) const { return data[i * 3 + j]; }
        double& operator()(int i, int j) { return data[i * 3 + j]; }

        Vector operator*(const Vector& v) const {
            return Vector(
                data[0] * v.data[0] + data[1] * v.data[1] + data[2] * v.data[2],
                data[3] * v.data[0] + data[4] * v.data[1] + data[5] * v.data[2],
                data[6] * v.data[0] + data[7] * v.data[1] + data[8] * v.data[2]
            );
        }

        Rotation operator*(const Rotation& R) const {
            Rotation result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result(i, j) = 0.0;
                    for (int k = 0; k < 3; k++) {
                        result(i, j) += (*this)(i, k) * R(k, j);
                    }
                }
            }
            return result;
        }

        Rotation Inverse() const {
            Rotation result;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result(i, j) = (*this)(j, i);
                }
            }
            return result;
        }

        void GetEulerZYX(double& alpha, double& beta, double& gamma) const {
            if (std::abs(data[6]) < 1.0 - 1e-10) {
                beta = std::asin(-data[6]);
                alpha = std::atan2(data[3], data[0]);
                gamma = std::atan2(data[7], data[8]);
            }
            else {
                // Gimbal lock
                if (data[6] > 0) {
                    beta = M_PI / 2.0;
                    alpha = std::atan2(data[7], data[4]);
                    gamma = 0.0;
                }
                else {
                    beta = -M_PI / 2.0;
                    alpha = std::atan2(-data[7], -data[4]);
                    gamma = 0.0;
                }
            }
        }

        static Rotation EulerZYX(double alpha, double beta, double gamma) {
            double ca = std::cos(alpha), sa = std::sin(alpha);
            double cb = std::cos(beta), sb = std::sin(beta);
            double cg = std::cos(gamma), sg = std::sin(gamma);

            return Rotation(
                ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg,
                sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg,
                -sb, cb * sg, cb * cg
            );
        }

        static Rotation RotX(double angle) {
            double c = std::cos(angle);
            double s = std::sin(angle);
            return Rotation(
                1, 0, 0,
                0, c, -s,
                0, s, c
            );
        }

        static Rotation RotY(double angle) {
            double c = std::cos(angle);
            double s = std::sin(angle);
            return Rotation(
                c, 0, s,
                0, 1, 0,
                -s, 0, c
            );
        }

        static Rotation RotZ(double angle) {
            double c = std::cos(angle);
            double s = std::sin(angle);
            return Rotation(
                c, -s, 0,
                s, c, 0,
                0, 0, 1
            );
        }

        static Rotation RPY(double roll, double pitch, double yaw) {
            return EulerZYX(yaw, pitch, roll);
        }

        friend std::ostream& operator<<(std::ostream& os, const Rotation& R) {
            os << "[" << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << "; "
                << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << "; "
                << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2) << "]";
            return os;
        }
    };

    // Frame class (pose)
    class Frame {
    public:
        Rotation M;
        Vector p;

        Frame() : M(), p() {}

        Frame(const Rotation& R, const Vector& V) : M(R), p(V) {}

        static Frame Identity() {
            return Frame(Rotation(), Vector(0, 0, 0));
        }

        Frame operator*(const Frame& T) const {
            return Frame(M * T.M, M * T.p + p);
        }

        Vector operator*(const Vector& v) const {
            return M * v + p;
        }

        Frame Inverse() const {
            Rotation Minv = M.Inverse();
            return Frame(Minv, Minv * (p * -1.0));
        }

        friend std::ostream& operator<<(std::ostream& os, const Frame& f) {
            os << "Frame: p=" << f.p << ", M=" << f.M;
            return os;
        }
    };

    // Twist class (velocity)
    class Twist {
    public:
        Vector vel;
        Vector rot;

        Twist() : vel(), rot() {}

        Twist(const Vector& _vel, const Vector& _rot) : vel(_vel), rot(_rot) {}

        Twist operator+(const Twist& t) const {
            return Twist(vel + t.vel, rot + t.rot);
        }

        Twist operator-() const {
            return Twist(-vel, -rot);
        }

        Twist operator-(const Twist& t) const {
            return Twist(vel - t.vel, rot - t.rot);
        }

        Twist operator*(double s) const {
            return Twist(vel * s, rot * s);
        }

        static Twist Zero() {
            return Twist(Vector(0, 0, 0), Vector(0, 0, 0));
        }

        friend std::ostream& operator<<(std::ostream& os, const Twist& t) {
            os << "Twist: vel=" << t.vel << ", rot=" << t.rot;
            return os;
        }
    };

    // Joint array
    class JntArray {
    public:
        std::vector<double> data;

        JntArray() {}

        explicit JntArray(unsigned int size) {
            resize(size);
        }

        void resize(unsigned int newSize) {
            data.resize(newSize, 0.0);
        }

        unsigned int rows() const {
            return data.size();
        }

        double operator()(unsigned int i) const {
            return data[i];
        }

        double& operator()(unsigned int i) {
            return data[i];
        }

        JntArray operator+(const JntArray& arg) const {
            assert(data.size() == arg.data.size());
            JntArray result(data.size());
            for (unsigned int i = 0; i < data.size(); i++) {
                result.data[i] = data[i] + arg.data[i];
            }
            return result;
        }

        JntArray operator-(const JntArray& arg) const {
            assert(data.size() == arg.data.size());
            JntArray result(data.size());
            for (unsigned int i = 0; i < data.size(); i++) {
                result.data[i] = data[i] - arg.data[i];
            }
            return result;
        }

        JntArray operator*(double arg) const {
            JntArray result(data.size());
            for (unsigned int i = 0; i < data.size(); i++) {
                result.data[i] = data[i] * arg;
            }
            return result;
        }

        JntArray& operator+=(const JntArray& arg) {
            assert(data.size() == arg.data.size());
            for (unsigned int i = 0; i < data.size(); i++) {
                data[i] += arg.data[i];
            }
            return *this;
        }

        JntArray& operator-=(const JntArray& arg) {
            assert(data.size() == arg.data.size());
            for (unsigned int i = 0; i < data.size(); i++) {
                data[i] -= arg.data[i];
            }
            return *this;
        }

        JntArray& operator*=(double arg) {
            for (unsigned int i = 0; i < data.size(); i++) {
                data[i] *= arg;
            }
            return *this;
        }

        bool isZero(double eps = 1e-10) const {
            for (unsigned int i = 0; i < data.size(); i++) {
                if (std::abs(data[i]) > eps) {
                    return false;
                }
            }
            return true;
        }

        friend std::ostream& operator<<(std::ostream& os, const JntArray& q) {
            os << "[";
            for (unsigned int i = 0; i < q.data.size(); i++) {
                if (i > 0) os << ", ";
                os << q.data[i];
            }
            os << "]";
            return os;
        }
    };

    // Jacobian matrix
    class Jacobian {
    public:
        std::vector<double> data;
        unsigned int rows, columns;

        Jacobian(unsigned int size) : rows(6), columns(size) {
            data.resize(rows * columns, 0.0);
        }

        double operator()(unsigned int i, unsigned int j) const {
            return data[i * columns + j];
        }

        double& operator()(unsigned int i, unsigned int j) {
            return data[i * columns + j];
        }

        unsigned int getNrOfColumns() const { return columns; }
        unsigned int getNrOfRows() const { return rows; }

        void resize(unsigned int newColumns) {
            columns = newColumns;
            data.resize(rows * columns, 0.0);
        }

        void setZero() {
            std::fill(data.begin(), data.end(), 0.0);
        }
    };

    // Joint type
    class Joint {
    public:
        enum JointType {
            RotX, RotY, RotZ, TransX, TransY, TransZ, None
        };

        JointType type;
        std::string name;
        double scale;
        double offset;

        Joint(JointType _type, const std::string& _name = "", double _scale = 1.0, double _offset = 0.0)
            : type(_type), name(_name), scale(_scale), offset(_offset) {}

        std::string getTypeName() const {
            switch (type) {
            case RotX: return "RotX";
            case RotY: return "RotY";
            case RotZ: return "RotZ";
            case TransX: return "TransX";
            case TransY: return "TransY";
            case TransZ: return "TransZ";
            case None: return "None";
            default: return "Unknown";
            }
        }

        Frame pose(double q) const {
            double val = scale * q + offset;
            switch (type) {
            case RotX: return Frame(Rotation::RotX(val), Vector(0, 0, 0));
            case RotY: return Frame(Rotation::RotY(val), Vector(0, 0, 0));
            case RotZ: return Frame(Rotation::RotZ(val), Vector(0, 0, 0));
            case TransX: return Frame(Rotation(), Vector(val, 0, 0));
            case TransY: return Frame(Rotation(), Vector(0, val, 0));
            case TransZ: return Frame(Rotation(), Vector(0, 0, val));
            case None: return Frame(Rotation(), Vector(0, 0, 0));
            default: return Frame(Rotation(), Vector(0, 0, 0));
            }
        }

        Twist twist(double q, double qdot) const {
            (void)q; // Mark as used to avoid warning
            double val = scale * qdot;
            switch (type) {
            case RotX: return Twist(Vector(0, 0, 0), Vector(val, 0, 0));
            case RotY: return Twist(Vector(0, 0, 0), Vector(0, val, 0));
            case RotZ: return Twist(Vector(0, 0, 0), Vector(0, 0, val));
            case TransX: return Twist(Vector(val, 0, 0), Vector(0, 0, 0));
            case TransY: return Twist(Vector(0, val, 0), Vector(0, 0, 0));
            case TransZ: return Twist(Vector(0, 0, val), Vector(0, 0, 0));
            case None: return Twist(Vector(0, 0, 0), Vector(0, 0, 0));
            default: return Twist(Vector(0, 0, 0), Vector(0, 0, 0));
            }
        }
    };

    // Segment of a kinematic chain
    class Segment {
    public:
        std::string name;
        Joint joint;
        Frame f_tip;  // Frame from joint end to segment tip

        Segment(const std::string& _name, const Joint& _joint, const Frame& _tip = Frame())
            : name(_name), joint(_joint), f_tip(_tip) {}

        const Joint& getJoint() const { return joint; }
        const Frame& getFrameToTip() const { return f_tip; }
    };

    // Kinematic chain
    class Chain {
    public:
        std::vector<Segment> segments;

        Chain() {}

        void addSegment(const Segment& segment) {
            segments.push_back(segment);
        }

        unsigned int getNrOfJoints() const {
            unsigned int nr = 0;
            for (unsigned int i = 0; i < segments.size(); i++) {
                if (segments[i].getJoint().getTypeName() != "None") {
                    nr++;
                }
            }
            return nr;
        }

        unsigned int getNrOfSegments() const {
            return segments.size();
        }

        const Segment& getSegment(unsigned int nr) const {
            return segments[nr];
        }
    };

    // Helper functions
    inline Twist diff(const Frame& F_a_b1, const Frame& F_a_b2, double dt = 1.0) {
        // Difference between two frames
        Rotation R_a_b1 = F_a_b1.M;
        Rotation R_a_b2 = F_a_b2.M;
        Vector p_a_b1 = F_a_b1.p;
        Vector p_a_b2 = F_a_b2.p;

        Rotation R_b1_b2 = R_a_b1.Inverse() * R_a_b2;

        // Angular velocity
        Vector omega_b1_b2;
        if (std::abs(R_b1_b2(0, 0) + R_b1_b2(1, 1) + R_b1_b2(2, 2) - 3) < 1e-10) {
            // Small angle approximation
            omega_b1_b2 = Vector(
                (R_b1_b2(2, 1) - R_b1_b2(1, 2)) / 2.0,
                (R_b1_b2(0, 2) - R_b1_b2(2, 0)) / 2.0,
                (R_b1_b2(1, 0) - R_b1_b2(0, 1)) / 2.0
            );
        }
        else {
            // Large angle
            double angle = std::acos((R_b1_b2(0, 0) + R_b1_b2(1, 1) + R_b1_b2(2, 2) - 1.0) / 2.0);
            double s = std::sin(angle);
            if (std::abs(s) < 1e-10) {
                omega_b1_b2 = Vector(0, 0, 0);
            }
            else {
                omega_b1_b2 = Vector(
                    (R_b1_b2(2, 1) - R_b1_b2(1, 2)) / (2.0 * s) * angle,
                    (R_b1_b2(0, 2) - R_b1_b2(2, 0)) / (2.0 * s) * angle,
                    (R_b1_b2(1, 0) - R_b1_b2(0, 1)) / (2.0 * s) * angle
                );
            }
        }

        // Linear velocity
        Vector v_a_b1_b2 = (p_a_b2 - p_a_b1) / dt;

        // Transform angular velocity to frame a
        Vector omega_a_b1_b2 = R_a_b1 * omega_b1_b2;

        return Twist(v_a_b1_b2, omega_a_b1_b2);
    }

    inline Twist diffRelative(const Frame& F_a_b1, const Frame& F_a_b2, double dt = 1.0) {
        // Difference relative to frame b1
        Twist t = diff(F_a_b1, F_a_b2, dt);
        // Transform to frame b1
        Rotation R_a_b1_inv = F_a_b1.M.Inverse();
        return Twist(R_a_b1_inv * t.vel, R_a_b1_inv * t.rot);
    }

    inline bool Equal(const Twist& a, const Twist& b, double eps = 1e-10) {
        return std::abs(a.vel.x() - b.vel.x()) < eps &&
            std::abs(a.vel.y() - b.vel.y()) < eps &&
            std::abs(a.vel.z() - b.vel.z()) < eps &&
            std::abs(a.rot.x() - b.rot.x()) < eps &&
            std::abs(a.rot.y() - b.rot.y()) < eps &&
            std::abs(a.rot.z() - b.rot.z()) < eps;
    }

    inline double dot(const Twist& a, const Twist& b) {
        return a.vel.dot(b.vel) + a.rot.dot(b.rot);
    }

    inline void Add(const JntArray& src1, const JntArray& src2, JntArray& dest) {
        assert(src1.rows() == src2.rows());
        assert(src1.rows() == dest.rows());
        for (unsigned int i = 0; i < src1.rows(); i++) {
            dest(i) = src1(i) + src2(i);
        }
    }

    inline void Subtract(const JntArray& src1, const JntArray& src2, JntArray& dest) {
        assert(src1.rows() == src2.rows());
        assert(src1.rows() == dest.rows());
        for (unsigned int i = 0; i < src1.rows(); i++) {
            dest(i) = src1(i) - src2(i);
        }
    }

} // namespace KDL

#endif