#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

#include <vector>
#include <memory>
#include <nlopt.h>
#include <nlopt.hpp>  // NLOPT C++ 接口

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <Eigen/Dense>

namespace NLOPT_IK {

    // 关节类型枚举
    enum BasicJointType {
        RotJoint, TransJoint, Continuous
    };


    /// 优化目标类型
    enum OptType {
        Joint,   ///< 最小化与目标关节配置的距离
        SumSq,   ///< 最小化误差平方和（基于 Twist）
        L2       ///< 最小化误差的 L2 范数
        // DualQuat 已移除，请见下方说明
    };

    /// 基于 NLOPT 的非线性优化逆运动学求解器
    class NLOPT_IK {
    public:
        /**
         * @brief 构造函数
         * @param chain KDL 运动链
         * @param q_min 各关节最小值
         * @param q_max 各关节最大值
         * @param maxtime 最大优化时间（秒，默认 0.005）
         * @param eps 优化精度（默认 1e-3）
         * @param type 优化目标类型（默认 SumSq）
         */
        NLOPT_IK(
            const KDL::Chain& chain,
            const KDL::JntArray& q_min,
            const KDL::JntArray& q_max,
            double maxtime = 0.005,
            double eps = 1e-3,
            OptType type = SumSq);

        // --------------------------
        // 配置接口
        // --------------------------
        void setBounds(const KDL::Twist& bounds) { bounds_ = bounds; }
        auto bounds() const -> const KDL::Twist& { return bounds_; }

        void setEps(double eps) { eps_ = eps; }
        double eps() const { return eps_; }

        void setMaxTime(double maxtime) { maxtime_ = maxtime; }
        double maxTime() const { return maxtime_; }

        void setOptType(OptType type) { opt_type_ = type; }
        OptType optType() const { return opt_type_; }

        // --------------------------
        // 求解接口
        // --------------------------
        /// 重置求解器状态（带目标位姿）
        void restart(const KDL::JntArray& q_init, const KDL::Frame& p_in);

        /// 重置求解器状态（不带目标位姿）
        void restart(const KDL::JntArray& q_init);

        /// 执行单步优化
        int step(int steps = 1);

        /// 获取当前关节解
        const KDL::JntArray& qout() const { return q_out_; }

        /// 执行完整优化流程
        int CartToJnt(
            const KDL::JntArray& q_init,
            const KDL::Frame& p_in,
            KDL::JntArray& q_out,
            const KDL::Twist& bounds = KDL::Twist::Zero());

    private:
        // --------------------------
        // NLOPT 相关
        // --------------------------
        std::unique_ptr<nlopt::opt> nlopt_;

        // --------------------------
        // KDL 相关
        // --------------------------
        const KDL::Chain& chain_;
        KDL::JntArray joint_min_, joint_max_;
        KDL::ChainFkSolverPos_recursive fk_solver_;
        KDL::JntArray q_tmp_;

        // --------------------------
        // 参数
        // --------------------------
        double maxtime_;
        double eps_;
        OptType opt_type_;

        // --------------------------
        // 数据
        // --------------------------
        std::vector<double> x_min_, x_max_, best_x_;
        KDL::JntArray q_out_;
        KDL::Frame f_target_;
        KDL::Twist bounds_;
        int progress_;

        std::vector<double> target_pos_;  // 用于关节最小化目标
        std::vector<BasicJointType> joint_types_;

        // --------------------------
        // 优化目标函数
        // --------------------------
        double minJoints(const std::vector<double>& x, std::vector<double>& grad);
        void cartSumSquaredError(const std::vector<double>& x, double error[]);
        void cartL2NormError(const std::vector<double>& x, double error[]);

        // --------------------------
        // 优化控制
        // --------------------------
        bool optimize(std::vector<double>& x, double& minf);
        void clipToJointLimits(std::vector<double>& x);
    };

} // namespace NLOPT_IK

#endif // NLOPT_IK_HPP