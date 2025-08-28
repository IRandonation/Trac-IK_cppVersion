#include <iostream>
#include <chrono>
#include <cmath>

#include "trac_ik.h"

/**
 * @brief 创建一个简单的 7-DOF 机械臂模型
 */
KDL::Chain create7DOFRobotArm() {
    KDL::Chain chain;
    
    chain.addSegment(KDL::Segment("base_link",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    chain.addSegment(KDL::Segment("link1",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.2))));
    
    chain.addSegment(KDL::Segment("link2",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.2))));
    
    chain.addSegment(KDL::Segment("link3",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    chain.addSegment(KDL::Segment("link4",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.2))));
    
    chain.addSegment(KDL::Segment("link5",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    chain.addSegment(KDL::Segment("link6",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    return chain;
}

// ✅ 辅助函数：打印关节角度数组
void printJntArray(const std::string& label, const KDL::JntArray& q) {
    std::cout << label << ": [";
    for (unsigned int i = 0; i < q.rows(); i++) {
        std::cout << q(i);
        if (i < q.rows() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// ✅ 辅助函数：打印位姿（位置 + RPY）
void printFrame(const std::string& label, const KDL::Frame& f) {
    std::cout << label << ":" << std::endl;
    std::cout << "  position: [" 
              << f.p.x() << ", " 
              << f.p.y() << ", " 
              << f.p.z() << "]" << std::endl;

    double roll, pitch, yaw;
    f.M.GetRPY(roll, pitch, yaw);
    std::cout << "  rotation(RPY): [" 
              << roll << ", " 
              << pitch << ", " 
              << yaw << "] (rad)" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "TRAC-IK example" << std::endl;
    std::cout << "=============" << std::endl;
    
    // 1. 创建机械臂模型
    KDL::Chain chain = create7DOFRobotArm();
    unsigned int nj = chain.getNrOfJoints();

    // 2. 设置关节限位
    KDL::JntArray q_min(nj), q_max(nj);
    for (unsigned int i = 0; i < nj; i++) {
        q_min(i) = -M_PI;
        q_max(i) = M_PI;
    }

    // 3. 创建 TRAC-IK 求解器
    TRAC_IK::TRAC_IK solver(chain, q_min, q_max, 0.005, 1e-3, TRAC_IK::TRAC_IK::Speed);

    // 4. 设置初始关节角度
    KDL::JntArray q_init(nj);
    for (unsigned int i = 0; i < nj; i++) {
        q_init(i) = 0.0;
    }

    // 5. 设置目标位姿（通过 FK 保证可达）
    KDL::Frame target_pose;
    KDL::ChainFkSolverPos_recursive fk(chain);
    q_init.resize(nj);  // 确保大小正确，nj 是关节数，比如 7
    q_init(0) = 0.1;
    q_init(1) = 1.0;
    q_init(2) = 0.54;
    q_init(3) = -0.32;
    q_init(4) = 0.01;
    q_init(5) = -1.2;    
    fk.JntToCart(q_init, target_pose);

    // ✅ 打印输入信息（使用辅助函数！）
    std::cout << "RobotArm joints number: " << nj << std::endl;
    printFrame("target pose", target_pose);
    printJntArray("init pose", q_init);

    // 6. 求解逆运动学
    KDL::JntArray q_out(nj);
    KDL::Twist bounds(KDL::Vector(0.001, 0.001, 0.001),   // 位置容限
                     KDL::Vector(0.01, 0.01, 0.01));      // 姿态容限

    auto start_time = std::chrono::high_resolution_clock::now();
    // int result = solver.CartToJnt(q_init, target_pose, q_out, bounds);
    int result = solver.CartToJntParallel(q_init, target_pose, q_out, bounds, 300.0);  // 300ms 超时

    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // 7. 输出结果
    if (result >= 0) {
        std::cout << "Find solution!" << std::endl;
        std::cout << "calculate time: " << duration.count() << " us" << std::endl;

        printJntArray("joint pose output", q_out);

        // 正向验证
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::Frame computed_pose;
        fk_solver.JntToCart(q_out, computed_pose);

        printFrame("calculate pose", computed_pose);

        // 误差
        KDL::Twist error = KDL::diff(computed_pose, target_pose);
        std::cout << "position error (m): " << error.vel.Norm() << std::endl;
        std::cout << "pose error (rad): " << error.rot.Norm() << std::endl;

    } else {
        std::cout << "no solution!" << std::endl;
        std::cout << "calculate time: " << duration.count() << " us" << std::endl;
    }

    // 8. 测试不同求解策略
    std::cout << "\ntest different trajectory:" << std::endl;
    std::vector<TRAC_IK::TRAC_IK::SolveType> solve_types = {
        TRAC_IK::TRAC_IK::Speed,
        TRAC_IK::TRAC_IK::Distance,
        TRAC_IK::TRAC_IK::Manip1,
        TRAC_IK::TRAC_IK::Manip2
    };
    std::vector<std::string> solve_type_names = {
        "speed first",
        "distance first",
        "multiple1",
        "multiple2"
    };

    for (size_t i = 0; i < solve_types.size(); i++) {
        TRAC_IK::TRAC_IK test_solver(chain, q_min, q_max, 0.005, 1e-3, solve_types[i]);
        start_time = std::chrono::high_resolution_clock::now();
        result = test_solver.CartToJnt(q_init, target_pose, q_out, bounds);
        end_time = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        std::cout << solve_type_names[i] << ": ";
        if (result >= 0) {
            std::cout << "success (" << duration.count() << " us)" << std::endl;
        } else {
            std::cout << "fail (" << duration.count() << " us)" << std::endl;
        }
    }

    return 0;
}