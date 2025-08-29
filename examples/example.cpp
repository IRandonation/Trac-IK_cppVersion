#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <random>

#include "trac_ik.h"

// 创建一个简单的 7-DOF 机械臂模型
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

// 打印关节角度
void printJntArray(const std::string& label, const KDL::JntArray& q) {
    std::cout << label << ": [";
    for (unsigned int i = 0; i < q.rows(); i++) {
        std::cout << q(i);
        if (i < q.rows() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// 打印位姿
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

// 压力测试统计
struct TestStats {
    std::string name;
    int success_count;
    int total_tests;
    double total_time;
    double min_time;
    double max_time;
    std::vector<double> all_times;
    
    TestStats(const std::string& n) : name(n), success_count(0), total_tests(0),
                                     total_time(0.0), min_time(1e9), max_time(0.0) {}
    
    void addResult(bool success, double time_us) {
        total_tests++;
        if (success) {
            success_count++;
        }
        all_times.push_back(time_us);
        total_time += time_us;
        min_time = std::min(min_time, time_us);
        max_time = std::max(max_time, time_us);
    }
    
    void printStats() const {
        double success_rate = (total_tests > 0) ? (100.0 * success_count / total_tests) : 0.0;
        double avg_time = (success_count > 0) ? (total_time / success_count) : 0.0;
        
        std::cout << std::fixed << std::setprecision(2);
        std::cout << name << ":\n";
        std::cout << "  Success Rate: " << success_rate << "% (" << success_count << "/" << total_tests << ")\n";
        std::cout << "  Average Time: " << avg_time << " us\n";
        std::cout << "  Min Time: " << min_time << " us\n";
        std::cout << "  Max Time: " << max_time << " us\n";
        
        if (success_count > 1) {
            double variance = 0.0;
            for (double time : all_times) {
                variance += (time - avg_time) * (time - avg_time);
            }
            variance /= success_count;
            double std_dev = sqrt(variance);
            std::cout << "  Time Std Dev: " << std_dev << " us\n";
        }
        std::cout << std::endl;
    }
};

// 压力测试函数（已修改：随机化 q_init 和通过 FK 生成 target_pose）
void runStressTest(const KDL::Chain& chain, const KDL::JntArray& q_min,
                   const KDL::JntArray& q_max, int test_count = 100) {
    
    std::cout << "\nStarting Stress Test - Each method tested " << test_count << " times with random q_init & target_pose\n";
    std::cout << "========================================\n" << std::endl;

    std::vector<TRAC_IK::TRAC_IK::SolveType> solve_types = {
        TRAC_IK::TRAC_IK::Speed,
        TRAC_IK::TRAC_IK::Distance,
        TRAC_IK::TRAC_IK::Manip1,
        TRAC_IK::TRAC_IK::Manip2
    };
    std::vector<std::string> solve_type_names = {
        "Speed (Serial)",
        "Distance (Serial)",
        "Manip1 (Serial)",
        "Manip2 (Serial)"
    };

    std::vector<TestStats> stats;
    for (const auto& name : solve_type_names) {
        stats.emplace_back(name);
    }

    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray q_out(nj);
    KDL::JntArray q_init(nj);

    // 随机数生成器
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-M_PI, M_PI);

    for (size_t i = 0; i < solve_types.size(); i++) {
        TRAC_IK::TRAC_IK solver(chain, q_min, q_max, 0.005, 1e-3, solve_types[i]);

        for (int j = 0; j < test_count; j++) {
            // 1. 随机生成初始关节角度 q_init ∈ [-π, π]
            for (unsigned int k = 0; k < nj; k++) {
                q_init(k) = distribution(generator);
            }

            // 2. 通过 FK 计算当前 q_init 对应的位姿，作为目标位姿 target_pose
            KDL::ChainFkSolverPos_recursive fk(chain);
            KDL::Frame target_pose;
            fk.JntToCart(q_init, target_pose);
            KDL::Twist bounds(KDL::Vector(0.001, 0.001, 0.001),   // 位置误差容限
                             KDL::Vector(0.01, 0.01, 0.01));      // 旋转误差容限
            // 3. 求解逆运动学：从 q_init（其实可以是任意值）到 target_pose
            auto start_time = std::chrono::high_resolution_clock::now();


            int result = solver.CartToJnt(q_init, target_pose, q_out, bounds);            
            auto end_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            bool success = (result >= 0);
            stats[i].addResult(success, duration.count());
        }
    }

    // 输出统计结果
    std::cout << "Stress Test Results:\n";
    std::cout << "==================\n" << std::endl;

    for (const auto& stat : stats) {
        stat.printStats();
    }

    // 找出成功率最高的方法
    auto best_it = std::max_element(stats.begin(), stats.end(),
        [](const TestStats& a, const TestStats& b) {
            double a_rate = (a.total_tests > 0) ? (100.0 * a.success_count / a.total_tests) : 0.0;
            double b_rate = (b.total_tests > 0) ? (100.0 * b.success_count / b.total_tests) : 0.0;
            if (a_rate != b_rate) return a_rate < b_rate;
            double a_avg = (a.success_count > 0) ? (a.total_time / a.success_count) : 1e9;
            double b_avg = (b.success_count > 0) ? (b.total_time / b.success_count) : 1e9;
            return a_avg > b_avg;
        });

    std::cout << "Recommended Method: " << best_it->name << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "TRAC-IK example (updated: no parallel, random q_init & target_pose in stress test)\n";
    std::cout << "=============\n" << std::endl;

    // 1. 创建机械臂模型
    KDL::Chain chain = create7DOFRobotArm();
    unsigned int nj = chain.getNrOfJoints();

    // 2. 设置关节限位
    KDL::JntArray q_min(nj), q_max(nj);
    for (unsigned int i = 0; i < nj; i++) {
        q_min(i) = -M_PI;
        q_max(i) = M_PI;
    }

    // 3. （可选）单次求解测试（您可以根据需要保留或删除这部分）
    /*
    TRAC_IK::TRAC_IK solver(chain, q_min, q_max, 0.005, 1e-3, TRAC_IK::TRAC_IK::Speed);
    KDL::JntArray q_init(nj);
    for (unsigned int i = 0; i < nj; i++) q_init(i) = 0.0;

    KDL::Frame target_pose;  // 您可以手动设置一个目标，或通过 FK 设置
    KDL::ChainFkSolverPos_recursive fk(chain);
    fk.JntToCart(q_init, target_pose);

    KDL::JntArray q_out(nj);
    KDL::Twist bounds(KDL::Vector(0.001, 0.001, 0.001), KDL::Vector(0.01, 0.01, 0.01));

    int result = solver.CartToJnt(q_init, target_pose, q_out, bounds);

    if (result >= 0) {
        printJntArray("solution", q_out);
    } else {
        std::cout << "No solution found." << std::endl;
    }
    */

    // 4. 压力测试：随机 q_init 和随机（但可达的）target_pose
    runStressTest(chain, q_min, q_max, 100);  // 100 次测试

    return 0;
}