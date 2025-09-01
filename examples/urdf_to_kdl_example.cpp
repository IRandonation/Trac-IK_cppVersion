#include <iostream>
#include <iomanip>
#include <chrono>
#include <memory>

#include "urdf_kdl_converter.h"
#include "trac_ik.h"

// 打印KDL Chain信息
void printChainInfo(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max) {
    std::cout << "=== KDL Chain Information ===" << std::endl;
    std::cout << "Number of segments: " << chain.getNrOfSegments() << std::endl;
    std::cout << "Number of joints: " << chain.getNrOfJoints() << std::endl;
    
    std::cout << "\n=== Joint Limits ===" << std::endl;
    for (unsigned int i = 0; i < q_min.rows(); ++i) {
        std::cout << "Joint " << i << ": [" << q_min(i) << ", " << q_max(i) << "]" << std::endl;
    }
    
    std::cout << "\n=== Chain Segments ===" << std::endl;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain.getSegment(i);
        std::cout << "Segment " << i << ": " << segment.getName() << std::endl;
        
        const KDL::Joint& joint = segment.getJoint();
        std::cout << "  Joint type: ";
        switch (joint.getType()) {
            case KDL::Joint::RotAxis:
                std::cout << "RotAxis";
                break;
            case KDL::Joint::TransAxis:
                std::cout << "TransAxis";
                break;
            case KDL::Joint::None:
                std::cout << "Fixed";
                break;
            default:
                std::cout << "Unknown";
                break;
        }
        std::cout << std::endl;
        
        std::cout << "  Joint origin: [" << joint.JointOrigin().x() << ", " 
                  << joint.JointOrigin().y() << ", " << joint.JointOrigin().z() << "]" << std::endl;
        std::cout << "  Joint axis: [" << joint.JointAxis().x() << ", " 
                  << joint.JointAxis().y() << ", " << joint.JointAxis().z() << "]" << std::endl;
    }
}

// 打印机器人数据信息
void printRobotData(const std::shared_ptr<URDF_KDL::RobotData>& robot_data) {
    if (!robot_data) {
        std::cout << "No robot data available" << std::endl;
        return;
    }
    
    std::cout << "=== Robot Data Information ===" << std::endl;
    std::cout << "Robot name: " << robot_data->name << std::endl;
    std::cout << "Root link: " << robot_data->root_link_name << std::endl;
    std::cout << "Number of links: " << robot_data->links.size() << std::endl;
    std::cout << "Number of joints: " << robot_data->joints.size() << std::endl;
    
    std::cout << "\n=== Links ===" << std::endl;
    for (const auto& link_pair : robot_data->links) {
        std::cout << "Link: " << link_pair.first << std::endl;
    }
    
    std::cout << "\n=== Joints ===" << std::endl;
    for (const auto& joint_pair : robot_data->joints) {
        const auto& joint = joint_pair.second;
        std::cout << "Joint: " << joint->name << " (" << joint->parent_link_name 
                  << " -> " << joint->child_link_name << ")" << std::endl;
        std::cout << "  Type: ";
        switch (joint->type) {
            case URDF_KDL::JointData::Type::REVOLUTE:
                std::cout << "Revolute";
                break;
            case URDF_KDL::JointData::Type::CONTINUOUS:
                std::cout << "Continuous";
                break;
            case URDF_KDL::JointData::Type::PRISMATIC:
                std::cout << "Prismatic";
                break;
            case URDF_KDL::JointData::Type::FIXED:
                std::cout << "Fixed";
                break;
            default:
                std::cout << "Unknown";
                break;
        }
        std::cout << std::endl;
        
        if (joint->limits) {
            std::cout << "  Limits: [" << joint->limits->lower << ", " << joint->limits->upper << "]" << std::endl;
        }
    }
}

// 测试TRAC-IK求解器
void testTRACIK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max) {
    std::cout << "\n=== Testing TRAC-IK Solver ===" << std::endl;
    
    // 创建TRAC-IK求解器
    TRAC_IK::TRAC_IK solver(chain, q_min, q_max, 0.005, 1e-3, TRAC_IK::TRAC_IK::Speed);
    
    // 创建随机初始关节角度
    KDL::JntArray q_init(chain.getNrOfJoints());
    for (unsigned int i = 0; i < q_init.rows(); ++i) {
        q_init(i) = (q_min(i) + q_max(i)) / 2.0;
    }
    
    // 计算正向运动学得到目标位姿
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame target_pose;
    if (fk_solver.JntToCart(q_init, target_pose) >= 0) {
        std::cout << "Target pose calculated successfully" << std::endl;
        
        // 打印目标位姿
        std::cout << "Target position: [" << target_pose.p.x() << ", " 
                  << target_pose.p.y() << ", " << target_pose.p.z() << "]" << std::endl;
        
        double roll, pitch, yaw;
        target_pose.M.GetRPY(roll, pitch, yaw);
        std::cout << "Target orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
        
        // 求解逆运动学
        KDL::JntArray q_out(chain.getNrOfJoints());
        KDL::Twist bounds(KDL::Vector(0.001, 0.001, 0.001), KDL::Vector(0.01, 0.01, 0.01));
        
        auto start_time = std::chrono::high_resolution_clock::now();
        int result = solver.CartToJnt(q_init, target_pose, q_out, bounds);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        if (result >= 0) {
            std::cout << "IK solution found successfully!" << std::endl;
            std::cout << "Solution time: " << duration.count() << " microseconds" << std::endl;
            
            std::cout << "Initial joint angles: [";
            for (unsigned int i = 0; i < q_init.rows(); ++i) {
                std::cout << q_init(i);
                if (i < q_init.rows() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            
            std::cout << "Solution joint angles: [";
            for (unsigned int i = 0; i < q_out.rows(); ++i) {
                std::cout << q_out(i);
                if (i < q_out.rows() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            
            // 验证解的正确性
            KDL::Frame computed_pose;
            if (fk_solver.JntToCart(q_out, computed_pose) >= 0) {
                KDL::Twist error = KDL::diff(target_pose, computed_pose);
                double pos_error = error.vel.Norm();
                double rot_error = error.rot.Norm();
                
                std::cout << "Position error: " << pos_error << " m" << std::endl;
                std::cout << "Orientation error: " << rot_error << " rad" << std::endl;
            }
        } else {
            std::cout << "Failed to find IK solution" << std::endl;
        }
    } else {
        std::cout << "Failed to calculate target pose" << std::endl;
    }
}

int main(int argc, char** argv) {
    std::cout << "URDF to KDL Chain Builder Example" << std::endl;
    std::cout << "=================================" << std::endl;
    
    // URDF文件路径
    std::string urdf_path;
    if (argc > 1) {
        urdf_path = argv[1];
    } else {
        // 使用默认的URDF文件
        urdf_path = "examples/robot.urdf";
    }
    
    std::cout << "Loading URDF file: " << urdf_path << std::endl;
    
    // 创建URDF到KDL转换器
    URDF_KDL::URDFToKDLConverter converter;
    
    // 从文件加载URDF
    if (!converter.loadFromFile(urdf_path)) {
        std::cerr << "Error loading URDF file: " << converter.getErrorMessage() << std::endl;
        return 1;
    }
    
    std::cout << "URDF file loaded successfully!" << std::endl;
    
    // 验证机器人模型
    if (!converter.validateRobotModel()) {
        std::cerr << "Robot model validation failed: " << converter.getErrorMessage() << std::endl;
        return 1;
    }
    
    std::cout << "Robot model validation passed!" << std::endl;
    
    // 获取机器人数据
    auto robot_data = converter.getRobotData();
    printRobotData(robot_data);
    
    // 转换为KDL Chain
    KDL::Chain chain;
    KDL::JntArray q_min, q_max;
    
    if (!converter.toKDLChain(chain, q_min, q_max)) {
        std::cerr << "Error converting to KDL Chain: " << converter.getErrorMessage() << std::endl;
        return 1;
    }
    
    std::cout << "\nSuccessfully converted to KDL Chain!" << std::endl;
    printChainInfo(chain, q_min, q_max);
    
    // 测试TRAC-IK求解器
    testTRACIK(chain, q_min, q_max);
    
    // 测试指定根和末端的Chain
    std::cout << "\n=== Testing Specific Chain ===" << std::endl;
    
    // 假设我们知道机器人的根和末端链接名称
    std::string root_link = "base_link";
    std::string tip_link = "link2";
    
    KDL::Chain specific_chain;
    KDL::JntArray specific_q_min, specific_q_max;
    
    // 验证链路是否存在
    if (!converter.validateLink(root_link)) {
        std::cout << "Root link '" << root_link << "' does not exist" << std::endl;
    } else if (!converter.validateLink(tip_link)) {
        std::cout << "Tip link '" << tip_link << "' does not exist" << std::endl;
    } else if (!converter.validatePath(root_link, tip_link)) {
        std::cout << "No valid path from '" << root_link << "' to '" << tip_link << "'" << std::endl;
        std::cout << "Error: " << converter.getErrorMessage() << std::endl;
    } else if (converter.toKDLChain(root_link, tip_link, specific_chain, specific_q_min, specific_q_max)) {
        std::cout << "Successfully created specific chain from '" << root_link << "' to '" << tip_link << "'" << std::endl;
        printChainInfo(specific_chain, specific_q_min, specific_q_max);
        
        // 测试特定Chain的TRAC-IK求解器
        testTRACIK(specific_chain, specific_q_min, specific_q_max);
    } else {
        std::cout << "Failed to create specific chain from '" << root_link << "' to '" << tip_link << "'" << std::endl;
        std::cout << "Error: " << converter.getErrorMessage() << std::endl;
    }
    
    return 0;
}