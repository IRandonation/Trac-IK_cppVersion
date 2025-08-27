#include <iostream>
#include <chrono>
#include <cmath>

#include "trac_ik.h"

// Create a simple 7-DOF robot arm
KDL::Chain create7DOFRobotArm() {
    KDL::Chain chain;
    
    // Base to joint 1
    chain.addSegment(KDL::Segment("base_link",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    // Joint 1 to joint 2
    chain.addSegment(KDL::Segment("link1",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.2))));
    
    // Joint 2 to joint 3
    chain.addSegment(KDL::Segment("link2",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.2))));
    
    // Joint 3 to joint 4
    chain.addSegment(KDL::Segment("link3",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    // Joint 4 to joint 5
    chain.addSegment(KDL::Segment("link4",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.2))));
    
    // Joint 5 to joint 6
    chain.addSegment(KDL::Segment("link5",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    // Joint 6 to end effector
    chain.addSegment(KDL::Segment("link6",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Rotation(), KDL::Vector(0.0, 0.0, 0.1))));
    
    return chain;
}

int main(int argc, char** argv) {
    // Create a 7-DOF robot arm
    KDL::Chain chain = create7DOFRobotArm();
    
    // Set joint limits (in radians)
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray q_min(nj), q_max(nj);
    
    for (unsigned int i = 0; i < nj; i++) {
        q_min(i) = -M_PI;
        q_max(i) = M_PI;
    }
    
    // Create TRAC-IK solver
    TRAC_IK::TRAC_IK solver(chain, q_min, q_max, 0.005, 1e-3, TRAC_IK::TRAC_IK::Speed);
    
    // Set initial joint angles
    KDL::JntArray q_init(nj);
    for (unsigned int i = 0; i < nj; i++) {
        q_init(i) = 0.0;
    }
    
    // // Set target pose
    // KDL::Frame target_pose(
    //     KDL::Rotation::RPY(0.0, 0.0, 0.0),       // 无旋转
    //     KDL::Vector(0.0, 0.0, 0.0)              // 靠近基座，正前方上方
    // );
    KDL::Frame target_pose;
    KDL::ChainFkSolverPos_recursive fk(chain);
    q_init(2)+=0.1;
    fk.JntToCart(q_init, target_pose);
    // target_pose.p.z(target_pose.p.z());
    
    // Solve IK
    KDL::JntArray q_out(nj);
    KDL::Twist bounds(KDL::Vector(0.001, 0.001, 0.001), 
                     KDL::Vector(0.01, 0.01, 0.01));
    
    auto start_time = std::chrono::high_resolution_clock::now();
    int result = solver.CartToJnt(q_init, target_pose, q_out, bounds);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Print results
    std::cout << "TRAC-IK Example" << std::endl;
    std::cout << "=============" << std::endl;
    std::cout << "Number of joints: " << nj << std::endl;
    std::cout << "Target pose: " << target_pose << std::endl;
    std::cout << "Initial joint angles: " << q_init << std::endl;
    
    if (result >= 0) {
        std::cout << "Solution found!" << std::endl;
        std::cout << "Solution joint angles: " << q_out << std::endl;
        std::cout << "Computation time: " << duration.count() << " microseconds" << std::endl;
        
        // Verify solution by forward kinematics
        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::Frame computed_pose;
        fk_solver.JntToCart(q_out, computed_pose);
        
        std::cout << "Computed pose: " << computed_pose << std::endl;
        
        // Compute error
        KDL::Twist error = KDL::diffRelative(target_pose, computed_pose);
        std::cout << "Position error: " << error.vel.Norm() << " m" << std::endl;
        std::cout << "Orientation error: " << error.rot.Norm() << " rad" << std::endl;
    } else {
        std::cout << "No solution found!" << std::endl;
        std::cout << "Computation time: " << duration.count() << " microseconds" << std::endl;
    }
    
    // Test different solve types
    std::cout << "\nTesting different solve types:" << std::endl;
    
    std::vector<TRAC_IK::TRAC_IK::SolveType> solve_types = {
        TRAC_IK::TRAC_IK::Speed,
        TRAC_IK::TRAC_IK::Distance,
        TRAC_IK::TRAC_IK::Manip1,
        TRAC_IK::TRAC_IK::Manip2
    };
    
    std::vector<std::string> solve_type_names = {
        "Speed",
        "Distance",
        "Manipulability 1",
        "Manipulability 2"
    };
    
    for (size_t i = 0; i < solve_types.size(); i++) {
        TRAC_IK::TRAC_IK test_solver(chain, q_min, q_max, 0.005, 1e-3, solve_types[i]);
        
        start_time = std::chrono::high_resolution_clock::now();
        result = test_solver.CartToJnt(q_init, target_pose, q_out, bounds);
        end_time = std::chrono::high_resolution_clock::now();
        
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        std::cout << solve_type_names[i] << ": ";
        if (result >= 0) {
            std::cout << "Success (" << duration.count() << " μs)" << std::endl;
        } else {
            std::cout << "Failed (" << duration.count() << " μs)" << std::endl;
        }
    }
    
    return 0;
}