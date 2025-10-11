#pragma once
#include <string>
#include <array>
#include <mutex>

constexpr int num_steps  = 9;
constexpr int num_joints = 16;

// All shared states for control demos
struct CtrlContext {
    int step_index = -1;
    bool joint_done[num_joints] = {};
    bool all_joints_done = false;
    bool move_allowed[num_joints] = {};
    bool all_move_allowed = false;
    bool plan_flag[num_joints] = {};
    bool quick_stop = false;
    bool end = false;
    std::array<int, num_joints> joint_mode{};
    std::array<bool, num_joints> enable_flags{};
    
    int main_cnt = -1;

    std::array<int, num_joints> joint_map = {
      0, 1, // head
      4, 3, 2, 5, 8, 7, 6, // left arm
      11,10,9,13,15,14,12  // right arm
    };  

    std::array<std::string, num_joints> joint_name = { 
      "H1", "H2", // head
      "L1", "L2", "L3", "L4", "L5", "LF", "LB", // left arm
      "R1", "R2", "R3", "R4", "R5", "RF", "RB" // right arm
    };

    std::array<double, num_joints> target_joint_vel{};
    std::array<float, num_joints> q{};
    std::array<float, num_joints> q_rot{};
    std::array<float, num_joints> target_q{0.00000, 0.00000,
                                           0.09292, 0.78706, 0.00749, -0.94108, -0.00470, 6.63745, 16.88646,
                                           -0.18298, -0.27286, -1.43342, 1.04043, 1.44569, 13.38606, 14.08255};
    std::array<float, num_joints> old_target_q{};
    std::array<float, num_joints> target_q_rot{};
    std::array<float, num_joints> target_q_dot{};
    std::array<float, num_joints> command_q_dot{};
    std::array<float, num_joints> qdot_kp{1.0, 1.0,
                                          1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                          1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,};
    std::array<float, num_joints> qdot_ki{1.0, 1.0,
                                          1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                          1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,};
    std::array<float, num_joints> q_err{};

    mutable std::mutex current_m;
    mutable std::mutex target_m;


    double target_joint_posi[num_steps][num_joints] = { 
      //Head     Left Arm                                     Right Arm 
      {0.0, 0.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  0.0,   -15.0,  0.0,    20.0,  0.0,  0.0,  0.0},
      {0.0, 0.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -120.0,  95.0,  90.0,  0.0,  0.0},
      {0.0, 0.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -110.0,  65.0,  90.0,  0.0,  0.0},
      {0.0, -30.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -130.0,  125.0,  90.0,  0.0,  0.0},
      {0.0, -20.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -110.0,  65.0,  90.0,  0.0,  0.0},
      {0.0, 30.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -130.0,  125.0,  90.0,  0.0,  0.0},
      {0.0, 20.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -110.0,  65.0,  90.0,  0.0,  0.0},
      {0.0, 0.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  -50.0, -50.0,  -130.0,  125.0,  90.0,  0.0,  0.0},
      {0.0, 0.0, 0.0,  15.0, 0.0,   -15.0,   0.0,  0.0, 0.0,  0.0,  -15.0,  0.0,    20.0,  0.0,  0.0,  0.0}
    };
    
    int joint_zero_posi[num_joints] = { 
    // head
    71115,
    38274,

    // left arm
    61601,
    42570,
    75126,
    69473,
    72827,
    -148201, //LF(L6)
    -160915, //LB(L7)

    // right arm
    67347,
    93106,
    65526,
    63598,
    69355,
    42213, //RF(R6)
    -152968 //RB(R7)
    };

    // float q[num_joints] = {
    //   // head
    //   0.0,
    //   0.0,

    //   // left arm
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0, //LF(L6)
    //   0.0, //LB(L7)

    //   // right arm
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0, //RF(R6)
    //   0.0 //RB(R7)
    // };

    // float q_rot[num_joints] = {
    //   // head
    //   0.0,
    //   0.0,

    //   // left arm
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0, //roll
    //   0.0, //pitch

    //   // right arm
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0,
    //   0.0, //roll
    //   0.0 //pitch
    // };

    double time_window[num_steps] = {
    0.5, 0.9, 0.35, 0.5, 0.5, 0.5, 0.5, 0.5, 0.9
    };
};