#include <iostream>
#include <string>
#include "xarm/wrapper/xarm_api.h"

// Helper to print robot telemetry and status
void print_robot_status(XArmAPI *arm) {
    fp32 pose[6];
    fp32 angles[7];
    int state, mode, err, warn;

    arm->get_position(pose);
    arm->get_servo_angle(angles);
    arm->get_state(&state);
    arm->get_mode(&mode);
    arm->get_err_warn_code(&err, &warn);

    printf("\n--- ROBOT STATUS ---");
    printf("State: %d | Mode: %d | Err: %d | Warn: %d\n", state, mode, err, warn);
    printf("TCP Pose (x,y,z,r,p,y): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    printf("Joint Angles: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
           angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
    printf("--------------------\n\n");
}

// Configures motion dynamics (Speeds & Accelerations)
void configure_motion_params(XArmAPI *arm) {
    // TCP Linear Motion limits (mm/s and mm/s^2)
    arm->set_tcp_jerk(10000);
    arm->set_tcp_maxacc(5000);
    arm->set_position_metric(1); // mm/deg metric

    // Joint Motion limits (deg/s and deg/s^2)
    arm->set_joint_jerk(500);
    arm->set_joint_maxacc(1000);

    // Set default operating speeds
    arm->set_self_collision_detection(true);
    arm->set_collision_sensitivity(3);
}

// Demonstrates Cartesian and Relative movement
void execute_cartesian_moves(XArmAPI *arm) {
    printf("[MOTION] Executing Cartesian Linear Movement...\n");
    
    // Set Target Linear Velocity (mm/s) and Acceleration (mm/s^2)
    fp32 linear_speed = 100.0;
    fp32 linear_acc = 500.0;

    fp32 target_pose[6] = { 350.0, 100.0, 250.0, 180.0, 0.0, 0.0 };
    int ret = arm->set_position(target_pose, -1, linear_speed, linear_acc, 0, true);
    printf("set_position (Absolute) ret = %d\n", ret);

    // Relative offset movement (dx, dy, dz, drx, dry, drz)
    printf("[MOTION] Executing Relative TCP Shift...\n");
    fp32 relative_pose[6] = { 0.0, -200.0, 50.0, 0.0, 0.0, 0.0 };
    ret = arm->set_tool_position(relative_pose, true); // Axis relative to Tool Frame
    printf("set_tool_position (Relative) ret = %d\n", ret);
}

// Demonstrates Joint-space trajectory movements
void execute_joint_moves(XArmAPI *arm) {
    printf("[MOTION] Executing Joint-Space Movement...\n");

    fp32 joint_speed = 30.0; // deg/s
    fp32 joint_acc = 200.0;  // deg/s^2

    // Target joint angles in degrees
    fp32 joint_target_1[7] = { 0.0, -15.0, -30.0, 0.0, 45.0, 0.0, 0.0 };
    fp32 joint_target_2[7] = { 30.0, 0.0, -45.0, 10.0, 35.0, 15.0, 0.0 };

    int ret = arm->set_servo_angle(joint_target_1, joint_speed, joint_acc, 0, true);
    printf("set_servo_angle (Target 1) ret = %d\n", ret);

    ret = arm->set_servo_angle(joint_target_2, joint_speed, joint_acc, 0, true);
    printf("set_servo_angle (Target 2) ret = %d\n", ret);
}

// Handles End-Effector Gripper operations
void operate_gripper(XArmAPI *arm) {
    printf("[END EFFECTOR] Initializing and testing Gripper...\n");

    arm->set_gripper_enable(true);
    arm->set_gripper_mode(0); // Location mode
    arm->set_gripper_speed(3000);

    // Open Gripper
    printf("Opening Gripper...\n");
    arm->set_gripper_position(850, true); // 850 is fully open for xArm Gripper
    sleep_milliseconds(500);

    // Close Gripper
    printf("Closing Gripper...\n");
    arm->set_gripper_position(0, true); // 0 is fully closed
    sleep_milliseconds(500);
}

// Dynamic real-time mode (ServoJ / ServoCartesian streaming)
void execute_servo_motion(XArmAPI *arm) {
    printf("[STREAMING] Switching to Mode 1 (Servo Motion)...\n");

    // Mode 1 is for real-time low-latency trajectory control (ServoJ/Servo Cartesian)
    arm->set_mode(1);
    arm->set_state(0);
    sleep_milliseconds(100);

    fp32 current_pose[6];
    arm->get_position(current_pose);

    // Stream small incremental steps at ~100Hz
    printf("[STREAMING] Streaming sinusoidal z-axis offset...\n");
    for (int i = 0; i < 200; ++i) {
        current_pose[2] += 0.5; // Move up gradually
        arm->set_servo_cartesian(current_pose);
        sleep_milliseconds(10); // 10ms loop rate (100Hz)
    }

    // Switch back to Mode 0 (Standard Position Control)
    arm->set_mode(0);
    arm->set_state(0);
    sleep_milliseconds(100);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: ./xarm_demo <Robot_IP>\n");
        return -1;
    }

    std::string ip(argv[1]);
    XArmAPI *arm = new XArmAPI(ip);

    // Initial setup and safe fault handling
    sleep_milliseconds(500);
    if (arm->error_code != 0) arm->clean_error();
    if (arm->warn_code != 0) arm->clean_warn();

    arm->motion_enable(true);
    arm->set_mode(0); // Mode 0: Position Control Mode
    arm->set_state(0); // State 0: Ready
    sleep_milliseconds(500);

    // System Operations Pipeline
    configure_motion_params(arm);
    print_robot_status(arm);

    // 1. Move Home
    printf("[ROUTINE] Moving to Home Position...\n");
    arm->move_gohome(true);

    // 2. Linear Cartesian Moves
    execute_cartesian_moves(arm);

    // 3. Axis-Angle Pose sequence (Your original routine)
    printf("[ROUTINE] Executing Axis-Angle movements...\n");
    fp32 poses[4][6] = {
        {0, 0, 0, 0, 50, 0},
        {0, 0, 0, 0, -50, 0},
        {0, 0, 0, 0, 0, 80},
        {0, 0, 0, 0, 0, -80},
    };
    for (int i = 0; i < 4; i++) {
        int ret = arm->set_position_aa(poses[i], false, true, true);
        printf("set_position_aa [%d], ret=%d\n", i, ret);
    }

    // 4. Joint-Space Moves
    execute_joint_moves(arm);

    // 5. Streamed Servo Motion
    execute_servo_motion(arm);

    // 6. Operate Gripper
    operate_gripper(arm);

    // 7. Cleanup & Finish
    print_robot_status(arm);
    printf("[ROUTINE] Returning to Home and Disabling Motion...\n");
    arm->move_gohome(true);
    arm->motion_enable(false);

    delete arm;
    return 0;
}
