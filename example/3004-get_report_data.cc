/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2023, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include "xarm/wrapper/xarm_api.h"

// Global flag to manage loop termination across threads
std::atomic<bool> g_running(true);

// Decodes system status flags from raw binary payload bytes
void parse_system_status(unsigned char *data_fp) {
    int total = bin8_to_32(data_fp);
    int state = data_fp[59];
    int mode = data_fp[60];
    int cmd_num = bin8_to_16(&data_fp[61]);
    int err_code = data_fp[63];
    int warn_code = data_fp[64];

    printf("[STATUS] Frame Total: %d bytes | Cmds in Queue: %d\n", total, cmd_num);
    printf("[STATUS] Mode: %d | State: %d | Err: %d | Warn: %d\n", mode, state, err_code, warn_code);
    
    if (err_code != 0) {
        printf("[WARNING] Hardware Error Flag Detected! Error Code: %d\n", err_code);
    }
}

// Extract joint dynamics (currents and voltages) from stream payload
void parse_joint_telemetry(unsigned char *data_fp) {
    float joint_currents[7];
    float joint_voltages[7];

    // Offsets based on UFACTORY Real-time Data Report Protocol
    hex_to_nfp32(&data_fp[71], joint_currents, 7);
    hex_to_nfp32(&data_fp[99], joint_voltages, 7);

    print_nvect("Joint Currents (A): ", joint_currents, 7);
    print_nvect("Joint Voltages (V): ", joint_voltages, 7);
}

// Log streaming frames to a CSV file for post-analysis
void log_telemetry_csv(std::ofstream &log_file, long long timestamp, float *pose, float *angles) {
    if (!log_file.is_open()) return;

    log_file << timestamp;
    for (int i = 0; i < 6; i++) log_file << "," << pose[i];
    for (int i = 0; i < 7; i++) log_file << "," << angles[i];
    log_file << "\n";
}

// Function to run socket reading in a dedicated background thread
void run_report_thread(SocketPort *sock, const std::string &log_filename) {
    std::ofstream log_file(log_filename);
    if (log_file.is_open()) {
        log_file << "timestamp,x,y,z,roll,pitch,yaw,j1,j2,j3,j4,j5,j6,j7\n";
    }

    float report_pose[6];
    float report_angles[7];
    unsigned char buf[256];
    unsigned char *data_fp;

    printf("[STREAM] Background reader thread started...\n");

    while (g_running && sock->is_ok() == 0) {
        if (sock->read_frame(buf) == 0) {
            data_fp = &buf[4];
            long long current_time = get_system_time();

            hex_to_nfp32(&data_fp[7], report_angles, 7);
            hex_to_nfp32(&data_fp[35], report_pose, 6);

            printf("\n=========================================\n");
            printf("Timestamp: %lld ms\n", current_time);
            print_nvect("TCP Pose (mm/deg): ", report_pose, 6);
            print_nvect("Joint Angles (deg): ", report_angles, 7);

            parse_system_status(data_fp);
            parse_joint_telemetry(data_fp);

            log_telemetry_csv(log_file, current_time, report_pose, report_angles);
        } else {
            sleep_milliseconds(1);
        }
    }

    if (log_file.is_open()) log_file.close();
    printf("[STREAM] Report reader thread exiting...\n");
}

int main(int argc, char **argv) {
    if (argc < 3) {
        printf("Usage: %s robot_ip report_port(30001/30002/30003)\n", argv[0]);
        return 0;
    }
    std::string robot_ip(argv[1]);
    int report_port = atoi(argv[2]);

    SocketPort *sock = new SocketPort((char*)robot_ip.c_str(), report_port, 10, 320, 1);
    if (sock->is_ok() != 0) {
        fprintf(stderr, "Error: TCP Report connection failed\n");
        return -1;
    }

    // Launch low-latency telemetry reader in a separate thread
    std::thread stream_thread(run_report_thread, sock, "robot_telemetry.csv");

    // Main thread execution loop (runs concurrently with telemetry stream)
    printf("Press ENTER to terminate streaming...\n");
    std::cin.get();

    // Signal thread cleanup
    g_running = false;
    if (stream_thread.joinable()) {
        stream_thread.join();
    }

    delete sock;
    printf("Socket disconnected and program terminated clean.\n");
    return 0;
}
