/**
 * Software License Agreement (MIT License)
 * 
 * Copyright (c) 2022, UFACTORY, Inc.
 * 
 * All rights reserved.
 * 
 * @author Zhang <jimy92@163.com>
 * @author Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 */

#include "xarm/core/connect.h"

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/instruction/uxbus_cmd_ser.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"
#include "xarm/core/xarm_config.h"

#define DB_FLG "[xArm Connect] "

/**
 * Establishes an RS485 serial connection to the arm controller.
 */
UxbusCmdSer *connect_rs485_control(const char *com) {
  if (com == nullptr || strlen(com) == 0) {
    fprintf(stderr, "%sError: Invalid serial port name provided.\n", DB_FLG);
    return nullptr;
  }

  // Allocate port safely
  std::unique_ptr<SerialPort> arm_port(new SerialPort(com, XARM_CONF::SERIAL_BAUD, 3, 128));

  if (arm_port->is_ok() != 0) {
    fprintf(stderr, "%sError: Serial RS485 connection failed on port: %s\n", DB_FLG, com);
    return nullptr;
  }

  // Transfer ownership of arm_port to UxbusCmdSer
  UxbusCmdSer *arm_cmd = new UxbusCmdSer(arm_port.release());
  printf("%sSerial RS485 connection successful on port: %s\n", DB_FLG, com);
  return arm_cmd;
}

/**
 * Establishes a TCP control connection to the xArm controller.
 */
UxbusCmdTcp *connect_tcp_control(const char *server_ip) {
  if (server_ip == nullptr || strlen(server_ip) == 0) {
    fprintf(stderr, "%sError: Invalid target IP address provided.\n", DB_FLG);
    return nullptr;
  }

  // Allocate SocketPort safely
  std::unique_ptr<SocketPort> arm_port(
      new SocketPort(const_cast<char*>(server_ip), XARM_CONF::TCP_PORT_CONTROL, 3, 320, 0));

  if (arm_port->is_ok() != 0) {
    fprintf(stderr, "%sError: TCP Control connection failed to IP: %s\n", DB_FLG, server_ip);
    return nullptr;
  }

  UxbusCmdTcp *arm_cmd = new UxbusCmdTcp(arm_port.release());
  printf("%sTCP Control connection successful to IP: %s\n", DB_FLG, server_ip);
  return arm_cmd;
}

/**
 * Connects with automatic retry attempts on connection drops or boot-up timing delays.
 */
UxbusCmdTcp *connect_tcp_control_retry(const char *server_ip, int max_retries, int delay_ms) {
  UxbusCmdTcp *arm_cmd = nullptr;
  int attempt = 0;

  while (attempt < max_retries) {
    attempt++;
    arm_cmd = connect_tcp_control(server_ip);
    if (arm_cmd != nullptr) {
      return arm_cmd;
    }

    if (attempt < max_retries) {
      printf("%sRetrying TCP Control connection to %s (%d/%d) in %d ms...\n",
             DB_FLG, server_ip, attempt, max_retries, delay_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
  }

  fprintf(stderr, "%sFailed to connect to %s after %d retries.\n", DB_FLG, server_ip, max_retries);
  return nullptr;
}

/**
 * Establishes Standard Real-Time Data Report socket (Port 20001).
 */
SocketPort *connect_tcp_report_norm(const char *server_ip) {
  if (!server_ip) return nullptr;

  SocketPort *arm_report =
      new SocketPort(const_cast<char*>(server_ip), XARM_CONF::TCP_PORT_REPORT_NORM, 5, 256 + 4, 1);

  if (arm_report->is_ok() != 0) {
    fprintf(stderr, "%sError: TCP Report Norm connection failed, IP: %s\n", DB_FLG, server_ip);
    delete arm_report;
    return nullptr;
  }
  printf("%sTCP Report Norm connection successful to IP: %s\n", DB_FLG, server_ip);
  return arm_report;
}

/**
 * Establishes Rich Data Report socket (Port 20002).
 */
SocketPort *connect_tcp_report_rich(const char *server_ip) {
  if (!server_ip) return nullptr;

  SocketPort *arm_report =
      new SocketPort(const_cast<char*>(server_ip), XARM_CONF::TCP_PORT_REPORT_RICH, 5, 1024, 1);

  if (arm_report->is_ok() != 0) {
    fprintf(stderr, "%sError: TCP Report Rich connection failed, IP: %s\n", DB_FLG, server_ip);
    delete arm_report;
    return nullptr;
  }
  printf("%sTCP Report Rich connection successful to IP: %s\n", DB_FLG, server_ip);
  return arm_report;
}

/**
 * Establishes Developer / Debug Data Report socket (Port 20003).
 */
SocketPort *connect_tcp_report_devl(const char *server_ip) {
  if (!server_ip) return nullptr;

  SocketPort *arm_report =
      new SocketPort(const_cast<char*>(server_ip), XARM_CONF::TCP_PORT_REPORT_DEVL, 10, 256 + 4, 1);

  if (arm_report->is_ok() != 0) {
    fprintf(stderr, "%sError: TCP Report Develop connection failed, IP: %s\n", DB_FLG, server_ip);
    delete arm_report;
    return nullptr;
  }
  printf("%sTCP Report Develop connection successful to IP: %s\n", DB_FLG, server_ip);
  return arm_report;
}

/**
 * Polymorphic route selector based on string identifier ("norm", "rich", "dev").
 */
SocketPort *connect_tcp_report(const char *server_ip, const std::string &report_type) {
  if (report_type == "dev" || report_type == "devl") {
    return connect_tcp_report_devl(server_ip);
  } else if (report_type == "rich") {
    return connect_tcp_report_rich(server_ip);
  } else {
    return connect_tcp_report_norm(server_ip);
  }
}

/**
 * Helper utility to safely release TCP control handles.
 */
void disconnect_tcp_control(UxbusCmdTcp **arm_cmd) {
  if (arm_cmd && *arm_cmd) {
    delete *arm_cmd;
    *arm_cmd = nullptr;
    printf("%sTCP Control connection disconnected safely.\n", DB_FLG);
  }
}

/**
 * Helper utility to safely release Report sockets.
 */
void disconnect_tcp_report(SocketPort **arm_report) {
  if (arm_report && *arm_report) {
    delete *arm_report;
    *arm_report = nullptr;
    printf("%sTCP Report socket closed safely.\n", DB_FLG);
  }
}
