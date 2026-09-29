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

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <mstcpip.h>
  #pragma comment(lib, "ws2_32.lib")
#else
  #include <errno.h>
  #include <unistd.h>
  #include <arpa/inet.h>
  #include <net/if.h>
  #include <netinet/tcp.h>
  #include <netinet/in.h>
  #include <sys/ioctl.h>
  #include <sys/socket.h>
  #include <sys/select.h>
#endif

#include "xarm/core/os/network.h"

#if !defined(SOL_TCP) && defined(IPPROTO_TCP)
  #define SOL_TCP IPPROTO_TCP
#endif
#if !defined(TCP_KEEPIDLE) && defined(TCP_KEEPALIVE)
  #define TCP_KEEPIDLE TCP_KEEPALIVE
#endif

#define DB_FLG "[network] "
#define PRINT_ERR printf

// Cross-Platform Error Handling Macros
#ifdef _WIN32
  typedef int socklen_t;
  #define CLOSE_SOCKET(s) closesocket(s)
  #define GET_NET_ERROR() WSAGetLastError()
  #define IS_WOULDBLOCK(err) ((err) == WSAEWOULDBLOCK || (err) == WSAEINPROGRESS)
  #define PERRNO(ret, db_flg, str)                                          \
    {                                                                        \
      if (ret < 0) {                                                         \
        PRINT_ERR("%s%s, error=%d\n", db_flg, str, WSAGetLastError());       \
        return -1;                                                           \
      }                                                                      \
    }
#else
  typedef int SOCKET;
  #define INVALID_SOCKET -1
  #define SOCKET_ERROR -1
  #define CLOSE_SOCKET(s) close(s)
  #define GET_NET_ERROR() errno
  #define IS_WOULDBLOCK(err) ((err) == EWOULDBLOCK || (err) == EAGAIN || (err) == EINPROGRESS)
  #define PERRNO(ret, db_flg, str)                                          \
    {                                                                        \
      if (ret < 0) {                                                         \
        PRINT_ERR("%s%s, errno=%d (%s)\n", db_flg, str, errno, strerror(errno)); \
        return -1;                                                           \
      }                                                                      \
    }
#endif

/**
 * Sets a socket to blocking or non-blocking mode.
 */
static int set_socket_blocking(int sockfd, bool blocking) {
#ifdef _WIN32
  u_long mode = blocking ? 0 : 1;
  return ioctlsocket(sockfd, FIONBIO, &mode);
#else
  int flags = fcntl(sockfd, F_GETFL, 0);
  if (flags == -1) return -1;
  flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
  return fcntl(sockfd, F_SETFL, flags);
#endif
}

/**
 * Initializes socket subsystem and creates/configures TCP socket handle.
 */
int socket_init(char *local_ip, int port, int is_server) {
#ifdef _WIN32
  WORD sockVersion = MAKEWORD(2, 2);
  WSADATA data;
  if (WSAStartup(sockVersion, &data) != 0) {
    PRINT_ERR("%sError: WSAStartup failed, err=%d\n", DB_FLG, WSAGetLastError());
    return -1;
  }
#endif

  int sockfd = static_cast<int>(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
  if (sockfd == INVALID_SOCKET) {
    PERRNO(-1, DB_FLG, "Error creating socket");
  }

  // Socket configurations
  int on = 1;
  int keepAlive = 1;     // Turn on keepalive
  int keepIdle = 5;      // Start probing after 5s idle
  int keepInterval = 2;  // Probe every 2s
  int keepCount = 3;     // Drop after 3 failed probes

  // Configure Socket Reuse & KeepAlive
  int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on));
  PERRNO(ret, DB_FLG, "Error: setsockopt[SO_REUSEADDR]");

  ret = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (const char *)&keepAlive, sizeof(keepAlive));
  PERRNO(ret, DB_FLG, "Error: setsockopt[SO_KEEPALIVE]");

  // Platform-specific Keep-Alive configuration
#ifdef _WIN32
  tcp_keepalive alive_in = {0};
  tcp_keepalive alive_out = {0};
  alive_in.onoff = 1;
  alive_in.keepalivetime = keepIdle * 1000;
  alive_in.keepaliveinterval = keepInterval * 1000;
  DWORD ulBytesReturn = 0;
  ret = WSAIoctl(sockfd, SIO_KEEPALIVE_VALS, &alive_in, sizeof(alive_in),
                 &alive_out, sizeof(alive_out), &ulBytesReturn, NULL, NULL);
  if (ret == SOCKET_ERROR) {
    PERRNO(-1, DB_FLG, "Error: WSAIoctl[SIO_KEEPALIVE_VALS]");
  }
#else
  setsockopt(sockfd, SOL_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
  setsockopt(sockfd, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
  setsockopt(sockfd, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));
#endif

  // Set default send and receive timeouts (2 seconds)
#ifdef _WIN32
  DWORD timeout_ms = 2000;
  setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms));
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout_ms, sizeof(timeout_ms));
#else
  struct timeval timeout = { 2, 0 };
  setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout));
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
#endif

  if (is_server) {
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(port);

    if (local_ip == NULL || strlen(local_ip) == 0) {
      local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
      inet_pton(AF_INET, local_ip, &local_addr.sin_addr);
    }

    ret = bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr));
    PERRNO(ret, DB_FLG, "Error: bind");

    ret = listen(sockfd, 10);
    PERRNO(ret, DB_FLG, "Error: listen");
  }

  return sockfd;
}

/**
 * Non-blocking connect implementation with configurable timeout (in seconds).
 */
int socket_connect_server_timeout(int *sock_ptr, const char server_ip[], int server_port, int timeout_sec) {
  if (!sock_ptr || *sock_ptr < 0) return -1;
  int sockfd = *sock_ptr;

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(server_port);
  if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0) {
    PRINT_ERR("%sInvalid target IP address format: %s\n", DB_FLG, server_ip);
    return -1;
  }

  // Set socket non-blocking for connect timeout logic
  set_socket_blocking(sockfd, false);

  int ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (ret < 0) {
    int err = GET_NET_ERROR();
    if (IS_WOULDBLOCK(err)) {
      fd_set write_fds;
      FD_ZERO(&write_fds);
      FD_SET(sockfd, &write_fds);

      struct timeval tv;
      tv.tv_sec = timeout_sec;
      tv.tv_usec = 0;

      ret = select(sockfd + 1, NULL, &write_fds, NULL, &tv);
      if (ret > 0) {
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char *)&so_error, &len);
        if (so_error == 0) {
          ret = 0; // Connection successful
        } else {
          PRINT_ERR("%sConnection failed with SO_ERROR: %d\n", DB_FLG, so_error);
          ret = -1;
        }
      } else {
        PRINT_ERR("%sConnection timed out to %s:%d\n", DB_FLG, server_ip, server_port);
        ret = -1;
      }
    } else {
      PERRNO(-1, DB_FLG, "Error: connect failed immediately");
    }
  }

  // Restore socket to blocking mode
  set_socket_blocking(sockfd, true);
  return ret;
}

int socket_connect_server(int *socket, char server_ip[], int server_port) {
  return socket_connect_server_timeout(socket, server_ip, server_port, 3);
}

/**
 * Guarantees sending all requested byte length across network fragmentation limits.
 */
int socket_send_data_all(int client_fp, const unsigned char *data, int len) {
  int total_sent = 0;
  while (total_sent < len) {
    int sent = send(client_fp, (const char *)(data + total_sent), len - total_sent, 0);
    if (sent <= 0) {
      int err = GET_NET_ERROR();
      if (IS_WOULDBLOCK(err)) {
        continue;
      }
      PERRNO(-1, DB_FLG, "Error: socket_send_data_all failed");
    }
    total_sent += sent;
  }
  return total_sent;
}

int socket_send_data(int client_fp, unsigned char *data, int len) {
  return socket_send_data_all(client_fp, data, len);
}

/**
 * Reads data into the buffer up to 'max_len' bytes.
 */
int socket_read_data(int client_fp, unsigned char *buf, int max_len) {
  int ret = recv(client_fp, (char *)buf, max_len, 0);
  if (ret < 0) {
    int err = GET_NET_ERROR();
    if (IS_WOULDBLOCK(err)) {
      return 0; // Timeout without data
    }
    PERRNO(-1, DB_FLG, "Error: socket_read_data");
  }
  return ret; // Returns bytes read (0 indicates standard TCP disconnect)
}

/**
 * Gracefully shuts down and closes an active socket connection.
 */
void socket_close(int *sockfd) {
  if (sockfd && *sockfd >= 0) {
#ifdef _WIN32
    shutdown(*sockfd, SD_BOTH);
    closesocket(*sockfd);
#else
    shutdown(*sockfd, SHUT_RDWR);
    close(*sockfd);
#endif
    *sockfd = -1;
  }
}
