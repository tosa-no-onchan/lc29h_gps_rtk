// MIT License
//
// Copyright (c) 2021 Yuming Meng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ntrip/ntrip_client.h"

#if defined(__linux__)
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#endif  // defined(__linux__)
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <list>
#include <memory>

#include "ntrip/ntrip_util.h"
#include "cmake_definition.h"

// add by nishi
#include <iostream>

#if defined(__linux__)
// #define ENABLE_TCP_KEEPALIVE
#endif // defined(__linux__)

namespace libntrip {

namespace {

using socket_t = decltype(socket(AF_INET, SOCK_STREAM, 0));

// GPGGA format example.
constexpr char gpgga_buffer[] =
    "$GPGGA,083552.00,3000.0000000,N,11900.0000000,E,"
    "1,08,1.0,0.000,M,100.000,M,,*57\r\n";

constexpr int kBufferSize = 4096;
constexpr int kReceiveTimeoutPeriod = 3;

}  // namespace

//
// Public method.
//

bool NtripClient::Run(void) {
  if (service_is_running_.load()) return true;
  Stop();
  socket_t socket_fd;
  // Establish a connection with NtripCaster.
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(server_port_);
  server_addr.sin_addr.s_addr = inet_addr(server_ip_.c_str());
  #if defined(WIN32) || defined(_WIN32)
    WSADATA ws_data;
    if (WSAStartup(MAKEWORD(2,2), &ws_data) != 0) {
      return false;
    }
    socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd == INVALID_SOCKET) {
      printf("Create socket failed!\r\n");
      WSACleanup();
      return false;
    }
    server_addr.sin_addr.S_un.S_addr = inet_addr(server_ip_.c_str());
  #else
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1) {
      //printf("Create socket failed, errno = -%d\r\n", errno);
      std::cout << "Create socket failed, errno"<< errno << std::endl;
      return false;
    }
  #endif  // defined(WIN32) || defined(_WIN32)
  if (connect(socket_fd, reinterpret_cast<struct sockaddr *>(&server_addr),
      sizeof(server_addr)) < 0) {
        //printf("Connect to NtripCaster[%s:%d] failed, errno = -%d\r\n",
        //server_ip_.c_str(), server_port_, errno);
        std::cout << "Connect to NtripCaster[" << server_ip_.c_str() << ":" << server_port_;
        std::cout << "] failed, errno =" << errno << std::endl;

    #if defined(WIN32) || defined(_WIN32)
        closesocket(socket_fd);
        WSACleanup();
    #else
        close(socket_fd);
    #endif  // defined(WIN32) || defined(_WIN32)
    return false;
  }
  // Set non-blocking.
  #if defined(WIN32) || defined(_WIN32)
    unsigned long ul = 1;
    if (ioctlsocket(socket_fd, FIONBIO, &ul) == SOCKET_ERROR) {
      closesocket(socket_fd);
      WSACleanup();
      return false;
    }
  #else
    int flags = fcntl(socket_fd, F_GETFL);
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
  #endif  // defined(WIN32) || defined(_WIN32)

  // Ntrip connection authentication.
  int ret = -1;
  std::string user_passwd = user_ + ":" + passwd_;
  std::string user_passwd_base64;
  std::unique_ptr<char[]> buffer(
      new char[kBufferSize], std::default_delete<char[]>());
  // Generate base64 encoding of username and password.
  Base64Encode(user_passwd, &user_passwd_base64);
  // Generate request data format of ntrip.
  ret = snprintf(buffer.get(), kBufferSize-1,
      "GET /%s HTTP/1.1\r\n"
      "User-Agent: %s\r\n"
      "Authorization: Basic %s\r\n"
      "\r\n",
      mountpoint_.c_str(), kClientAgent, user_passwd_base64.c_str());

  // test by nishi 2024.4.14
  std::cout << ">> header:" <<buffer.get() << std::endl;

  if (send(socket_fd, buffer.get(), ret, 0) < 0) {
    //printf("Send request failed!!!\r\n");
    std::cout << "Send request failed!!!"<< std::endl;

    #if defined(WIN32) || defined(_WIN32)
      closesocket(socket_fd);
      WSACleanup();
    #else
      close(socket_fd);
    #endif  // defined(WIN32) || defined(_WIN32)

    return false;
  }
  // Waitting for request to connect caster success.
  int timeout = 30;  // 30*100ms=3s.
  while (timeout--) {
    ret = recv(socket_fd, buffer.get(), kBufferSize, 0);
    if (ret > 0) {
      std::string result(buffer.get(), ret);
      if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
          (result.find("ICY 200 OK") != std::string::npos)) 
      {

        // test by nishi 2024.4.14
        std::cout << ">> receve response HTTP/1.1 200 OK or ICY 200 OK" << std::endl;

        // ここは、GNSS デバイスから、実際に GPGGA レコードを受信しないといけない。 by nishi 2024.4.14
        if (gga_buffer_.empty()) {
          GGAFrameGenerate(latitude_, longitude_, 10.0, &gga_buffer_);
        }
        ret = send(socket_fd, gga_buffer_.c_str(), gga_buffer_.size(), 0);
        if (ret < 0) {
          //printf("Send gpgga data fail\r\n");
          std::cout << "Send gpgga data fail"<< std::endl;
          #if defined(WIN32) || defined(_WIN32)
            closesocket(socket_fd);
            WSACleanup();
          #else
            close(socket_fd);
          #endif  // defined(WIN32) || defined(_WIN32)
          return false;
        }
        // test by nishi 2024.4.14
        std::cout << ">> send my GPGGA data to server" << std::endl;

        break;
      } 
      else {
        //printf("Request result: %s\r\n", result.c_str());
        std::cout << "Request result:"<< result.c_str() << std::endl;
      }
    } 
    else if (ret == 0) {
      //printf("Remote socket close!!!\r\n");
      std::cout << "Remote socket close!!!"<< std::endl;
      #if defined(WIN32) || defined(_WIN32)
        closesocket(socket_fd);
        WSACleanup();
      #else
        close(socket_fd);
      #endif  // defined(WIN32) || defined(_WIN32)
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (timeout <= 0) {
    //printf("NtripCaster[%s:%d %s %s %s] access failed!!!\r\n",
    //    server_ip_.c_str(), server_port_,
    //    user_.c_str(), passwd_.c_str(), mountpoint_.c_str());

    std::cout << "NtripCaster[" << server_ip_.c_str() << ":"<< server_port_<< " ";
    std::cout << user_.c_str() << " "<<passwd_.c_str()<<" "<< mountpoint_.c_str()<< "]" << std::endl;


    #if defined(WIN32) || defined(_WIN32)
      closesocket(socket_fd);
      WSACleanup();
    #else
      close(socket_fd);
    #endif  // defined(WIN32) || defined(_WIN32)
    return false;
  }
  // TCP socket keepalive.
  #if defined(ENABLE_TCP_KEEPALIVE)
    int keepalive = 1;  // Enable keepalive attributes.
    int keepidle = 30;  // Time out for starting detection.
    int keepinterval = 5;  // Time interval for sending packets during detection.
    int keepcount = 3;  // Max times for sending packets during detection.
    setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE,
        &keepalive, sizeof(keepalive));
    setsockopt(socket_fd, SOL_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(socket_fd, SOL_TCP, TCP_KEEPINTVL,
        &keepinterval, sizeof(keepinterval));
    setsockopt(socket_fd, SOL_TCP, TCP_KEEPCNT, &keepcount, sizeof(keepcount));
  #endif  // defined(ENABLE_TCP_KEEPALIVE)
  socket_fd_ = socket_fd;

  thread_.reset(&NtripClient::ThreadHandler, this);

  return true;
}

void NtripClient::Stop(void) {
  service_is_running_.store(false);
  #if defined(WIN32) || defined(_WIN32)
    if (socket_fd_ != INVALID_SOCKET) {
      closesocket(socket_fd_);
      WSACleanup();
      socket_fd_ = INVALID_SOCKET;
    }
  #else
    if (socket_fd_ > 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  #endif  // defined(WIN32) || defined(_WIN32)
  thread_.join();
}

//
// Private method.
//
// https://cpprefjp.github.io/reference/chrono/steady_clock.html
// https://jp-seemore.com/iot/26755/
// https://programdoria.com/programming/cpp/milliseconds_today/
//
void NtripClient::ThreadHandler(void) {
  //std::cout << ">> start NtripClient::ThreadHandler()" << std::endl;

  service_is_running_.store(true);
  int ret;
  std::unique_ptr<char[]> buffer(
      new char[kBufferSize], std::default_delete<char[]>());

  auto tp_beg = std::chrono::steady_clock::now();
  auto tp_end = tp_beg;
  auto tp_rcv_beg = tp_beg;
  auto tp_rcv_end = tp_beg;
  int intv_ms = report_interval_ * 1000;
  //auto intv_ms = std::chrono::milliseconds(report_interval_ * 1000);
  //auto intv_req_ms = std::chrono::milliseconds(intv_recv_);
  int receive_timeout_cnt = kReceiveTimeoutPeriod;

  //printf("NtripClient service running...\r\n");
  std::cout << "NtripClient service running..." << std::endl;
  while (service_is_running_.load()) {
    tp_rcv_end = std::chrono::steady_clock::now();
    // intv_recv_[ms] 間隔で、Server に request する。
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        tp_rcv_end-tp_rcv_beg).count() >= intv_recv_) {

      //std::cout << " recv()"<< std::endl;     
      ret = recv(socket_fd_, buffer.get(), kBufferSize, 0);
      //std::cout << " recv() ret="<< ret << std::endl;
      if (ret == 0) {
        //printf("Remote socket close!!!\r\n");
        std::cout << "Remote socket close!!!"<< std::endl;
        break;
      } 
      else if (ret < 0) {
        if ((errno != 0) && (errno != EAGAIN) &&
            (errno != EWOULDBLOCK) && (errno != EINTR)) {
          //printf("Remote socket error, errno=%d\r\n", errno);
          std::cout << "Remote socket error, errno="<< errno << std::endl;
          break;
        }
      } 
      else {
        receive_timeout_cnt = kReceiveTimeoutPeriod;
        callback_(buffer.get(), ret);
        if (ret == kBufferSize) continue;
      }
      tp_rcv_beg=std::chrono::steady_clock::now();
    }
    tp_end = std::chrono::steady_clock::now();
    // report_interval_[ms] 間隔で、Server に request する。
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        tp_end-tp_beg).count() >= intv_ms) {
      // 連続して time out 3 回で、終了みたい。 -> これは、将来はどうか? by nishi 2024.9.14
      if (receive_timeout_cnt-- <= 0) 
        break;
      tp_beg = std::chrono::steady_clock::now();
      if (!gga_is_update_.load()) {
        GGAFrameGenerate(latitude_, longitude_, 10.0, &gga_buffer_);
      }
      std::cout << " send()"<< std::endl;     
      send(socket_fd_, gga_buffer_.c_str(), gga_buffer_.size(), 0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  #if defined(WIN32) || defined(_WIN32)
    if (socket_fd_ != INVALID_SOCKET) {
      closesocket(socket_fd_);
      WSACleanup();
      socket_fd_ = INVALID_SOCKET;
    }
  #else
    if (socket_fd_ > 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  #endif  // defined(WIN32) | defined(_WIN32)
  //printf("NtripClient service done.\r\n");
  std::cout << "NtripClient service done."<< std::endl;
  service_is_running_.store(false);
}

}  // namespace libntrip
