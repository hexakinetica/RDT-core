#ifndef UDPPEER_HPP
#define UDPPEER_HPP

#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using socket_t = SOCKET;
inline void init_sockets() {
    static const bool initialized = []() {
        WSADATA wsaData;
        return WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;
    }();
}
inline void close_socket(socket_t s) { closesocket(s); }
#else
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
using socket_t = int;
inline void init_sockets() {}
inline void close_socket(socket_t s) { close(s); }
#endif

namespace RDT { 

struct NetworkConfig {
    uint16_t local_port;
    std::string remote_ip;
    uint16_t remote_port;
    std::size_t max_packet_size = 4096;
    int receive_timeout_ms = 100; // default 100ms timeout
};

class UdpPeer {
public:
    explicit UdpPeer(NetworkConfig config)
        : _config(std::move(config)), _sock(-1) {
        init_sockets();
    }

    ~UdpPeer() {
        disconnect();
    }

    [[nodiscard]] int connect() {
        if (_sock >= 0) return 0;

        _sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (_sock < 0) return -1;

        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(_config.local_port);
        local_addr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(_sock, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) < 0)
            return -2;

        timeval tv{};
        tv.tv_sec = _config.receive_timeout_ms / 1000;
        tv.tv_usec = (_config.receive_timeout_ms % 1000) * 1000;
        if (setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO,
                       reinterpret_cast<const char*>(&tv), sizeof(tv)) < 0)
            return -3;

        _remote_addr = sockaddr_in{};
        _remote_addr.sin_family = AF_INET;
        _remote_addr.sin_port = htons(_config.remote_port);
        if (inet_pton(AF_INET, _config.remote_ip.c_str(), &_remote_addr.sin_addr) <= 0)
            return -4;

        return 0;
    }

    void disconnect() {
        if (_sock >= 0) {
            close_socket(_sock);
            _sock = -1;
        }
    }

    [[nodiscard]] int send(const std::vector<char>& data) const {
        if (_sock < 0) return -1;
        if (data.size() > _config.max_packet_size) return -2;

        const int sent = ::sendto(_sock, data.data(), static_cast<int>(data.size()), 0,
                                  reinterpret_cast<const sockaddr*>(&_remote_addr),
                                  sizeof(_remote_addr));
        return (sent < 0) ? -3 : sent;
    }

    [[nodiscard]] int receive(std::vector<char>& out) const {
        if (_sock < 0) return -1;

        out.resize(_config.max_packet_size);
        sockaddr_in sender{};
        socklen_t sender_len = sizeof(sender);

        const int received = ::recvfrom(_sock, out.data(), static_cast<int>(out.size()), 0,
                                        reinterpret_cast<sockaddr*>(&sender), &sender_len);
        if (received < 0) return -2;

        out.resize(received);
        return received;
    }

private:
    NetworkConfig _config;
    socket_t _sock;
    sockaddr_in _remote_addr{};
};

}
#endif // UDPPEER_HPP
