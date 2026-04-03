#pragma once

#include <string>
#include <mutex>

/**
 * Simple TCP client for connecting to the IFSSIM RPC server.
 * Protocol: send "command [args]\n", receive "json_response\n"
 */
class TcpClient
{
public:
    TcpClient();
    ~TcpClient();

    bool connect(const std::string& host, int port, double timeout_sec = 5.0);
    void disconnect();
    bool isConnected() const;

    /** Send a command and receive the response */
    std::string sendCommand(const std::string& command);

    /** Convenience: send and parse a simple boolean response */
    bool sendBool(const std::string& command);

    /** Convenience: send and parse a float response from JSON */
    double parseDouble(const std::string& json, const std::string& key);

private:
    int socket_fd_ = -1;
    bool connected_ = false;
    std::mutex mutex_;
};
