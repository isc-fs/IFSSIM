#include "udp_receiver.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

UdpReceiver::UdpReceiver() {}

UdpReceiver::~UdpReceiver()
{
    stop();
}

void UdpReceiver::start(int sensor_port, int lidar_port)
{
    if (running_) return;
    running_ = true;

    sensor_thread_ = std::thread(&UdpReceiver::sensorListenerThread, this, sensor_port);
    lidar_thread_ = std::thread(&UdpReceiver::lidarListenerThread, this, lidar_port);
}

void UdpReceiver::stop()
{
    running_ = false;
    if (sensor_thread_.joinable()) sensor_thread_.join();
    if (lidar_thread_.joinable()) lidar_thread_.join();
}

void UdpReceiver::sensorListenerThread(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "IFSSIM UDP: Failed to create sensor socket" << std::endl;
        return;
    }

    // Allow reuse and set receive timeout
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Increase receive buffer for high-frequency data
    int rcvbuf = 1024 * 1024; // 1MB
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "IFSSIM UDP: Failed to bind sensor port " << port << std::endl;
        close(sock);
        return;
    }

    std::cout << "IFSSIM UDP: Listening for sensors on port " << port << std::endl;

    while (running_) {
        SensorFrame frame;
        ssize_t n = recv(sock, &frame, sizeof(frame), 0);

        if (n == sizeof(frame) && frame.magic == SENSOR_MAGIC) {
            if (sensor_cb_) {
                sensor_cb_(frame);
            }
        }
        // Timeout or wrong size — just loop
    }

    close(sock);
}

void UdpReceiver::lidarListenerThread(int port)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "IFSSIM UDP: Failed to create LiDAR socket" << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Large receive buffer for point cloud chunks
    int rcvbuf = 4 * 1024 * 1024; // 4MB
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "IFSSIM UDP: Failed to bind LiDAR port " << port << std::endl;
        close(sock);
        return;
    }

    std::cout << "IFSSIM UDP: Listening for LiDAR on port " << port << std::endl;

    // Max chunk: header + 5000 points * 3 floats * 4 bytes = ~60KB
    std::vector<uint8_t> buffer(65536);

    while (running_) {
        ssize_t n = recv(sock, buffer.data(), buffer.size(), 0);

        if (n < (ssize_t)sizeof(LidarChunkHeader)) continue;

        LidarChunkHeader* header = (LidarChunkHeader*)buffer.data();
        if (header->magic != LIDAR_MAGIC) continue;

        int data_offset = sizeof(LidarChunkHeader);
        int expected_data = header->points_in_chunk * 3 * sizeof(float);
        if (n < data_offset + expected_data) continue;

        // New frame? Reset
        if (header->frame_id != pending_lidar_.frame_id || header->chunk_index == 0) {
            pending_lidar_.frame_id = header->frame_id;
            pending_lidar_.total_points = header->total_points;
            pending_lidar_.channels = header->channels;
            pending_lidar_.total_chunks = header->total_chunks;
            pending_lidar_.chunks_received = 0;
            pending_lidar_.points.resize(header->total_points * 3);
        }

        // Copy chunk data into correct position
        int point_offset = header->chunk_index * 5000 * 3; // 5000 points per chunk max
        float* src = (float*)(buffer.data() + data_offset);
        int floats_count = header->points_in_chunk * 3;

        if (point_offset + floats_count <= (int)pending_lidar_.points.size()) {
            memcpy(pending_lidar_.points.data() + point_offset, src, floats_count * sizeof(float));
        }

        pending_lidar_.chunks_received++;

        // All chunks received? Deliver
        if (pending_lidar_.chunks_received >= pending_lidar_.total_chunks) {
            if (lidar_cb_) {
                lidar_cb_(pending_lidar_.total_points, pending_lidar_.channels,
                         pending_lidar_.points);
            }
        }
    }

    close(sock);
}
