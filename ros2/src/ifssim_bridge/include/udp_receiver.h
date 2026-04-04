#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <thread>
#include <atomic>

/**
 * Binary frame structs — must match UE5 FSDSUdpBroadcaster.h exactly.
 */
#pragma pack(push, 1)

struct SensorFrame
{
    uint32_t magic;
    uint32_t frame_id;
    uint64_t timestamp;

    // GPS
    double latitude, longitude;
    float altitude;

    // IMU (body frame)
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float orient_x, orient_y, orient_z, orient_w;

    // GSS (body frame, m/s)
    float gss_vx, gss_vy, gss_vz;

    // Odom/Pose (ENU meters)
    float pos_x, pos_y, pos_z;
    float pose_qx, pose_qy, pose_qz, pose_qw;
    float speed, rpm;

    // Referee
    int32_t doo_counter, oc_counter, lap_count;

    // Controls echo
    float throttle, steering, brake;
};

struct LidarChunkHeader
{
    uint32_t magic;
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint32_t frame_id;
    int32_t points_in_chunk;
    int32_t total_points;
    int32_t channels;
};

#pragma pack(pop)

static constexpr uint32_t SENSOR_MAGIC = 0x49465353; // "IFSS"
static constexpr uint32_t LIDAR_MAGIC  = 0x4C494452; // "LIDR"

/**
 * UDP Receiver — listens for sensor and LiDAR broadcasts from the IFSSIM UE5 plugin.
 * Runs two listener threads (one per port).
 */
class UdpReceiver
{
public:
    using SensorCallback = std::function<void(const SensorFrame&)>;
    using LidarCallback = std::function<void(int32_t total_points, int32_t channels,
                                              const std::vector<float>& points)>;

    UdpReceiver();
    ~UdpReceiver();

    void start(int sensor_port = 41452, int lidar_port = 41453);
    void stop();

    void setSensorCallback(SensorCallback cb) { sensor_cb_ = cb; }
    void setLidarCallback(LidarCallback cb) { lidar_cb_ = cb; }

    bool isRunning() const { return running_; }

private:
    void sensorListenerThread(int port);
    void lidarListenerThread(int port);

    SensorCallback sensor_cb_;
    LidarCallback lidar_cb_;

    std::thread sensor_thread_;
    std::thread lidar_thread_;
    std::atomic<bool> running_{false};

    // LiDAR chunk reassembly
    struct LidarFrame {
        uint32_t frame_id = 0;
        int32_t total_points = 0;
        int32_t channels = 0;
        int32_t total_chunks = 0;
        int32_t chunks_received = 0;
        std::vector<float> points;
    };
    LidarFrame pending_lidar_;
};
