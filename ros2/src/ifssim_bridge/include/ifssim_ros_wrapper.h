#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fs_msgs/msg/control_command.hpp>
#include <fs_msgs/msg/go_signal.hpp>
#include <fs_msgs/msg/finished_signal.hpp>
#include <fs_msgs/msg/track.hpp>
#include <fs_msgs/msg/extra_info.hpp>
#include <fs_msgs/srv/reset.hpp>

#include "tcp_client.h"
#include "udp_receiver.h"  // For frame struct definitions

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * IFSSIM ROS2 Wrapper — TCP push model.
 *
 * Architecture (4 TCP connections):
 *   1. Sensor stream (streamSensors) → IMU at 400Hz, GSS/TF/Odom at 100Hz, GPS at 10Hz
 *   2. LiDAR stream (streamLidar)   → PointCloud2 at ~10Hz
 *   3. Camera client (TCP req/resp)  → CompressedImage at 10Hz
 *   4. Command client (TCP req/resp) → control commands, settings, referee queries
 */
class IFSSIMRosWrapper
{
public:
    IFSSIMRosWrapper(
        std::shared_ptr<rclcpp::Node> node,
        const std::string& host,
        int port,
        double timeout_sec);

    ~IFSSIMRosWrapper();

private:
    void initializeConnection();
    void initializePublishers();
    void initializeSubscribers();
    void initializeTimers();
    void startStreaming();
    void triggerReconnect();  // Called by stream threads on disconnect

    // Streaming threads
    void sensorStreamThread();
    void lidarStreamThread();

    // Stream data handlers
    void onSensorFrame(const SensorFrame& frame);
    void onLidarFrame(const LidarChunkHeader& header, const float* points);

    // Timer callbacks (TCP command client, low frequency)
    void cameraTimerCb();
    void goSignalTimerCb();
    void extraInfoTimerCb();
    void trackPublishCb();
    void staticTfCb();

    // Subscriber callbacks
    void controlCommandCb(const fs_msgs::msg::ControlCommand::SharedPtr msg);
    void resetSrvCb(
        const std::shared_ptr<fs_msgs::srv::Reset::Request> request,
        std::shared_ptr<fs_msgs::srv::Reset::Response> response);

    // Node
    std::shared_ptr<rclcpp::Node> node_;

    // TCP clients
    std::unique_ptr<TcpClient> client_;          // Commands + referee queries
    std::unique_ptr<TcpClient> client_camera_;   // Camera image requests

    // Streaming sockets (raw, not TcpClient — held open)
    std::atomic<int> sensor_stream_fd_{-1};
    std::atomic<int> lidar_stream_fd_{-1};
    std::thread sensor_thread_;
    std::thread lidar_thread_;
    std::atomic<bool> streaming_{false};
    std::mutex reconnect_mutex_;  // Ensures only one thread reconnects at a time

    // Connection params
    std::string host_;
    int port_;
    double timeout_sec_;

    // Frame IDs — match original FSDS simulator convention
    std::string map_frame_id_ = "odom";
    std::string vehicle_frame_id_ = "fsds/FSCar";

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gss_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> camera_pubs_;
    rclcpp::Publisher<fs_msgs::msg::GoSignal>::SharedPtr go_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr finished_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_pub_;
    rclcpp::Publisher<fs_msgs::msg::Track>::SharedPtr track_pub_;

    // Transient: previous referee.finished value, for edge-triggered publish
    bool last_finished_state_ = false;

    // Subscribers
    rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr control_cmd_sub_;
    rclcpp::Service<fs_msgs::srv::Reset>::SharedPtr reset_srv_;

    // TF
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::TimerBase::SharedPtr go_signal_timer_;
    rclcpp::TimerBase::SharedPtr extra_info_timer_;
    rclcpp::TimerBase::SharedPtr track_publish_timer_;
    rclcpp::TimerBase::SharedPtr static_tf_timer_;

    // Config
    std::string mission_name_ = "trackdrive";
    std::string track_name_ = "A";
    bool competition_mode_ = false;
    std::vector<std::string> camera_names_;

    // Per-sensor publish rates (divisors of the 400Hz sensor stream):
    //   IMU  → publish every frame    (400 Hz)
    //   GSS / TF / Odom → every 4    (100 Hz)
    //   GPS  → every 40              ( 10 Hz)
    uint64_t sensor_frame_count_ = 0;

    // Noise params
    double gps_position_noise_std_ = 0.0;
    double imu_accel_noise_std_ = 0.0;
    double imu_gyro_noise_std_ = 0.0;
    double gss_velocity_noise_std_ = 0.0;

    void parseNoiseSettings(const std::string& settings_json);

    // Helper: open a raw TCP socket and send a command
    int openStreamSocket(const std::string& command);
};
