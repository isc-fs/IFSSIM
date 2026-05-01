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
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "tcp_client.h"
#include "udp_receiver.h"  // For frame struct definitions

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
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
    void sensorPublishThread();  // Drains the single-slot buffer, calls onSensorFrame
    void lidarStreamThread();
    void lidarPublishThread();   // Drains the single-slot buffer, calls onLidarFrame

    // Stream data handlers
    void onSensorFrame(const SensorFrame& frame);
    void onLidarFrame(const LidarChunkHeader& header, const float* points);

    // Timer callbacks (TCP command client, low frequency)
    void cameraTimerCb();
    void goSignalTimerCb();
    void extraInfoTimerCb();
    void trackPublishCb();
    void staticTfCb();
    void tireLoadsTimerCb();

    // Subscriber callbacks
    void controlCommandCb(const fs_msgs::msg::ControlCommand::SharedPtr msg);
    void ebsRequestCb(const std_msgs::msg::Empty::SharedPtr msg);
    void ebsResetCb(const std_msgs::msg::Empty::SharedPtr msg);
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

    // Sensor producer-consumer split — same pattern as the LiDAR split
    // below. Before this, the 400 Hz sensor recv thread published IMU
    // (400 Hz), GSS/TF/Odom (100 Hz), GPS (10 Hz) inline, so any DDS
    // backpressure (slow subscriber, full RELIABLE buffer) blocked the
    // recv loop and the kernel TCP buffer filled within ~milliseconds.
    // Plugin's SendAll then hit its 1 s timeout, tore the stream down,
    // and ALL sensor topics fell to ~0 Hz under pipeline load. Symptom
    // captured: with pipeline running, /imu and /gps each reported
    // "topic does not appear to be published" for 10 s. Move publishes
    // to a dedicated thread; recv thread now only pushes the latest
    // SensorFrame into a single-slot buffer and immediately re-enters
    // recv. Frame drops are acceptable: at 400 Hz a missed sample is
    // 2.5 ms of IMU.
    std::mutex sensor_pub_mutex_;
    std::condition_variable sensor_pub_cv_;
    std::optional<SensorFrame> sensor_pending_;
    std::thread sensor_pub_thread_;

    // LiDAR producer-consumer split. The recv thread used to call
    // publish() inline, which under sustained pipeline-subscriber load
    // would block long enough for the kernel TCP recv buffer to fill ⇒
    // plugin's send buffer fills ⇒ plugin's SendAll hits its 1 s timeout
    // ⇒ stream tear-down ⇒ reconnect cascade. By moving publish() to a
    // dedicated thread the recv thread never blocks on ROS work — it
    // pushes the latest frame into a single-slot buffer (dropping any
    // unconsumed older frame) and immediately re-enters recv. Drops are
    // intentional: LiDAR is a streaming firehose, latency matters more
    // than every-frame delivery.
    struct PendingLidarFrame {
        LidarChunkHeader header;
        std::vector<float> points;
    };
    std::mutex lidar_pub_mutex_;
    std::condition_variable lidar_pub_cv_;
    std::optional<PendingLidarFrame> lidar_pending_;
    std::thread lidar_pub_thread_;

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
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tire_loads_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> camera_pubs_;
    rclcpp::Publisher<fs_msgs::msg::GoSignal>::SharedPtr go_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr finished_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_pub_;
    rclcpp::Publisher<fs_msgs::msg::Track>::SharedPtr track_pub_;

    // Transient: previous referee.finished value, for edge-triggered publish
    bool last_finished_state_ = false;

    // Transient: previous referee.laps value, used together with
    // last_finished_state_ to detect a session restart and auto-clear
    // ebs_triggered_ — the only reliable in-bridge signal that the next
    // setCarControls belongs to a new run. See extraInfoTimerCb() for
    // the edge-detection logic.
    uint32_t last_laps_state_ = 0;

    // Monotonic-stamp guards for IMU and LiDAR. GLIM (and any LiDAR-IMU
    // SLAM pipeline) rejects samples whose timestamp ≤ the last accepted
    // sample's timestamp. The bridge's `node_->now()` snapshot in the
    // sensor and lidar publish threads can race occasionally — at 400 Hz
    // IMU we observed ~5-10 ms rewinds in 2026-04-26 step-2 verification,
    // which caused GLIM to reject every subsequent IMU sample after one
    // outlier-future sample landed first. Clamp each stream's published
    // stamp to be strictly greater than the previous one (bump by 1 ns
    // when the natural `now()` would regress). Same clock domain for both
    // streams (wall-clock from container), so no cross-stream alignment
    // is needed beyond per-stream monotonicity.
    rclcpp::Time last_imu_stamp_   = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time last_lidar_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Subscribers
    rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr control_cmd_sub_;
    // /signal/ebs — autonomy-initiated emergency stop. On first message the
    // bridge applies a full-brake command then disables api_control so no
    // subsequent setCarControls can release the brake (real-car EBS analog).
    //
    // /signal/ebs_reset — release the latch. Published by the control node
    // on init so a fresh session always starts with controls accepted, even
    // if the previous session ended with a latched EBS. Without this, the
    // bridge would silently drop every setCarControls until dv_pipeline_stack was
    // restarted (the flag had no reset path).
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ebs_request_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ebs_reset_sub_;
    bool ebs_triggered_ = false;
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
    rclcpp::TimerBase::SharedPtr tire_loads_timer_;

    // Config
    std::string mission_name_ = "trackdrive";
    std::string track_name_ = "A";
    bool competition_mode_ = false;
    std::vector<std::string> camera_names_;

    // Sensor mount offsets (ROS body frame, metres). Queried once at
    // connection time via `getSensorOffset <name>` so the static TFs the
    // bridge publishes follow settings.json instead of stale hardcoded
    // numbers.
    struct Vec3 { double x = 0.0; double y = 0.0; double z = 0.0; bool valid = false; };
    Vec3 lidar_offset_;
    std::map<std::string, Vec3> camera_offsets_;

    // Spawn position captured once at connection time via simGetVehiclePose.
    // resetSrvCb teleports back here; position-only (no quaternion) to avoid
    // the ENU↔UE5 ~90° yaw drift described in feedback_reset_orientation.md.
    struct HomePose { double x = 0.0; double y = 0.0; double z = 0.3; bool valid = false; };
    HomePose home_pose_;

    // Helper: query getSensorOffset for a single sensor, parse into Vec3.
    Vec3 querySensorOffset(const std::string& name);

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
