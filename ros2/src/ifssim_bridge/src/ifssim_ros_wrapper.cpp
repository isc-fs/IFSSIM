/**
 * IFSSIM ROS2 Wrapper — TCP push model.
 *
 * Sensor data streams continuously from the sim over persistent TCP connections.
 * No polling. The sim pushes binary frames at engine tick rate (~100Hz).
 * Camera and commands use traditional TCP request-response.
 */

#include "ifssim_ros_wrapper.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fs_msgs/msg/cone.hpp>
#include <sstream>
#include <cmath>
#include <cstring>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

using namespace std::chrono_literals;

IFSSIMRosWrapper::IFSSIMRosWrapper(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& host, int port, double timeout_sec)
    : node_(node), host_(host), port_(port), timeout_sec_(timeout_sec)
{
    mission_name_ = node_->declare_parameter<std::string>("mission_name", "trackdrive");
    track_name_ = node_->declare_parameter<std::string>("track_name", "A");
    competition_mode_ = node_->declare_parameter<bool>("competition_mode", false);

    initializeConnection();
    initializePublishers();
    initializeSubscribers();
    initializeTimers();
    startStreaming();
}

IFSSIMRosWrapper::~IFSSIMRosWrapper()
{
    streaming_ = false;
    // Close fds so blocking recv() calls in threads wake up
    int sfd = sensor_stream_fd_.exchange(-1);
    int lfd = lidar_stream_fd_.exchange(-1);
    if (sfd >= 0) close(sfd);
    if (lfd >= 0) close(lfd);
    // Wake the publish-threads out of their condition_variable waits so
    // they can observe streaming_=false and exit.
    sensor_pub_cv_.notify_all();
    lidar_pub_cv_.notify_all();
    if (sensor_thread_.joinable()) sensor_thread_.join();
    if (sensor_pub_thread_.joinable()) sensor_pub_thread_.join();
    if (lidar_thread_.joinable()) lidar_thread_.join();
    if (lidar_pub_thread_.joinable()) lidar_pub_thread_.join();
}

int IFSSIMRosWrapper::openStreamSocket(const std::string& command)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
        struct hostent* he = gethostbyname(host_.c_str());
        if (!he) { close(sock); return -1; }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }

    if (::connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }

    // Disable Nagle for low latency
    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // Send the stream command
    std::string msg = command + "\n";
    send(sock, msg.c_str(), msg.size(), 0);

    // Read exactly "OK\n" (3 bytes) — do NOT read more or we'll consume the start
    // of the first sensor frame (sent at 400Hz immediately after the ACK).
    char buf[4];
    size_t total = 0;
    while (total < 3) {
        ssize_t n = recv(sock, buf + total, 3 - total, 0);
        if (n <= 0) { close(sock); return -1; }
        total += n;
    }
    buf[3] = '\0';

    return sock;
}

void IFSSIMRosWrapper::initializeConnection()
{
    client_ = std::make_unique<TcpClient>();
    if (!client_->connect(host_, port_, timeout_sec_)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect command client to %s:%d", host_.c_str(), port_);
        return;
    }

    client_camera_ = std::make_unique<TcpClient>();
    if (!client_camera_->connect(host_, port_, timeout_sec_)) {
        RCLCPP_WARN(node_->get_logger(), "Camera client failed, using main");
    }

    client_->sendBool("enableApiControl");

    // Discover cameras
    std::string cam_list = client_->sendCommand("listCameras");
    if (!cam_list.empty() && cam_list[0] == '[') {
        std::string stripped = cam_list.substr(1, cam_list.size() - 2);
        std::stringstream ss(stripped);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(std::remove(token.begin(), token.end(), '"'), token.end());
            token.erase(std::remove(token.begin(), token.end(), ' '), token.end());
            if (!token.empty()) camera_names_.push_back(token);
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Discovered %zu cameras", camera_names_.size());

    if (client_->sendBool("ping")) {
        RCLCPP_INFO(node_->get_logger(), "IFSSIM connected (TCP push model)");
    }

    std::string settings = client_->sendCommand("getSettingsString");
    parseNoiseSettings(settings);

    // Cache sensor mount offsets from the plugin so static TFs don't
    // drift out of sync with settings.json. Queried once; offsets are
    // constructor-time constants on the sim side and won't change until
    // the sim is restarted.
    lidar_offset_ = querySensorOffset("lidar");
    for (const auto& cam_name : camera_names_) {
        camera_offsets_[cam_name] = querySensorOffset(cam_name);
    }
}

IFSSIMRosWrapper::Vec3 IFSSIMRosWrapper::querySensorOffset(const std::string& name)
{
    Vec3 v;
    if (!client_ || !client_->isConnected()) return v;
    std::string resp = client_->sendCommand("getSensorOffset " + name);
    if (resp.empty() || resp.find("\"error\"") != std::string::npos) {
        RCLCPP_WARN(node_->get_logger(),
            "getSensorOffset(%s) failed: %s — static TF for this sensor will be skipped",
            name.c_str(), resp.empty() ? "no response" : resp.c_str());
        return v;
    }
    v.x = client_->parseDouble(resp, "x");
    v.y = client_->parseDouble(resp, "y");
    v.z = client_->parseDouble(resp, "z");
    v.valid = true;
    RCLCPP_INFO(node_->get_logger(),
        "Sensor '%s' mount: (%.3f, %.3f, %.3f) m (REP-103 body frame)",
        name.c_str(), v.x, v.y, v.z);
    return v;
}

void IFSSIMRosWrapper::initializePublishers()
{
    // High-rate sensors use BEST_EFFORT QoS for the same reason /lidar/Lidar1
    // does (see comment below). With the default RELIABLE keep_last(10), a
    // single slow subscriber stalled the publish thread → kernel TCP recv
    // buffer filled → plugin SendAll hit its 1 s timeout → stream tear-down,
    // and /imu / /gps fell to 0 Hz under pipeline load. Sensor topics are a
    // "latest sample wins" stream by nature; drops are correct, backpressure
    // is not. GPS at 10 Hz is the marginal case — kept BEST_EFFORT for
    // consistency since its subscribers (none currently reliable-only) can
    // tolerate the rare drop.
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("gps", sensor_qos);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", sensor_qos);
    gss_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gss", sensor_qos);
    // /motor_rpm — engine RPM straight from Chaos VehicleMovement
    // (FSDSVehiclePawn::GetCarState → SensorFrame.rpm). Real-car parity:
    // IFS-08 inverter publishes motor RPM on CAN at 100 Hz. cone_slam
    // consumes it as a body-frame longitudinal velocity factor:
    //   v_x = rpm × (2π × WheelRadius / GearRatio) / 60
    // For IFS-08 (WheelRadius=0.228 m, GearRatio=2.909): v ≈ rpm × 0.00821 m/s.
    motor_rpm_pub_ = node_->create_publisher<std_msgs::msg::Float32>("motor_rpm", sensor_qos);
    // /lidar/Lidar1 uses BEST_EFFORT QoS (rather than the default RELIABLE
    // keep_last(10)) so a slow subscriber — most notably the numba-JIT
    // cone-detection node during its first ~15 s of warmup, but also any
    // foxglove_bridge consumer that pauses to render a frame — drops
    // messages locally instead of backpressuring the bridge's lidar_thread.
    // Without this, the lidar_thread blocks inside `publish()`, the kernel
    // TCP recv buffer fills, the *plugin's* TCP send buffer fills, the
    // plugin's SendAll hits its 1 s timeout, the plugin closes the stream,
    // and the bridge sees a dead socket → reconnect cascade. Net effect on
    // a real autocross run: LiDAR drops from 10 Hz to ~1 Hz the moment
    // pipeline subscribers come online. Sensor data is fundamentally a
    // best-effort stream — drops are fine, backpressure is not.
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/Lidar1", lidar_qos);
    go_signal_pub_ = node_->create_publisher<fs_msgs::msg::GoSignal>("signal/go", 10);
    // Published edge-triggered when the referee's bFinished flips false->true;
    // consumers (e.g. the control node) brake the car to end the event cleanly.
    finished_signal_pub_ = node_->create_publisher<fs_msgs::msg::FinishedSignal>("signal/finished", 10);

    for (const auto& cam_name : camera_names_) {
        auto pub = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "camera/" + cam_name + "/compressed", 10);
        camera_pubs_[cam_name] = pub;
        RCLCPP_INFO(node_->get_logger(), "Camera publisher: camera/%s/compressed", cam_name.c_str());
    }

    if (!competition_mode_) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("testing_only/odom", sensor_qos);
        auto qos = rclcpp::QoS(1).transient_local();
        track_pub_ = node_->create_publisher<fs_msgs::msg::Track>("testing_only/track", qos);
        extra_info_pub_ = node_->create_publisher<fs_msgs::msg::ExtraInfo>("testing_only/extra_info", 10);
    }

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    RCLCPP_INFO(node_->get_logger(), "Publishers initialized (competition_mode: %s)",
        competition_mode_ ? "true" : "false");
}

void IFSSIMRosWrapper::initializeSubscribers()
{
    control_cmd_sub_ = node_->create_subscription<fs_msgs::msg::ControlCommand>(
        "control_command", 10,
        std::bind(&IFSSIMRosWrapper::controlCommandCb, this, std::placeholders::_1));

    // Latched QoS so a late-joining publisher that already fired its EBS
    // signal still triggers a handler on connect. Only one in-flight msg
    // needed — EBS is one-shot.
    auto ebs_qos = rclcpp::QoS(1).transient_local();
    ebs_request_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "signal/ebs", ebs_qos,
        std::bind(&IFSSIMRosWrapper::ebsRequestCb, this, std::placeholders::_1));

    // /signal/ebs_reset — explicit release of the EBS latch. Without this
    // the bridge's `ebs_triggered_` had no way to flip back to false after
    // a session ended with the autonomous-stop logic firing — every
    // subsequent Start Session would silently drop setCarControls until
    // the user restarted dv_pipeline_stack. The control node publishes this once
    // on init, so a fresh session always starts with controls accepted.
    ebs_reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "signal/ebs_reset", ebs_qos,
        std::bind(&IFSSIMRosWrapper::ebsResetCb, this, std::placeholders::_1));

    reset_srv_ = node_->create_service<fs_msgs::srv::Reset>(
        "reset",
        std::bind(&IFSSIMRosWrapper::resetSrvCb, this, std::placeholders::_1, std::placeholders::_2));
}

void IFSSIMRosWrapper::initializeTimers()
{
    // Camera rate configurable via parameter (default 10Hz, 0 to disable)
    int camera_hz = node_->declare_parameter<int>("camera_hz", 0);
    if (camera_hz > 0) {
        int period_ms = 1000 / camera_hz;
        camera_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&IFSSIMRosWrapper::cameraTimerCb, this));
        RCLCPP_INFO(node_->get_logger(), "Camera streaming at %dHz (%dms)", camera_hz, period_ms);
    } else {
        RCLCPP_INFO(node_->get_logger(), "Camera streaming disabled (use camera_hz:=10 to enable)");
    }
    go_signal_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::goSignalTimerCb, this));
    static_tf_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::staticTfCb, this));

    if (!competition_mode_) {
        extra_info_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::extraInfoTimerCb, this));
        track_publish_timer_ = node_->create_wall_timer(5000ms, std::bind(&IFSSIMRosWrapper::trackPublishCb, this));
    }
}

void IFSSIMRosWrapper::startStreaming()
{
    // Open streaming connections
    sensor_stream_fd_ = openStreamSocket("streamSensors");
    if (sensor_stream_fd_ >= 0) {
        RCLCPP_INFO(node_->get_logger(), "Sensor stream connected");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open sensor stream");
    }

    lidar_stream_fd_ = openStreamSocket("streamLidar");
    if (lidar_stream_fd_ >= 0) {
        RCLCPP_INFO(node_->get_logger(), "LiDAR stream connected");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open LiDAR stream");
    }

    streaming_ = true;

    sensor_thread_     = std::thread(&IFSSIMRosWrapper::sensorStreamThread,  this);
    sensor_pub_thread_ = std::thread(&IFSSIMRosWrapper::sensorPublishThread, this);
    lidar_thread_      = std::thread(&IFSSIMRosWrapper::lidarStreamThread,   this);
    lidar_pub_thread_  = std::thread(&IFSSIMRosWrapper::lidarPublishThread,  this);

    // If initial connection failed (UE5 not in Play mode yet), kick off reconnect
    if (sensor_stream_fd_ < 0 || lidar_stream_fd_ < 0) {
        std::thread(&IFSSIMRosWrapper::triggerReconnect, this).detach();
    }
}

// =============================================================================
// Streaming threads — read continuous binary data from sim
// =============================================================================

static bool readExact(int fd, void* buf, size_t len)
{
    size_t total = 0;
    while (total < len) {
        ssize_t n = recv(fd, (char*)buf + total, len - total, 0);
        if (n <= 0) return false;
        total += n;
    }
    return true;
}

void IFSSIMRosWrapper::sensorStreamThread()
{
    RCLCPP_INFO(node_->get_logger(), "Sensor stream thread started");

    while (streaming_) {
        int fd = sensor_stream_fd_.load();
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        SensorFrame frame;
        if (!readExact(fd, &frame, sizeof(frame))) {
            RCLCPP_WARN(node_->get_logger(), "Sensor stream disconnected");
            int expected = fd;
            if (sensor_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }

        if (frame.magic != SENSOR_MAGIC) continue;

        // Hand off to the publish thread. Single-slot buffer with
        // drop-oldest semantics — if the consumer hasn't yet drained the
        // previous frame, we overwrite it. The mutex is held only long
        // enough to swap the std::optional; publish() runs entirely
        // outside the lock on the consumer side. See the header comment
        // for the failure mode this prevents.
        {
            std::lock_guard<std::mutex> lock(sensor_pub_mutex_);
            sensor_pending_ = frame;
        }
        sensor_pub_cv_.notify_one();
    }
}

void IFSSIMRosWrapper::sensorPublishThread()
{
    RCLCPP_INFO(node_->get_logger(), "Sensor publish thread started");

    while (streaming_) {
        SensorFrame frame;
        {
            std::unique_lock<std::mutex> lock(sensor_pub_mutex_);
            sensor_pub_cv_.wait(lock, [this] {
                return !streaming_ || sensor_pending_.has_value();
            });
            if (!streaming_) break;
            frame = *sensor_pending_;
            sensor_pending_.reset();
        }

        // publish() runs OUTSIDE the lock so the recv thread can keep
        // pushing new frames. Whatever time DDS / subscriber back-
        // pressure costs is purely consumer-side; the recv loop never
        // sees it.
        onSensorFrame(frame);
    }

    RCLCPP_INFO(node_->get_logger(), "Sensor publish thread exiting");
}

void IFSSIMRosWrapper::lidarStreamThread()
{
    RCLCPP_INFO(node_->get_logger(), "LiDAR stream thread started");

    while (streaming_) {
        int fd = lidar_stream_fd_.load();
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        LidarChunkHeader header;
        if (!readExact(fd, &header, sizeof(header))) {
            RCLCPP_WARN(node_->get_logger(), "LiDAR stream disconnected");
            int expected = fd;
            if (lidar_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }

        if (header.magic != LIDAR_MAGIC || header.total_points <= 0) continue;

        int data_size = header.total_points * 3 * sizeof(float);
        std::vector<float> points(header.total_points * 3);

        if (!readExact(fd, points.data(), data_size)) {
            RCLCPP_WARN(node_->get_logger(), "LiDAR stream data incomplete");
            int expected = fd;
            if (lidar_stream_fd_.compare_exchange_strong(expected, -1)) close(fd);
            triggerReconnect();
            continue;
        }

        // Hand the frame off to the publish thread. Single-slot buffer
        // with drop-oldest semantics: if the consumer hasn't yet
        // drained the previous frame, the new one overwrites it. This
        // keeps the recv loop free to immediately re-enter recv() and
        // drain the kernel TCP buffer — preventing the backpressure
        // chain that used to cause stream tear-downs every ~1 s under
        // pipeline load. The mutex is held only long enough to swap
        // the std::optional (a pointer-swap level operation since
        // the underlying vector is moved); publish() runs entirely
        // outside the lock on the consumer side.
        {
            std::lock_guard<std::mutex> lock(lidar_pub_mutex_);
            lidar_pending_ = PendingLidarFrame{header, std::move(points)};
        }
        lidar_pub_cv_.notify_one();
    }
}

void IFSSIMRosWrapper::lidarPublishThread()
{
    RCLCPP_INFO(node_->get_logger(), "LiDAR publish thread started");

    while (streaming_) {
        PendingLidarFrame frame;
        {
            std::unique_lock<std::mutex> lock(lidar_pub_mutex_);
            lidar_pub_cv_.wait(lock, [this] {
                return !streaming_ || lidar_pending_.has_value();
            });
            if (!streaming_) break;
            frame = std::move(*lidar_pending_);
            lidar_pending_.reset();
        }

        // publish() runs OUTSIDE the lock so the recv thread can keep
        // pushing new frames into the slot. Whatever time this takes
        // (PointCloud2 serialization + DDS fan-out + downstream subscriber
        // back-pressure) is purely consumer-side; the recv loop never
        // sees it.
        onLidarFrame(frame.header, frame.points.data());
    }

    RCLCPP_INFO(node_->get_logger(), "LiDAR publish thread exiting");
}

void IFSSIMRosWrapper::triggerReconnect()
{
    std::lock_guard<std::mutex> lock(reconnect_mutex_);

    // Another thread may have already completed the reconnect while we waited
    if (sensor_stream_fd_ >= 0 && lidar_stream_fd_ >= 0) return;

    RCLCPP_INFO(node_->get_logger(), "Reconnecting to IFSSIM (level reset?)...");

    // Only recreate the command client if it's actually broken. A transient
    // stream glitch (partial LiDAR frame, brief sensor recv hiccup) does NOT
    // mean UE5 is gone — the streams use *separate* TCP connections from
    // the command client. Tearing down and rebuilding `client_` on every
    // stream hiccup was breaking control: each reconnect dropped any
    // in-flight setCarControls and re-issued enableApiControl, while
    // `controlCommandCb` reads `client_` lock-free.
    //
    // Symptom that surfaced today: with a 10 Hz LiDAR push and the plugin
    // occasionally closing its push socket mid-frame, this function fired
    // ~once per second. Each fire tore down the command client for ~30 ms,
    // so a meaningful fraction of `setCarControls` were silently dropped.
    // The car drove like the controls were lagging by 1 s.
    if (!client_ || !client_->isConnected()) {
        // Belt-and-suspenders: clear any latched EBS state when we reconnect
        // the *command* client. The primary release path is
        // /signal/ebs_reset published by the control node on init, but if
        // the control node crashed without publishing — or if UE5 itself was
        // restarted — we don't want a stale flag to keep dropping
        // setCarControls forever. Doing this here (instead of on every
        // stream-only reconnect) prevents the EBS flag from being cleared
        // mid-session by spurious LiDAR-stream reconnects.
        ebs_triggered_ = false;

        int attempt = 0;
        while (streaming_) {
            client_ = std::make_unique<TcpClient>();
            if (client_->connect(host_, port_, 3.0)) {
                client_->sendBool("enableApiControl");
                RCLCPP_INFO(node_->get_logger(), "Command client reconnected (attempt %d)", ++attempt);
                break;
            }
            RCLCPP_INFO(node_->get_logger(), "Reconnect attempt %d failed, retrying in 2s...", ++attempt);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // Reconnect sensor stream
    while (streaming_ && sensor_stream_fd_ < 0) {
        int fd = openStreamSocket("streamSensors");
        if (fd >= 0) {
            sensor_stream_fd_.store(fd);
            RCLCPP_INFO(node_->get_logger(), "Sensor stream reconnected");
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // Reconnect LiDAR stream
    while (streaming_ && lidar_stream_fd_ < 0) {
        int fd = openStreamSocket("streamLidar");
        if (fd >= 0) {
            lidar_stream_fd_.store(fd);
            RCLCPP_INFO(node_->get_logger(), "LiDAR stream reconnected");
        } else {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
}

// =============================================================================
// Frame handlers — publish ROS2 topics
// =============================================================================

void IFSSIMRosWrapper::onSensorFrame(const SensorFrame& f)
{
    auto now = node_->now();
    ++sensor_frame_count_;

    // GPS — 10 Hz (every 40 frames of the 400 Hz stream)
    if (sensor_frame_count_ % 40 == 0)
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now;
        msg.header.frame_id = "fsds/GPS";
        msg.latitude = f.latitude;
        msg.longitude = f.longitude;
        msg.altitude = f.altitude;
        msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        double gps_var = gps_position_noise_std_ * gps_position_noise_std_;
        msg.position_covariance = {gps_var,0,0, 0,gps_var,0, 0,0,gps_var};
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps_pub_->publish(msg);
    }

    // IMU
    {
        // Monotonic guard — see last_imu_stamp_ comment in the header. GLIM
        // rejects any IMU sample whose stamp ≤ the previously-accepted one;
        // bump by 1 ns when node_->now() would regress so the publish stream
        // is strictly increasing.
        rclcpp::Time imu_stamp = now;
        if (last_imu_stamp_.nanoseconds() > 0 && imu_stamp <= last_imu_stamp_) {
            imu_stamp = last_imu_stamp_ + rclcpp::Duration::from_nanoseconds(1);
        }
        last_imu_stamp_ = imu_stamp;

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = imu_stamp;
        msg.header.frame_id = "fsds/IMU";
        // Convert UE5 body frame (left-handed: X=fwd, Y=right, Z=up) to
        // ROS REP-103 body frame (right-handed: X=fwd, Y=left, Z=up).
        // The UDP broadcaster forwards FSDSImuSensor's body-frame outputs
        // (accel was already body-framed in the sensor; gyro was made
        // body-framed by the same sensor file as of 2026-04-27).
        //
        // Linear acceleration is a polar vector — under the Y-axis basis
        // change det(R)=-1 it transforms as v → (vx, -vy, vz).
        //
        // Angular velocity is a pseudovector (axial vector). Under the
        // same basis change with det(R)=-1 the transformation gains an
        // extra det factor: ω → (-ωx, ωy, -ωz). Only the polar transform
        // (Y-flip) was applied originally; that produced the right
        // gravity vector but mirrored yaw direction, which the
        // 2026-04-27 Phase 2 drive made obvious — fast_LIMO integrated
        // turns the wrong way.
        //
        // The orientation quat is already ENU-converted via UEQuatToENU
        // upstream and needs no further flip here.
        msg.linear_acceleration.x =  f.accel_x;
        msg.linear_acceleration.y = -f.accel_y;
        msg.linear_acceleration.z =  f.accel_z;
        msg.angular_velocity.x = -f.gyro_x;
        msg.angular_velocity.y =  f.gyro_y;
        msg.angular_velocity.z = -f.gyro_z;
        msg.orientation.x = f.orient_x;
        msg.orientation.y = f.orient_y;
        msg.orientation.z = f.orient_z;
        msg.orientation.w = f.orient_w;
        msg.orientation_covariance = {1e-6,0,0, 0,1e-6,0, 0,0,1e-6};
        double gyro_var = imu_gyro_noise_std_ * imu_gyro_noise_std_;
        msg.angular_velocity_covariance = {gyro_var,0,0, 0,gyro_var,0, 0,0,gyro_var};
        double accel_var = imu_accel_noise_std_ * imu_accel_noise_std_;
        msg.linear_acceleration_covariance = {accel_var,0,0, 0,accel_var,0, 0,0,accel_var};
        imu_pub_->publish(msg);
    }

    // GSS + TF + Odom — 100 Hz (every 4 frames of the 400 Hz stream)
    if (sensor_frame_count_ % 4 == 0)
    {
        // GSS
        {
            geometry_msgs::msg::TwistWithCovarianceStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = vehicle_frame_id_;
            msg.twist.twist.linear.x = f.gss_vx;
            msg.twist.twist.linear.y = f.gss_vy;
            msg.twist.twist.linear.z = f.gss_vz;
            double gss_var = gss_velocity_noise_std_ * gss_velocity_noise_std_;
            msg.twist.covariance[0] = gss_var;
            msg.twist.covariance[7] = gss_var;
            msg.twist.covariance[14] = gss_var;
            gss_pub_->publish(msg);
        }

        // Motor RPM — straight from Chaos. Real IFS-08 publishes the
        // same field on CAN at 100 Hz; matching rate keeps sim/real
        // parity for the cone_slam velocity factor.
        {
            std_msgs::msg::Float32 msg;
            msg.data = f.rpm;
            motor_rpm_pub_->publish(msg);
        }

        // TF odom → fsds/FSCar — REMOVED in PR #3 of the GLIM rebuild.
        // GLIM (LiDAR-IMU SLAM) owns odom → base_link now. Odometria_perfecta
        // continues to publish odom → fsds/FSCar from /fsds/testing_only/odom
        // until step 4 of the rebuild renames pipeline frame references and
        // step 5 deletes Odometria_perfecta entirely. The bridge no longer
        // needs to publish a duplicate sim-GT TF — and doing so would conflict
        // with GLIM's odom frame (multiple writers to the same TF parent
        // cause non-deterministic last-writer-wins behavior in TF2).

        // Odom (testing only)
        if (odom_pub_) {
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = now;
            msg.header.frame_id = map_frame_id_;
            msg.child_frame_id = vehicle_frame_id_;
            msg.pose.pose.position.x = f.pos_x;
            msg.pose.pose.position.y = f.pos_y;
            msg.pose.pose.position.z = f.pos_z;
            msg.pose.pose.orientation.x = f.pose_qx;
            msg.pose.pose.orientation.y = f.pose_qy;
            msg.pose.pose.orientation.z = f.pose_qz;
            msg.pose.pose.orientation.w = f.pose_qw;
            double pos_var = gps_position_noise_std_ * gps_position_noise_std_;
            msg.pose.covariance[0] = pos_var;
            msg.pose.covariance[7] = pos_var;
            msg.pose.covariance[14] = pos_var;
            msg.pose.covariance[21] = 1e-6;
            msg.pose.covariance[28] = 1e-6;
            msg.pose.covariance[35] = 1e-6;
            odom_pub_->publish(msg);
        }
    }
}

void IFSSIMRosWrapper::onLidarFrame(const LidarChunkHeader& header, const float* points)
{
    int total_points = header.total_points;
    if (total_points <= 0) return;

    // Monotonic guard — same rationale as the IMU clamp in onSensorFrame.
    // GLIM expects strictly increasing timestamps on /lidar/Lidar1.
    rclcpp::Time lidar_stamp = node_->now();
    if (last_lidar_stamp_.nanoseconds() > 0 && lidar_stamp <= last_lidar_stamp_) {
        lidar_stamp = last_lidar_stamp_ + rclcpp::Duration::from_nanoseconds(1);
    }
    last_lidar_stamp_ = lidar_stamp;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = lidar_stamp;
    msg.header.frame_id = "fsds/Lidar";
    msg.height = 1;
    msg.width = total_points;
    msg.is_dense = true;
    msg.is_bigendian = false;

    // x/y/z + per-point absolute timestamp (FLOAT64, seconds). fast_LIMO's
    // HESAI handler hard-requires the timestamp field; without it the
    // node throws "FATAL ERROR: invalid pointcloud structure" on the
    // first scan.
    //
    // The IFSSIM LiDAR sensor (FSDSLidarSensor.cpp) is INSTANTANEOUS —
    // it snapshots the car transform ONCE per scan and ray-traces all
    // ~20000 points from that single pose. No physical sweep, no
    // motion-during-scan. The real Hesai ATX (which we model) is a
    // hybrid solid-state LiDAR: 128 vertical lasers fire simultaneously,
    // and a MEMS mirror sweeps horizontally over the 100 ms scan
    // period, so the real hardware DOES have per-azimuth time
    // variation. The simulator collapses that sweep into a single tick.
    //
    // We therefore set every point's timestamp to the scan stamp.
    // fast_LIMO's deskew uses per-point time to interpolate IMU pose at
    // each point's capture instant; with all-equal times the
    // interpolation collapses to identity and no fake motion
    // compensation is applied. This matches the simulator's actual
    // behavior.
    //
    // History:
    //   - Initial (linear-by-index): t = scan_start + (i/N) * 0.1
    //   - Tried (azimuthal, fetty31 #13): t = scan_start + (pi - atan2(y,x))/(2*pi) * 0.1
    // Both assumed a real spinning sweep and applied wrong deskew on the
    // sim's instantaneous data. The 2026-04-27 audit confirmed FSDS is
    // single-tick by reading FSDSLidarSensor.cpp:52 (single
    // GetActorTransform() before the ray trace loop).
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
        "x",         1, sensor_msgs::msg::PointField::FLOAT32,
        "y",         1, sensor_msgs::msg::PointField::FLOAT32,
        "z",         1, sensor_msgs::msg::PointField::FLOAT32,
        "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64);
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float>  iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float>  iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float>  iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<double> iter_t(msg, "timestamp");

    const double scan_stamp_sec = lidar_stamp.seconds();

    for (int i = 0; i < total_points; i++) {
        *iter_x = points[i * 3];
        *iter_y = points[i * 3 + 1];
        *iter_z = points[i * 3 + 2];
        *iter_t = scan_stamp_sec;
        ++iter_x; ++iter_y; ++iter_z; ++iter_t;
    }

    lidar_pub_->publish(msg);
}

// =============================================================================
// TCP timer callbacks (low frequency)
// =============================================================================

void IFSSIMRosWrapper::cameraTimerCb()
{
    TcpClient* cam = (client_camera_ && client_camera_->isConnected())
        ? client_camera_.get() : client_.get();
    if (!cam || !cam->isConnected()) return;

    for (const auto& cam_name : camera_names_) {
        auto it = camera_pubs_.find(cam_name);
        if (it == camera_pubs_.end()) continue;

        std::vector<uint8_t> data;
        std::string header = cam->sendBinaryCommand("simGetImageBinary " + cam_name + " 0", data);

        if (!data.empty()) {
            sensor_msgs::msg::CompressedImage msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = vehicle_frame_id_ + "/" + cam_name;
            msg.format = "png";
            msg.data = std::move(data);
            it->second->publish(msg);
        }
    }
}

void IFSSIMRosWrapper::goSignalTimerCb()
{
    fs_msgs::msg::GoSignal msg;
    msg.header.stamp = node_->now();
    msg.mission = mission_name_;
    msg.track = track_name_;
    go_signal_pub_->publish(msg);
}

void IFSSIMRosWrapper::extraInfoTimerCb()
{
    if (!client_ || !client_->isConnected()) return;
    std::string resp = client_->sendCommand("getRefereeState");
    if (resp.empty()) return;

    fs_msgs::msg::ExtraInfo msg;
    msg.doo_counter = (uint32_t)client_->parseDouble(resp, "doo_counter");
    msg.laps = (uint32_t)client_->parseDouble(resp, "laps");
    extra_info_pub_->publish(msg);

    // Detect finished edge (false -> true) and notify downstream.
    // Robust JSON key-level parse — previously a raw substring search for
    // `"finished":true`, which was fragile to key reordering and to any
    // other field whose string value happened to contain that literal.
    bool finished_now = client_->parseBool(resp, "finished");
    if (finished_now && !last_finished_state_ && finished_signal_pub_) {
        fs_msgs::msg::FinishedSignal fin;
        fin.header.stamp = node_->now();
        finished_signal_pub_->publish(fin);
        RCLCPP_INFO(node_->get_logger(), "Event finished — published /signal/finished");
    }

    // Detect session restart: finished true → false, OR the lap counter
    // went backwards (referee was reset by a fresh setEvent call mid-
    // session, before the previous one ever flipped finished=true).
    // Either case means the next setCarControls on the wire belongs to a
    // *new* run, so any latched ebs_triggered_ from the previous run is
    // stale and must be cleared. This is the bridge-side counterpart to
    // the control node's publish-on-init of /signal/ebs_reset, which
    // races DDS discovery after pipeline restarts; with both in place
    // the latch will reliably clear regardless of which signal lands
    // first.
    uint32_t laps_now = (uint32_t)client_->parseDouble(resp, "laps");
    const bool finished_edge = !finished_now && last_finished_state_;
    const bool lap_rewind = laps_now < last_laps_state_;
    if ((finished_edge || lap_rewind) && ebs_triggered_) {
        ebs_triggered_ = false;
        RCLCPP_INFO(node_->get_logger(),
            "Session restart detected (%s) — EBS latch cleared",
            finished_edge ? "finished true→false" : "lap counter rewound");
    }

    last_finished_state_ = finished_now;
    last_laps_state_ = laps_now;
}

void IFSSIMRosWrapper::trackPublishCb()
{
    if (!client_ || !client_->isConnected()) return;
    std::string resp = client_->sendCommand("getRefereeState");
    if (resp.empty()) return;

    fs_msgs::msg::Track msg;
    size_t arr_start = resp.find("\"cone_positions\":[");
    if (arr_start == std::string::npos) return;
    arr_start = resp.find('[', arr_start);
    size_t arr_end = resp.find(']', arr_start);
    if (arr_end == std::string::npos) return;

    std::string arr = resp.substr(arr_start + 1, arr_end - arr_start - 1);
    if (arr.empty()) return;

    size_t pos = 0;
    while (pos < arr.size()) {
        size_t obj_start = arr.find('{', pos);
        if (obj_start == std::string::npos) break;
        size_t obj_end = arr.find('}', obj_start);
        if (obj_end == std::string::npos) break;
        std::string obj = arr.substr(obj_start, obj_end - obj_start + 1);
        pos = obj_end + 1;

        auto pf = [&obj](const std::string& key) -> double {
            size_t kpos = obj.find("\"" + key + "\":");
            if (kpos == std::string::npos) return 0.0;
            return std::stod(obj.substr(kpos + key.size() + 3));
        };

        fs_msgs::msg::Cone cone;
        cone.location.x = pf("x");
        cone.location.y = pf("y");
        cone.color = (uint8_t)pf("color");
        msg.track.push_back(cone);
    }

    if (!msg.track.empty()) track_pub_->publish(msg);
}

void IFSSIMRosWrapper::staticTfCb()
{
    auto now = node_->now();

    // Sensor static transforms — base_link → fsds/{IMU,Lidar,GPS}.
    //
    // GLIM (LiDAR-IMU SLAM) owns the odom→base_link dynamic transform; the
    // bridge owns the static base_link→sensor chain. GLIM uses these to
    // compute T_lidar_imu for scan undistortion and to express its output
    // in the body frame.
    //
    // Identity transforms — UE5 already pre-transforms LiDAR points and IMU
    // readings into the vehicle frame before sending them to the bridge.
    // Applying the settings.json offsets (Lidar1.X/Y/Z = 0.5/0/0.9) here
    // would double-apply them and place sensor data at the wrong location.
    //
    // TODO real-car: when the actual IFS-08 sends LiDAR points in the
    // sensor's own frame, replace these with the real CAD offsets, source
    // from getSensorOffset RPC (cached at initializeConnection).
    auto publishIdentityStatic = [&](const std::string& parent, const std::string& child) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.rotation.w = 1.0;  // identity (translation defaults to zero)
        static_tf_broadcaster_->sendTransform(tf);
    };

    publishIdentityStatic("base_link", "fsds/IMU");
    publishIdentityStatic("base_link", "fsds/Lidar");
    publishIdentityStatic("base_link", "fsds/GPS");

    // Camera static TFs — REMOVED in PR #3 step 5. Cameras don't exist on
    // the real IFS-08 (memo: project_no_cameras_on_real_car.md), and after
    // Odometria_perfecta was deleted, the legacy fsds/FSCar parent has
    // no publisher anyway — leaving the camera children would create a
    // disconnected subtree. Camera *image* publishing is unaffected.
}

void IFSSIMRosWrapper::parseNoiseSettings(const std::string& settings)
{
    auto pf = [&settings](const std::string& key) -> double {
        size_t pos = settings.find("\"" + key + "\"");
        if (pos == std::string::npos) return 0.0;
        pos = settings.find(':', pos);
        if (pos == std::string::npos) return 0.0;
        pos++;
        while (pos < settings.size() && settings[pos] == ' ') pos++;
        try { return std::stod(settings.substr(pos)); } catch (...) { return 0.0; }
    };

    // settings.json values are in SI (m, m/s, m/s², rad/s). The accel
    // conversion that used to divide by 100 here was compensating for an
    // older cm/s² convention inside the plugin — settings are now SI on
    // both sides (the plugin ×100 bumps them to its internal cm/s² accel
    // signal; the IMU RPC still emits m/s² via UEVelocityToENU).
    gps_position_noise_std_ = pf("GpsPositionNoiseStd");
    imu_accel_noise_std_ = pf("AccelNoiseStd");
    imu_gyro_noise_std_ = pf("GyroNoiseStd");
    gss_velocity_noise_std_ = pf("VelocityNoiseStd");

    RCLCPP_INFO(node_->get_logger(), "Noise: GPS=%.3fm, IMU accel=%.4f gyro=%.4f, GSS=%.3f",
        gps_position_noise_std_, imu_accel_noise_std_, imu_gyro_noise_std_, gss_velocity_noise_std_);
}

// =============================================================================
// Subscriber callbacks
// =============================================================================

void IFSSIMRosWrapper::controlCommandCb(const fs_msgs::msg::ControlCommand::SharedPtr msg)
{
    if (!client_ || !client_->isConnected()) return;
    // Drop any commands after EBS has latched — real-car EBS can't be
    // overridden by the AS until it's released, and the sim's analog is
    // api_control disabled. Keeping the if-check here as well means a late
    // publisher won't slip a post-EBS setCarControls through the client
    // before the disableApiControl call has propagated.
    if (ebs_triggered_) return;
    std::ostringstream cmd;
    cmd << "setCarControls " << msg->throttle << " " << msg->steering << " " << msg->brake;
    client_->sendCommand(cmd.str());
}

void IFSSIMRosWrapper::ebsRequestCb(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    if (ebs_triggered_ || !client_ || !client_->isConnected()) return;
    ebs_triggered_ = true;
    // Route EBS through the dedicated handbrake channel. The older
    // `setCarControls 0 0 1 + disableApiControl` sequence was silently
    // undone every tick by UE5's axis-input system (keyboard brake axis
    // reads 0 → overwrites CurrentControls.Brake), delivering only
    // drag decel (~2 m/s²) instead of full brake (~11 m/s²). ActivateEbs
    // locks all input channels and clamps handbrake=true.
    client_->sendCommand("activateEbs");
    RCLCPP_INFO(node_->get_logger(), "EBS engaged — handbrake latched, all inputs locked");
}

void IFSSIMRosWrapper::ebsResetCb(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    if (!ebs_triggered_) return;  // already cleared, nothing to do
    ebs_triggered_ = false;
    RCLCPP_INFO(node_->get_logger(), "EBS latch released — control commands re-enabled");
}

void IFSSIMRosWrapper::resetSrvCb(
    const std::shared_ptr<fs_msgs::srv::Reset::Request> request,
    std::shared_ptr<fs_msgs::srv::Reset::Response> response)
{
    (void)request;
    if (client_ && client_->isConnected()) {
        client_->sendCommand("reset");
        response->success = true;
    } else {
        response->success = false;
    }
}
