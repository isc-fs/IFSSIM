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
    if (sensor_thread_.joinable()) sensor_thread_.join();
    if (lidar_thread_.joinable()) lidar_thread_.join();
    if (sensor_stream_fd_ >= 0) close(sensor_stream_fd_);
    if (lidar_stream_fd_ >= 0) close(lidar_stream_fd_);
}

int IFSSIMRosWrapper::openStreamSocket(const std::string& command)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

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

    // Read "OK\n" acknowledgment
    char buf[16];
    int n = recv(sock, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
        close(sock);
        return -1;
    }
    buf[n] = 0;

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
}

void IFSSIMRosWrapper::initializePublishers()
{
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    gss_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gss", 10);
    lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/Lidar1", 10);
    go_signal_pub_ = node_->create_publisher<fs_msgs::msg::GoSignal>("signal/go", 10);

    for (const auto& cam_name : camera_names_) {
        auto pub = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
            "camera/" + cam_name + "/compressed", 10);
        camera_pubs_[cam_name] = pub;
        RCLCPP_INFO(node_->get_logger(), "Camera publisher: camera/%s/compressed", cam_name.c_str());
    }

    if (!competition_mode_) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("testing_only/odom", 10);
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

    finished_signal_sub_ = node_->create_subscription<fs_msgs::msg::FinishedSignal>(
        "signal/finished", 10,
        std::bind(&IFSSIMRosWrapper::finishedSignalCb, this, std::placeholders::_1));

    reset_srv_ = node_->create_service<fs_msgs::srv::Reset>(
        "reset",
        std::bind(&IFSSIMRosWrapper::resetSrvCb, this, std::placeholders::_1, std::placeholders::_2));
}

void IFSSIMRosWrapper::initializeTimers()
{
    // Camera disabled by default — enable with parameter camera_enabled:=true
    bool camera_enabled = node_->declare_parameter<bool>("camera_enabled", false);
    if (camera_enabled) {
        camera_timer_ = node_->create_wall_timer(200ms, std::bind(&IFSSIMRosWrapper::cameraTimerCb, this));
        RCLCPP_INFO(node_->get_logger(), "Camera streaming enabled at 5Hz");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Camera streaming disabled (use camera_enabled:=true to enable)");
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

    if (sensor_stream_fd_ >= 0) {
        sensor_thread_ = std::thread(&IFSSIMRosWrapper::sensorStreamThread, this);
    }
    if (lidar_stream_fd_ >= 0) {
        lidar_thread_ = std::thread(&IFSSIMRosWrapper::lidarStreamThread, this);
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

    while (streaming_ && sensor_stream_fd_ >= 0) {
        SensorFrame frame;
        if (!readExact(sensor_stream_fd_, &frame, sizeof(frame))) {
            RCLCPP_WARN(node_->get_logger(), "Sensor stream disconnected");
            break;
        }

        if (frame.magic != SENSOR_MAGIC) continue;
        onSensorFrame(frame);
    }
}

void IFSSIMRosWrapper::lidarStreamThread()
{
    RCLCPP_INFO(node_->get_logger(), "LiDAR stream thread started");

    while (streaming_ && lidar_stream_fd_ >= 0) {
        // Read header
        LidarChunkHeader header;
        if (!readExact(lidar_stream_fd_, &header, sizeof(header))) {
            RCLCPP_WARN(node_->get_logger(), "LiDAR stream disconnected");
            break;
        }

        if (header.magic != LIDAR_MAGIC || header.total_points <= 0) continue;

        // Read point data
        int data_size = header.total_points * 3 * sizeof(float);
        std::vector<float> points(header.total_points * 3);

        if (!readExact(lidar_stream_fd_, points.data(), data_size)) {
            RCLCPP_WARN(node_->get_logger(), "LiDAR stream data incomplete");
            break;
        }

        onLidarFrame(header, points.data());
    }
}

// =============================================================================
// Frame handlers — publish ROS2 topics
// =============================================================================

void IFSSIMRosWrapper::onSensorFrame(const SensorFrame& f)
{
    auto now = node_->now();

    // GPS
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = now;
        msg.header.frame_id = vehicle_frame_id_;
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
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now;
        msg.header.frame_id = vehicle_frame_id_;
        msg.linear_acceleration.x = f.accel_x;
        msg.linear_acceleration.y = f.accel_y;
        msg.linear_acceleration.z = f.accel_z;
        msg.angular_velocity.x = f.gyro_x;
        msg.angular_velocity.y = f.gyro_y;
        msg.angular_velocity.z = f.gyro_z;
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

    // TF (map → vehicle)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = map_frame_id_;
        tf.child_frame_id = vehicle_frame_id_;
        tf.transform.translation.x = f.pos_x;
        tf.transform.translation.y = f.pos_y;
        tf.transform.translation.z = f.pos_z;
        tf.transform.rotation.x = f.pose_qx;
        tf.transform.rotation.y = f.pose_qy;
        tf.transform.rotation.z = f.pose_qz;
        tf.transform.rotation.w = f.pose_qw;
        tf_broadcaster_->sendTransform(tf);
    }

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

void IFSSIMRosWrapper::onLidarFrame(const LidarChunkHeader& header, const float* points)
{
    int total_points = header.total_points;
    if (total_points <= 0) return;

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = vehicle_frame_id_ + "/Lidar1";
    msg.height = 1;
    msg.width = total_points;
    msg.is_dense = true;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

    for (int i = 0; i < total_points; i++) {
        *iter_x = points[i * 3];
        *iter_y = points[i * 3 + 1];
        *iter_z = points[i * 3 + 2];
        ++iter_x; ++iter_y; ++iter_z;
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

    if (client_ && client_->isConnected()) {
        std::string resp = client_->sendCommand("getRefereeState");
        if (!resp.empty()) {
            size_t epos = resp.find("\"event\":\"");
            if (epos != std::string::npos) {
                epos += 9;
                size_t eend = resp.find('"', epos);
                if (eend != std::string::npos) {
                    mission_name_ = resp.substr(epos, eend - epos);
                    msg.mission = mission_name_;
                }
            }
        }
    }
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

    // LiDAR
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = vehicle_frame_id_;
    tf.child_frame_id = vehicle_frame_id_ + "/Lidar1";
    tf.transform.translation.y = 1.4;
    tf.transform.translation.z = -0.2;
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);

    for (const auto& cam_name : camera_names_) {
        geometry_msgs::msg::TransformStamped ctf;
        ctf.header.stamp = now;
        ctf.header.frame_id = vehicle_frame_id_;
        ctf.child_frame_id = vehicle_frame_id_ + "/" + cam_name;
        ctf.transform.translation.y = 1.6;
        ctf.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(ctf);
    }
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

    gps_position_noise_std_ = pf("GpsPositionNoiseStd");
    imu_accel_noise_std_ = pf("AccelNoiseStd") / 100.0;
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
    std::ostringstream cmd;
    cmd << "setCarControls " << msg->throttle << " " << msg->steering << " " << msg->brake;
    client_->sendCommand(cmd.str());
}

void IFSSIMRosWrapper::finishedSignalCb(const fs_msgs::msg::FinishedSignal::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(node_->get_logger(), "Received finished signal");
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
