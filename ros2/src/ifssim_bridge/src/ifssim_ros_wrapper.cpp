#include "ifssim_ros_wrapper.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fs_msgs/msg/cone.hpp>
#include <sstream>
#include <cmath>
#include <vector>
#include <cstring>
#include <algorithm>

using namespace std::chrono_literals;

IFSSIMRosWrapper::IFSSIMRosWrapper(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& host,
    int port,
    double timeout_sec)
    : node_(node), host_(host), port_(port), timeout_sec_(timeout_sec)
{
    // Read parameters
    mission_name_ = node_->declare_parameter<std::string>("mission_name", "trackdrive");
    track_name_ = node_->declare_parameter<std::string>("track_name", "A");
    competition_mode_ = node_->declare_parameter<bool>("competition_mode", false);

    initializeConnection();
    initializePublishers();
    initializeSubscribers();
    initializeTimers();

    RCLCPP_INFO(node_->get_logger(), "IFSSIM ROS2 Bridge initialized");
}

IFSSIMRosWrapper::~IFSSIMRosWrapper()
{
    if (client_) client_->disconnect();
    if (client_lidar_) client_lidar_->disconnect();
}

void IFSSIMRosWrapper::initializeConnection()
{
    client_ = std::make_unique<TcpClient>();
    client_lidar_ = std::make_unique<TcpClient>();

    if (!client_->connect(host_, port_, timeout_sec_)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to connect main client to %s:%d", host_.c_str(), port_);
        return;
    }

    if (!client_lidar_->connect(host_, port_, timeout_sec_)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to connect lidar client, using main client");
    }

    client_camera_ = std::make_unique<TcpClient>();
    if (!client_camera_->connect(host_, port_, timeout_sec_)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to connect camera client, using main client");
    }

    // Enable API control
    client_->sendBool("enableApiControl");

    // Discover cameras
    std::string cam_list = client_->sendCommand("listCameras");
    // Parse ["cam1","cam2"] format
    if (!cam_list.empty() && cam_list[0] == '[') {
        std::string stripped = cam_list.substr(1, cam_list.size() - 2);
        std::stringstream ss(stripped);
        std::string token;
        while (std::getline(ss, token, ',')) {
            // Remove quotes
            token.erase(std::remove(token.begin(), token.end(), '"'), token.end());
            token.erase(std::remove(token.begin(), token.end(), ' '), token.end());
            if (!token.empty()) {
                camera_names_.push_back(token);
            }
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Discovered %zu cameras", camera_names_.size());

    // Ping
    if (client_->sendBool("ping")) {
        RCLCPP_INFO(node_->get_logger(), "IFSSIM simulator connected successfully");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Ping failed!");
    }
}

void IFSSIMRosWrapper::initializePublishers()
{
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 10);
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    gss_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gss", 10);
    lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/Lidar1", 10);
    go_signal_pub_ = node_->create_publisher<fs_msgs::msg::GoSignal>("signal/go", 10);

    // Camera publishers — one per camera
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
    gps_timer_ = node_->create_wall_timer(100ms, std::bind(&IFSSIMRosWrapper::gpsTimerCb, this));
    imu_timer_ = node_->create_wall_timer(std::chrono::microseconds(2500), std::bind(&IFSSIMRosWrapper::imuTimerCb, this)); // 400 Hz (BMI088)
    gss_timer_ = node_->create_wall_timer(10ms, std::bind(&IFSSIMRosWrapper::gssTimerCb, this));
    lidar_timer_ = node_->create_wall_timer(100ms, std::bind(&IFSSIMRosWrapper::lidarTimerCb, this));
    camera_timer_ = node_->create_wall_timer(100ms, std::bind(&IFSSIMRosWrapper::cameraTimerCb, this)); // 10 Hz
    go_signal_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::goSignalTimerCb, this));
    static_tf_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::staticTfCb, this));

    if (!competition_mode_) {
        odom_timer_ = node_->create_wall_timer(4ms, std::bind(&IFSSIMRosWrapper::odomTimerCb, this));
        extra_info_timer_ = node_->create_wall_timer(1000ms, std::bind(&IFSSIMRosWrapper::extraInfoTimerCb, this));
        track_publish_timer_ = node_->create_wall_timer(5000ms, std::bind(&IFSSIMRosWrapper::trackPublishCb, this)); // 0.2 Hz
    }
}

// === Timer callbacks ===

void IFSSIMRosWrapper::gpsTimerCb()
{
    if (!client_ || !client_->isConnected()) return;

    std::string resp = client_->sendCommand("getGpsData");
    if (resp.empty()) return;

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = vehicle_frame_id_;
    msg.latitude = client_->parseDouble(resp, "lat");
    msg.longitude = client_->parseDouble(resp, "lon");
    msg.altitude = client_->parseDouble(resp, "alt");
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    gps_pub_->publish(msg);
}

void IFSSIMRosWrapper::imuTimerCb()
{
    if (!client_ || !client_->isConnected()) return;

    std::string resp = client_->sendCommand("getImuData");
    if (resp.empty()) return;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = vehicle_frame_id_;
    // RPC server already outputs in ENU frame (m/s^2)
    msg.linear_acceleration.x = client_->parseDouble(resp, "ax");
    msg.linear_acceleration.y = client_->parseDouble(resp, "ay");
    msg.linear_acceleration.z = client_->parseDouble(resp, "az");
    msg.angular_velocity.x = client_->parseDouble(resp, "gx");
    msg.angular_velocity.y = client_->parseDouble(resp, "gy");
    msg.angular_velocity.z = client_->parseDouble(resp, "gz");

    imu_pub_->publish(msg);
}

void IFSSIMRosWrapper::gssTimerCb()
{
    if (!client_ || !client_->isConnected()) return;

    std::string resp = client_->sendCommand("getGroundSpeedSensorData");
    if (resp.empty()) return;

    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = vehicle_frame_id_;
    msg.twist.twist.linear.x = client_->parseDouble(resp, "vx");
    msg.twist.twist.linear.y = client_->parseDouble(resp, "vy");
    msg.twist.twist.linear.z = client_->parseDouble(resp, "vz");

    gss_pub_->publish(msg);
}

void IFSSIMRosWrapper::odomTimerCb()
{
    if (!client_ || !client_->isConnected()) return;

    std::string resp = client_->sendCommand("getCarState");
    if (resp.empty()) return;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = map_frame_id_;
    msg.child_frame_id = vehicle_frame_id_;

    // RPC server already outputs in ENU (meters)
    msg.pose.pose.position.x = client_->parseDouble(resp, "x");
    msg.pose.pose.position.y = client_->parseDouble(resp, "y");
    msg.pose.pose.position.z = client_->parseDouble(resp, "z");

    // Orientation
    msg.pose.pose.orientation.w = client_->parseDouble(resp, "qw");
    msg.pose.pose.orientation.x = client_->parseDouble(resp, "qx");
    msg.pose.pose.orientation.y = client_->parseDouble(resp, "qy");
    msg.pose.pose.orientation.z = client_->parseDouble(resp, "qz");

    odom_pub_->publish(msg);
}

void IFSSIMRosWrapper::lidarTimerCb()
{
    TcpClient* lidar_client = (client_lidar_ && client_lidar_->isConnected())
        ? client_lidar_.get() : client_.get();
    if (!lidar_client || !lidar_client->isConnected()) return;

    // Use binary protocol to get actual point cloud data
    std::vector<uint8_t> binaryData;
    std::string header = lidar_client->sendBinaryCommand("getLidarDataBinary", binaryData);

    if (header.empty() || binaryData.empty()) return;

    // Parse point count from header "PTS:N"
    int point_count = 0;
    size_t colonPos = header.find(':');
    if (colonPos != std::string::npos) {
        try { point_count = std::stoi(header.substr(colonPos + 1)); } catch (...) {}
    }

    if (point_count <= 0) return;

    // Build PointCloud2 message with actual data
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = vehicle_frame_id_;
    msg.height = 1;
    msg.width = point_count;
    msg.is_dense = true;
    msg.is_bigendian = false;

    // Define fields: x, y, z (float32 each)
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);

    // Copy actual point data into the message
    // Binary data is [x,y,z, x,y,z, ...] as float32, same layout as PointCloud2
    size_t expectedBytes = point_count * 3 * sizeof(float);
    if (binaryData.size() >= expectedBytes) {
        memcpy(msg.data.data(), binaryData.data(), expectedBytes);
    }

    lidar_pub_->publish(msg);
}

void IFSSIMRosWrapper::cameraTimerCb()
{
    TcpClient* cam_client = (client_camera_ && client_camera_->isConnected())
        ? client_camera_.get() : client_.get();
    if (!cam_client || !cam_client->isConnected()) return;

    for (const auto& cam_name : camera_names_) {
        auto it = camera_pubs_.find(cam_name);
        if (it == camera_pubs_.end()) continue;

        // Use binary protocol to get PNG image
        std::string command = "simGetImageBinary " + cam_name + " 0";
        std::vector<uint8_t> imageData;
        std::string header = cam_client->sendBinaryCommand(command, imageData);

        if (header.empty() || imageData.empty()) continue;

        // Parse image size from header "IMG:size"
        int imgSize = 0;
        size_t colonPos = header.find(':');
        if (colonPos != std::string::npos) {
            try { imgSize = std::stoi(header.substr(colonPos + 1)); } catch (...) {}
        }
        if (imgSize <= 0 || imageData.empty()) continue;

        // Publish as CompressedImage
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = node_->now();
        msg.header.frame_id = vehicle_frame_id_ + "/" + cam_name;
        msg.format = "png";
        msg.data.assign(imageData.begin(), imageData.end());

        it->second->publish(msg);
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
}

void IFSSIMRosWrapper::trackPublishCb()
{
    if (!client_ || !client_->isConnected()) return;

    std::string resp = client_->sendCommand("getRefereeState");
    if (resp.empty()) return;

    // Parse cone_positions array from response
    // Format: {"doo_counter":N,"cones":N,"laps":N,"lap_times":[...],"cone_positions":[{"x":1.0,"y":2.0,"color":0},...]}"
    fs_msgs::msg::Track msg;

    // Find cone_positions array
    size_t arr_start = resp.find("\"cone_positions\":[");
    if (arr_start == std::string::npos) return;
    arr_start = resp.find('[', arr_start);
    size_t arr_end = resp.find(']', arr_start);
    if (arr_end == std::string::npos) return;

    std::string arr = resp.substr(arr_start + 1, arr_end - arr_start - 1);
    if (arr.empty()) return;

    // Parse each cone object: {"x":1.0,"y":2.0,"color":0}
    size_t pos = 0;
    while (pos < arr.size()) {
        size_t obj_start = arr.find('{', pos);
        if (obj_start == std::string::npos) break;
        size_t obj_end = arr.find('}', obj_start);
        if (obj_end == std::string::npos) break;

        std::string obj = arr.substr(obj_start, obj_end - obj_start + 1);
        pos = obj_end + 1;

        // Parse x, y, color from the object
        double x = 0, y = 0;
        int color = 4; // UNKNOWN

        auto parseField = [&obj](const std::string& key) -> double {
            size_t kpos = obj.find("\"" + key + "\":");
            if (kpos == std::string::npos) return 0.0;
            kpos += key.size() + 3; // skip "key":
            return std::stod(obj.substr(kpos));
        };

        x = parseField("x");
        y = parseField("y");
        color = (int)parseField("color");

        fs_msgs::msg::Cone cone;
        cone.location.x = x;
        cone.location.y = y;
        cone.location.z = 0.0;
        cone.color = (uint8_t)color;
        msg.track.push_back(cone);
    }

    if (!msg.track.empty()) {
        track_pub_->publish(msg);
    }
}

void IFSSIMRosWrapper::staticTfCb()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = node_->now();
    tf.header.frame_id = map_frame_id_;
    tf.child_frame_id = vehicle_frame_id_;
    tf.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(tf);
}

// === Subscriber callbacks ===

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
