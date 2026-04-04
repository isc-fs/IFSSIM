#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <fs_msgs/msg/control_command.hpp>
#include <fs_msgs/msg/go_signal.hpp>
#include <fs_msgs/msg/finished_signal.hpp>
#include <fs_msgs/msg/track.hpp>
#include <fs_msgs/msg/extra_info.hpp>
#include <fs_msgs/msg/wheel_states.hpp>
#include <fs_msgs/srv/reset.hpp>

#include "tcp_client.h"

#include <memory>
#include <string>
#include <map>
#include <vector>

/**
 * IFSSIM ROS2 Wrapper — connects to the IFSSIM TCP RPC server
 * and publishes sensor data as ROS2 topics.
 * Same topic names and message types as the original FSDS bridge.
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

    // Timer callbacks
    void gpsTimerCb();
    void imuTimerCb();
    void gssTimerCb();
    void odomTimerCb();
    void lidarTimerCb();
    void cameraTimerCb();
    void goSignalTimerCb();
    void extraInfoTimerCb();
    void staticTfCb();

    // Subscriber callbacks
    void controlCommandCb(const fs_msgs::msg::ControlCommand::SharedPtr msg);
    void finishedSignalCb(const fs_msgs::msg::FinishedSignal::SharedPtr msg);

    // Service callbacks
    void resetSrvCb(
        const std::shared_ptr<fs_msgs::srv::Reset::Request> request,
        std::shared_ptr<fs_msgs::srv::Reset::Response> response);

    // Node
    std::shared_ptr<rclcpp::Node> node_;

    // TCP clients (three for parallelism: main, lidar, camera)
    std::unique_ptr<TcpClient> client_;
    std::unique_ptr<TcpClient> client_lidar_;
    std::unique_ptr<TcpClient> client_camera_;

    // Connection params
    std::string host_;
    int port_;
    double timeout_sec_;

    // Frame IDs
    std::string map_frame_id_ = "fsds/map";
    std::string vehicle_frame_id_ = "fsds/FSCar";

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gss_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> camera_pubs_;
    rclcpp::Publisher<fs_msgs::msg::GoSignal>::SharedPtr go_signal_pub_;
    rclcpp::Publisher<fs_msgs::msg::ExtraInfo>::SharedPtr extra_info_pub_;
    rclcpp::Publisher<fs_msgs::msg::Track>::SharedPtr track_pub_;

    // Subscribers
    rclcpp::Subscription<fs_msgs::msg::ControlCommand>::SharedPtr control_cmd_sub_;
    rclcpp::Subscription<fs_msgs::msg::FinishedSignal>::SharedPtr finished_signal_sub_;

    // Services
    rclcpp::Service<fs_msgs::srv::Reset>::SharedPtr reset_srv_;

    // TF
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr gps_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr gss_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::TimerBase::SharedPtr go_signal_timer_;
    rclcpp::TimerBase::SharedPtr extra_info_timer_;
    rclcpp::TimerBase::SharedPtr static_tf_timer_;

    // Config
    std::string mission_name_ = "trackdrive";
    std::string track_name_ = "A";
    bool competition_mode_ = false;
    std::vector<std::string> camera_names_;
};
