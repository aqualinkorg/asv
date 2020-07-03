#include <functional>
#include <csignal>
#include <thread>
#include <string>

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// Mavlink
#include <mavchannel/mavchannel.hpp>

class ASVBridgeNode : public rclcpp::Node
{
public:
    ASVBridgeNode();
    virtual ~ASVBridgeNode() = default;

private:
    const std::string TOPIC_TIME            = "time_unix_ms";
    const std::string TOPIC_IS_ARMED        = "is_armed";
    const std::string TOPIC_BATTERY_STATE   = "battery_state";
    const std::string TOPIC_GPS             = "gps";
    const std::string TOPIC_ATTITUDE        = "attitude";
    const std::string TOPIC_SPEED           = "speed"; 

    // Mavlink
    mr::Mavchannel      _mavchannel;
    mavlink_message_t   _msg;

    // ROS2
    rclcpp::TimerBase::SharedPtr _timer_hb;

    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr               _pub_time;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 _pub_is_armed;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr      _pub_battery_state;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr         _pub_gps;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  _pub_attitude;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  _pub_speed;

    // Senders
    void send_heartbeat();
    void send_enable_telemetry();

    // Handlers
    void handle_mavlink();
    void handle_heartbeat();
    void handle_sys_status();
    void handle_system_time();
    void handle_battery_status();
    void handle_gps_raw_int();
    void handle_vfr_hud();
    void handle_attitude();
};
