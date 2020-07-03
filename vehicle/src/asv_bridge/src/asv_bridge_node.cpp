#include <asv_bridge/asv_bridge_node.hpp>

namespace qos {
    const rclcpp::QoS BEST_EFFORT                           = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).best_effort().durability_volatile();
    const rclcpp::QoS KEEP_LAST_RELIABLE_VOLATILE           = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().durability_volatile();
    const rclcpp::QoS KEEP_LAST_RELIABLE_TRANSIENT_LOCAL    = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();
    const rclcpp::QoS DEFAULT                               = BEST_EFFORT;
}

ASVBridgeNode::ASVBridgeNode()
    : Node( "asv_bridge" )
    , _mavchannel{ "/dev/ttyTHS1", 115200, 100 }
    , _pub_time{ create_publisher<std_msgs::msg::UInt64>( TOPIC_TIME, qos::BEST_EFFORT ) }
    , _pub_is_armed{ create_publisher<std_msgs::msg::Bool>( TOPIC_IS_ARMED, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_battery_state{ create_publisher<sensor_msgs::msg::BatteryState>( TOPIC_BATTERY_STATE, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_gps{ create_publisher<sensor_msgs::msg::NavSatFix>( TOPIC_GPS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_attitude{ create_publisher<geometry_msgs::msg::Vector3Stamped>( TOPIC_ATTITUDE, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_speed{ create_publisher<geometry_msgs::msg::Vector3Stamped>( TOPIC_SPEED, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
{
    // Register mavlink data handler
    _mavchannel.register_on_data( [this](){ 
    this->handle_mavlink();
    } );

    // Start accepting mavlink messages
    _mavchannel.connect();

    // Setup periodic timers
    _timer_hb = create_wall_timer(
        std::chrono::milliseconds( 1000 ),
        [this](){
            this->send_heartbeat();
            this->send_enable_telemetry();
            spdlog::info( "Sending heartbeat" );
        }
    );
}

// ===================================
// SENDERS

void ASVBridgeNode::send_heartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        255,
        0,
        &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );
    _mavchannel.write( msg );
}

void ASVBridgeNode::send_enable_telemetry()
{
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(
        255,
        0,
        &msg,
        1,                    // Target Component
        1,                    // Target Vehicle
        MAV_DATA_STREAM_ALL,  // Which messages to enable/disable
        5,                    // Message rate
        1                     // Enable/disable
    );
    _mavchannel.write( msg );
}

// ===================================================
// HANDLERS
void ASVBridgeNode::handle_mavlink()
{
    this->_mavchannel.read(_msg);
    switch( _msg.msgid )
    {
        case MAVLINK_MSG_ID_HEARTBEAT:              { handle_heartbeat();               break; }
        case MAVLINK_MSG_ID_SYS_STATUS:             { handle_sys_status();              break; }
        case MAVLINK_MSG_ID_SYSTEM_TIME:            { handle_system_time();             break; }
        case MAVLINK_MSG_ID_BATTERY_STATUS:         { handle_battery_status();          break; }
        case MAVLINK_MSG_ID_GPS_RAW_INT:            { handle_gps_raw_int();             break; }
        case MAVLINK_MSG_ID_VFR_HUD:                { handle_vfr_hud();                 break; }
        case MAVLINK_MSG_ID_ATTITUDE:               { handle_attitude();                break; }
        default: { break; }
    }
}

void ASVBridgeNode::handle_heartbeat()
{
    bool armed = mavlink_msg_heartbeat_get_base_mode(&_msg) & MAV_MODE_FLAG_SAFETY_ARMED;

    spdlog::info( "Heartbeat - Armed: {}", armed );

    std_msgs::msg::Bool msg;
    msg.data = armed;
    _pub_is_armed->publish( msg );
}

void ASVBridgeNode::handle_sys_status()
{
    double voltage = mavlink_msg_sys_status_get_voltage_battery( &_msg ) / 1000.0;
    double current = mavlink_msg_sys_status_get_current_battery( &_msg ) / 100.0;
    spdlog::info( "Battery State: {0}V | {1}A", voltage, current ); 

    sensor_msgs::msg::BatteryState msg;

    msg.voltage = voltage;
    msg.current = current;
    _pub_battery_state->publish( msg );
}

void ASVBridgeNode::handle_system_time()
{
    uint64_t time = mavlink_msg_system_time_get_time_unix_usec( &_msg );
    spdlog::info( "Time: {0}", time ); 

    std_msgs::msg::UInt64 msg;
    msg.data = time;

    _pub_time->publish( msg );
}

void ASVBridgeNode::handle_battery_status()
{
    int32_t mah_consumed = mavlink_msg_battery_status_get_current_consumed( &_msg );
    spdlog::info( "Battery Current Consumed: {0} mAH", mah_consumed ); 
}

void ASVBridgeNode::handle_gps_raw_int()
{
    using NavSatFix     = sensor_msgs::msg::NavSatFix;
    using NavSatStatus  = sensor_msgs::msg::NavSatStatus;

    // Get mavlink fields
    uint8_t fix_type    = mavlink_msg_gps_raw_int_get_fix_type( &_msg );
    int32_t lat         = mavlink_msg_gps_raw_int_get_lat( &_msg );
    int32_t lon         = mavlink_msg_gps_raw_int_get_lon( &_msg );
    int32_t alt         = mavlink_msg_gps_raw_int_get_alt( &_msg );

    // Convert to ROS2 representation
    NavSatFix msg;
    msg.header.stamp                = rclcpp::Clock().now();    // TODO: correlate to incoming GPS time
    msg.latitude                    = lat / 10000000.0; // Degrees
    msg.longitude                   = lon / 10000000.0; // Degrees
    msg.altitude                    = alt * 1000.0;     // Meters
    msg.position_covariance_type    = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    msg.status.service              = NavSatStatus::SERVICE_GPS;

    msg.status.status = NavSatStatus::STATUS_NO_FIX;
    switch( fix_type )
    {
        case GPS_FIX_TYPE_NO_GPS:   { msg.status.status = NavSatStatus::STATUS_NO_FIX;    break; }
        case GPS_FIX_TYPE_NO_FIX:   { msg.status.status = NavSatStatus::STATUS_NO_FIX;    break; }
        case GPS_FIX_TYPE_2D_FIX:   { msg.status.status = NavSatStatus::STATUS_FIX;       break; }
        case GPS_FIX_TYPE_3D_FIX:   { msg.status.status = NavSatStatus::STATUS_FIX;       break; }
        case GPS_FIX_TYPE_DGPS:     { msg.status.status = NavSatStatus::STATUS_SBAS_FIX;  break; }
        default: { break; }
    }

    // Publish
    _pub_gps->publish( msg );
}

void ASVBridgeNode::handle_vfr_hud()
{
    float groundspeed = mavlink_msg_vfr_hud_get_groundspeed( &_msg );

    // TODO: Stop abusing these vector3's
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp    = rclcpp::Clock().now();
    msg.vector.z        = static_cast<double>( groundspeed );

    _pub_speed->publish( msg );

}

void ASVBridgeNode::handle_attitude()
{
    float roll  = mavlink_msg_attitude_get_roll( &_msg );
    float pitch = mavlink_msg_attitude_get_roll( &_msg );
    float yaw   = mavlink_msg_attitude_get_roll( &_msg );

    // TODO: Stop abusing these vector3's
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp    = rclcpp::Clock().now();
    msg.vector.x        = static_cast<double>( roll );
    msg.vector.y        = static_cast<double>( pitch );
    msg.vector.z        = static_cast<double>( yaw );

    _pub_attitude->publish( msg );
}


// NOTES:

// HEARTBEAT ( #0 )
// SYS_STATUS ( #1 )
// SYSTEM_TIME ( #2 )
// BATTERY_STATUS ( #147 )
// POWER_STATUS ( #125 )

// GPS_RAW_INT ( #24 )
// GLOBAL_POSITION_INT ( #33 )
// LOCAL_POSITION_NED ( #32 )

// VFR_HUD ( #74 )
// ATTITUDE ( #30 )
// MAVLINK_MSG_ID_AHRS ( #163 )
// MAVLINK_MSG_ID_AHRS2 (178)
// MAVLINK_MSG_ID_AHRS3 (182)

// HWSTATUS (#165 ) 
// RAW_IMU ( #27 )
// SCALED_IMU2 ( #116 )
// SCALED_PRESSURE ( #29 )
// SCALED_PRESSURE2 ( #137 )
// MAVLINK_MSG_ID_MOUNT_STATUS ( #158 )
// MAVLINK_MSG_ID_EKF_STATUS_REPORT( #193 )
// MAVLINK_MSG_ID_VIBRATION (#241)
// MISSION_CURRENT ( #42 )
// NAV_CONTROLLER_OUTPUT ( #62 )