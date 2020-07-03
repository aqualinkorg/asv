#include <functional>
#include <csignal>
#include <thread>
#include <string>

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

// ZED
#include <sl/Camera.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

// Local
#include "recorder/periodic_scheduler.hpp"

class RecorderNode : public rclcpp::Node
{
public:
  RecorderNode();
  virtual ~RecorderNode();
  
  void update();

private:
  // Constants
  const std::string TOPIC_HEARTBEAT         = "heartbeat";
  const std::string TOPIC_CAMERA_STATUS     = "camera_status";
  const std::string TOPIC_GPS_STATUS        = "gps_status";
  const std::string TOPIC_STORAGE_STATUS    = "storage_status";
  const std::string TOPIC_SYSTEM_TIME       = "system_time";
  const std::string TOPIC_RECORDING_STATUS  = "recording_status";
  const std::string TOPIC_RECORDED_SAMPLES  = "recorded_samples";
  const std::string TOPIC_CAMERA_LEFT       = "camera_left/image_raw/compressed";
  const std::string TOPIC_CAMERA_TRIGGER    = "camera_trigger";

  const std::string TOPIC_TIME              = "time_unix_ms";
  const std::string TOPIC_GPS               = "gps";
  const std::string TOPIC_ATTITUDE          = "attitude";
  const std::string TOPIC_SPEED             = "speed"; 
  const std::string TOPIC_ENABLE_RECORDING  = "recording_enabled_target";


  // Attributes
  uint32_t                              _minimum_sample_period_msecs;
  std::string                           _storage_path;
  std::string                           _data_dir;
  std::chrono::steady_clock::time_point _last_pub_time;
  
  // State
  uint64_t  _time                 = 0;
  bool      _is_enabled           = false;
  bool      _got_gps              = false;
  bool      _got_time             = false;
  int64_t   _gps_samples          = 0;
  int64_t   _gps_samples_last_hb  = -1;
  int64_t   _frame_number         = 0;
  int64_t   _recorded_samples     = 0;

  double    _lat              = 0.0;
  double    _long             = 0.0;  
  double    _speed            = 0.0;
  double    _roll             = 0.0;
  double    _pitch            = 0.0;
  double    _yaw              = 0.0;

  // ZED Camera
  sl::InitParameters  _cam_params;
  sl::Camera          _camera;
  sl::Mat             _image;
  sl::Mat             _depth_raw;
  sl::Mat             _depth_image;

  // Image Data
  std::unique_ptr<uint8_t[]> _data_left;
  std::unique_ptr<uint8_t[]> _data_right;
  std::unique_ptr<uint8_t[]> _data_depth_raw;
  std::unique_ptr<uint8_t[]> _data_depth_image;

  std::vector<uint8_t> _jpeg_buffer;

  uint64_t                      _heartbeat_count;
  rclcpp::TimerBase::SharedPtr  _timer_hb;

  // ROS2 Publishers & Subscribers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_heartbeat;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_camera_status;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_gps_status;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_storage_status;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_system_time;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_recording_status;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 _pub_recorded_samples;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr     _pub_camera_left;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr                 _pub_camera_trigger;

  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr              _sub_time;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr        _sub_gps;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _sub_attitude;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _sub_speed;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                _sub_enable_recording;

  // Update timer
  PeriodicScheduler _scheduler;

  void publish_initial_status();
  void initialize_parameters();
  void initialize_storage();
  void create_subscriptions();
  void write_data();
};
