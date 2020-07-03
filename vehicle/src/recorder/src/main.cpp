#include <recorder/recorder_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init( argc, argv );

  rclcpp::executors::SingleThreadedExecutor exec;
  
  spdlog::info("========================================");
  spdlog::info("STARTING recorder\n");

  auto node = std::make_shared<RecorderNode>();

  exec.add_node( node );

  while( rclcpp::ok() )
  {
    exec.spin();
  }
  
  spdlog::info("ENDING recorder\n");
  return 0;
}