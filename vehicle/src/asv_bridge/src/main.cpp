#include <asv_bridge/asv_bridge_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init( argc, argv );
  
  spdlog::info("========================================");
  spdlog::info("STARTING asv_bridge\n");

  auto node = std::make_shared<ASVBridgeNode>();

  rclcpp::spin( node );  

  spdlog::info("ENDING asv_bridge\n");
  return 0;
}