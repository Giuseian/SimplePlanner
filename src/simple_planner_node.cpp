#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("simple_planner"), "Simple planner node started!");
  rclcpp::spin(std::make_shared<rclcpp::Node>("simple_planner_node"));
  rclcpp::shutdown();
  return 0;
}
