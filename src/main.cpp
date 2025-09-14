#include <rclcpp/rclcpp.hpp>
#include "simple_planner/simple_planner.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
