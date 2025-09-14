#include "rclcpp/rclcpp.hpp"
#include "simple_planner/simple_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Nota: la classe è dentro al namespace simple_planner
  auto node = std::make_shared<simple_planner::SimplePlanner>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
