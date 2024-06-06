#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "image_stabilizer_proc/stabilizer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions();
  const auto node = std::make_shared<image_stabilizer_proc::Stabilizer>(options);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}