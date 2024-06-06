#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace image_stabilizer_proc {

class Stabilizer : public rclcpp::Node
{
  public:
    Stabilizer(const rclcpp::NodeOptions & options);
  private:
    void topic_callback(const std_msgs::msg::String & msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

};
