#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace image_stabilizer_proc {

class Stabilizer : public rclcpp::Node
{
  public:
    Stabilizer(const rclcpp::NodeOptions & options);
  private:
    void _cb_camera(const image_transport::ImageTransport::ImageConstPtr& image, const image_transport::ImageTransport::CameraInfoConstPtr& info);
    image_transport::ImageTransport _it;
    image_transport::CameraSubscriber _cam_in;
    image_transport::CameraPublisher _cam_out;
    
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::string _parent_frame;
    std::string _child_frame;
};

};
