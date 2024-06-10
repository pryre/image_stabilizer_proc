#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace image_stabilizer_proc {

class Stabilizer : public rclcpp::Node
{
  public:
    Stabilizer(const rclcpp::NodeOptions & options);
  private:
    void _cb_camera(const image_transport::ImageTransport::ImageConstPtr& image, const image_transport::ImageTransport::CameraInfoConstPtr& info);

    rclcpp::Node::SharedPtr _ptr;
    std::string _parent_frame;
    std::string _child_frame;
    int64_t _sync_window;
    //Enabling for stabilization
    bool _stabilize_rotation;
    bool _stabilize_translation;
    //Time constants for "spring to center"
    double _spring_tau_rotation;
    double _spring_tau_translation;

    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    // image_transport::ImageTransport _it;
    image_geometry::PinholeCameraModel _model;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _proc_out;
    image_transport::Publisher _proc_out;
    image_transport::CameraSubscriber _cam_in;
};

};
