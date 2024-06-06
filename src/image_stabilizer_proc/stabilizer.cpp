#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "image_stabilizer_proc/stabilizer.hpp"

using std::placeholders::_1, std::placeholders::_2;

namespace image_stabilizer_proc {

Stabilizer::Stabilizer(const rclcpp::NodeOptions & options)
: Node("stabilizer", options),
_it(this->shared_from_this()),
_tf_buffer(std::make_unique<tf2_ros::Buffer>(this->get_clock()))
{
  _parent_frame = this->declare_parameter<std::string>("parent_frame", "map");
  _child_frame = this->declare_parameter<std::string>("child_frame", "base_link");

  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
  _cam_in = _it.subscribeCamera("camera/image", 10, std::bind(&Stabilizer::_cb_camera, this, _1, _2));
  _cam_out = _it.advertiseCamera("processed/image", 10);
}

void Stabilizer::_cb_camera(const image_transport::ImageTransport::ImageConstPtr& image, const image_transport::ImageTransport::CameraInfoConstPtr& info) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Camera message received");

  geometry_msgs::msg::TransformStamped t;
  try {
    t = _tf_buffer->lookupTransform(_parent_frame, _child_frame, image->header.stamp);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Could not transform " << _parent_frame << " to " << _child_frame << ": " << ex.what());
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  std::string orientation = "TEST";
  cv::putText(cv_ptr->image, orientation, cv::Point(200, 200), cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(255,0,0));

  _cam_out.publish(cv_ptr->toImageMsg(), info);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_stabilizer_proc::Stabilizer)