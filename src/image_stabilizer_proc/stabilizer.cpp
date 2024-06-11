#include <memory>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "image_stabilizer_proc/stabilizer.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

namespace image_stabilizer_proc {

namespace {
double right_hand_angle(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d n) {
  //det = n · (v1 × v2)
  const auto c = v1.cross(v2);
  const auto det = n.dot(c);
  //dot = v1 · v2
  const auto dot = v1.dot(v2);
  return atan2(det, dot);
}

double any_hand_angle(Eigen::Vector3d v1, Eigen::Vector3d v2) {
  //Computed as the angle between two vectors in an arbitrary orientation
  const auto c = v1.cross(v2);
  return right_hand_angle(v1, v2, c);
}


template<typename T>
T limit(const T& x, const T& x_min, const T& x_max) {
  return std::max(std::min(x, x_max), x_min);
}

cv::Mat chain_affine_transformation_mats(const cv::Mat& M0, const cv::Mat& M1) {
  assert(M0.rows == 2 && M0.cols == 3);
  assert(M1.rows == 2 && M1.cols == 3);

  cv::Mat Mx = (cv::Mat_<double>(1,3) << 0, 0, 1);
  cv::Mat M0_3, M1_3;
  cv::vconcat(M0, Mx, M0_3);
  cv::vconcat(M1, Mx, M1_3);
  
  //Multiply and drop the last row (cv::Rect is column major)
  return (M0_3 * M1_3)(cv::Rect(0, 0, 3, 2));
}
}

Stabilizer::Stabilizer(const rclcpp::NodeOptions & options)
: Node("stabilizer", options),
_ptr(rclcpp::Node::SharedPtr(this, [](auto *) {})),
_parent_frame(this->declare_parameter<std::string>("parent_frame", "map")),
_child_frame(this->declare_parameter<std::string>("child_frame", "base_link")),
_sync_window(this->declare_parameter<int64_t>("sync_window", 50)),
_stabilize_rotation(this->declare_parameter<bool>("stabilize_rotation", true)),
_stabilize_translation(this->declare_parameter<bool>("stabilize_translation", true)),
_spring_tau_rotation(this->declare_parameter<double>("spring_tau_rotation", 1.0)),
_spring_tau_jitter(this->declare_parameter<double>("spring_tau_jitter", 1.0)),
_deadzone_rotation(M_PI*this->declare_parameter<double>("deadzone_rotation", 1.0)/180.0),
_deadzone_translation(M_PI*this->declare_parameter<double>("deadzone_translation", 1.0)/180.0),
_last_frame(0, 0, RCL_ROS_TIME),
_last_rotation(Eigen::Quaterniond::Identity()),
_spring_correction(Eigen::Quaterniond::Identity()),
_tf_buffer(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
_tf_listener(std::make_shared<tf2_ros::TransformListener>(*_tf_buffer)),
_tf_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(*this)),
// _it(_ptr),
// _cam_out(_it.advertiseCamera("processed", 10)),
_proc_out(image_transport::create_publisher(this, "processed")),
_cam_in(image_transport::create_camera_subscription(this, "image", std::bind(&Stabilizer::_cb_camera, this, _1, _2), "raw"))
// _proc_out(this->create_publisher<sensor_msgs::msg::Image>("processed", rclcpp::SensorDataQoS()))
{
  //TODO: Can do lazy subscribing in future releases
}

void Stabilizer::_cb_camera(const image_transport::ImageTransport::ImageConstPtr& image, const image_transport::ImageTransport::CameraInfoConstPtr& info) {
  //Check if anyone is listening
  if(_proc_out.getNumSubscribers() < 1) {
    return;
  }

  // Verify camera is actually calibrated
  if (info->k[0] == 0.0) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Camera is publishing un-calibrated image!");
    return;
  }

  // Update the camera model
  _model.fromCameraInfo(info);

  //Short-cut early if stabilization is disabled
  if(!_stabilize_rotation && !_stabilize_translation) {
    _proc_out.publish(image);
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  try {
    t = _tf_buffer->lookupTransform(_parent_frame, _child_frame, image->header.stamp, _sync_window > 0 ? rclcpp::Duration::from_nanoseconds(_sync_window*1000000) : rclcpp::Duration::from_nanoseconds(0) );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Could not transform " << _parent_frame << " to " << _child_frame << ": " << ex.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(image);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  // q_c as the camera rotation from base_footprint (q_0) to base_link
  // q_t as the "angular translation" from q_0
  // q_r as the "roll" rotation around the x-axis
  // Such that:
  //  q_0 = (1, 0, 0, 0) | Or some initial other reference
  //  q_a = q_t * q_0
  //  q_c = q_r * q_a
  //  q_c = q_r * q_t * q_0

  Eigen::Quaterniond q_c;
  // tf2::convert(t.transform.rotation, q_c);
  tf2::fromMsg(t.transform.rotation, q_c);

  //Spring filtering for "return to center"
  const auto delta = (rclcpp::Time(image->header.stamp) - _last_frame).seconds();
  _last_frame = image->header.stamp;

  if(_spring_tau_jitter > 0) {
    const auto alpha = limit((1.0/_spring_tau_jitter) * delta, 0.0, 1.0);
    q_c = _last_rotation.slerp(alpha, q_c).normalized();
  }
  _last_rotation = q_c;

  // _spring_correction = Eigen::Quaterniond::Identity();
  if(_spring_tau_rotation > 0) {
    const auto alpha = limit((1.0/_spring_tau_rotation)*delta, 0.0, 1.0);
    // RCLCPP_INFO_STREAM(this->get_logger(), "tau_r: " << _spring_tau_rotation << "; alpha: " << alpha);
    _spring_correction = _spring_correction.slerp(alpha, q_c).normalized();
    // _spring_correction = spring_delta * _spring_correction;

    q_c = _spring_correction.inverse() * q_c;
  }

  geometry_msgs::msg::TransformStamped t_c;
  t_c.header.stamp = image->header.stamp;
  t_c.header.frame_id = _parent_frame;
  t_c.child_frame_id = image->header.frame_id + "_stabilized";
  t_c.transform.rotation = tf2::toMsg(q_c);
  _tf_broadcaster->sendTransform(t_c);

  //Calculate the "translation"/(yaw,pitch) rotation
  const auto R_c =  q_c.toRotationMatrix();
  //TODO: Check to make sure that col(0) != UnitZ
  const auto y_t = Eigen::Vector3d::UnitZ().cross(R_c.col(0));
  const auto z_t = R_c.col(0).cross(y_t);
  Eigen::Matrix3d R_t;
  R_t << R_c.col(0), y_t, z_t;

  auto correction_angle = 0.0;
  const auto center = cv::Point( cv_ptr->image.cols/2, cv_ptr->image.rows/2 );
  if(_stabilize_rotation) {
    //Calculate the "roll" angle between the two 'y' vectors
    //TODO: this will probably need to be ajusted if 'y' is not used above
    // const auto q_r = Eigen::Quaterniond::FromTwoVectors(y_t, R_c.col(1));
    // const Eigen::AngleAxisd a_r(q_r);
    const auto a_r = right_hand_angle(y_t, R_c.col(1), R_c.col(0));
    // RCLCPP_INFO_STREAM(this->get_logger(), "a_r: " << a_r);

    if(_deadzone_rotation <= 0 || std::fabs(a_r) > _deadzone_rotation) {
      correction_angle = -180.0*a_r/M_PI; //Rotation angle is measured clockwise around axis, cv takes positive angle as anticlockwise
    }
  }

  //Get the translation warp affine
  double correction_x = 0.0;
  double correction_y = 0.0;
  const auto translation_angle = any_hand_angle(Eigen::Vector3d::UnitX(), R_c.col(0));
  const bool is_in_translation_deadzone = _deadzone_translation > 0 && translation_angle < _deadzone_translation;

  if(_stabilize_translation && !is_in_translation_deadzone) {
    // const auto center
    double fov_xd, fov_yd, focal_length, aspect_ratio;
    cv::Point2d principle;
    cv::calibrationMatrixValues(_model.intrinsicMatrix(), cv_ptr->image.size(), 0, 0, fov_xd, fov_yd, focal_length, principle, aspect_ratio);
    // RCLCPP_INFO_STREAM(this->get_logger(), "fov_xd: " << fov_xd << "; fov_yd: " << fov_yd);

    //Get ratio of angle per pixel
    const auto a_px = (M_PI*fov_xd/180.0) / cv_ptr->image.cols;
    const auto a_py = (M_PI*fov_yd/180.0) / cv_ptr->image.rows;

    //Angle between the X axis and the center of image
    const auto axc = right_hand_angle(Eigen::Vector3d::UnitY(), y_t, Eigen::Vector3d::UnitZ());
    //Normalized Y offset is the same as the rotation X-vector in the Z-axis
    const auto ayc = right_hand_angle(Eigen::Vector3d::UnitZ(), z_t, Eigen::Vector3d::UnitY());

    correction_x = -axc / a_px; //x-axis is aligned to image frame (inverse needed)
    correction_y = ayc / a_py; //y-axis is inverted to image frame (no inverse needed)
    // RCLCPP_INFO_STREAM(this->get_logger(), "correction_x: " << correction_x << "; correction_y: " << correction_y);
  }

  // RCLCPP_INFO_STREAM(this->get_logger(), "a_r: " << correction_angle << "; a_t: " << 180*translation_angle/M_PI);

  //Short-cut early if stabilization is disabled
  if(correction_angle == 0 && correction_x == 0 && correction_y == 0) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Image in stabilization deadzone");
    _proc_out.publish(image);
    return;
  }

  //Get the rotational warp affine
  const auto Mr = cv::getRotationMatrix2D( center, correction_angle, 1.0 );
  cv::Mat Mt = (cv::Mat_<double>(2,3) << 1, 0, correction_x, 0, 1, correction_y);
  // https://stackoverflow.com/questions/75388906/how-to-rotate-and-translate-an-image-with-opencv-without-losing-off-screen-data
  const auto M = chain_affine_transformation_mats(Mt, Mr);

  // RCLCPP_INFO_STREAM(this->get_logger(), "Mr: " << Mr);
  // RCLCPP_INFO_STREAM(this->get_logger(), "Mt: " << Mt);
  // RCLCPP_INFO_STREAM(this->get_logger(), "M: " << M);
  //Create the destination image
  cv::Mat stab(cv_ptr->image.size(), cv_ptr->image.type());
  cv::warpAffine(cv_ptr->image, stab, M, stab.size());


  // // Draw an example circle on the video stream
  // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  // std::string orientation = "TEST";
  // cv::putText(cv_ptr->image, orientation, cv::Point(200, 200), cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(255,0,0));
  // auto b = cv_bridge::CvImageConstPtr(image->header, sensor_msgs::image_encodings::RGB8,);

// Allocate new rectified image message
  sensor_msgs::msg::Image::SharedPtr stab_msg =
    cv_bridge::CvImage(image->header, image->encoding, stab).toImageMsg();

  _proc_out.publish(stab_msg);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_stabilizer_proc::Stabilizer)