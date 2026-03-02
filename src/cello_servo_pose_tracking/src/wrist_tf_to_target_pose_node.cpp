#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

class WristTfToTargetPoseNode : public rclcpp::Node
{
public:
  WristTfToTargetPoseNode()
  : Node("wrist_tf_to_target_pose"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    wrist_frame_ = declare_parameter<std::string>("wrist_frame", "right_wrist");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "link6");
    output_topic_ = declare_parameter<std::string>("output_topic", "/xr_target_pose");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    reset_on_time_jump_ = declare_parameter<bool>("reset_on_time_jump", true);
    reset_on_forward_time_jump_ = declare_parameter<bool>("reset_on_forward_time_jump", false);
    time_jump_threshold_sec_ = declare_parameter<double>("time_jump_threshold_sec", 1.0);
    orientation_mode_ = declare_parameter<std::string>("orientation_mode", "full_offset");
    translation_scale_x_ = declare_parameter<double>("translation_scale_x", 1.0);
    translation_scale_y_ = declare_parameter<double>("translation_scale_y", 1.0);
    translation_scale_z_ = declare_parameter<double>("translation_scale_z", 1.0);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/xr_target_markers");

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&WristTfToTargetPoseNode::onTimer, this));
  }

private:
  void onTimer()
  {
    const auto current_time = now();
    if (have_last_time_ && reset_on_time_jump_)
    {
      const double dt = (current_time - last_time_).seconds();
      if (dt < -1e-3 || (reset_on_forward_time_jump_ && dt > time_jump_threshold_sec_))
      {
        calibrated_ = false;
        RCLCPP_WARN(
            get_logger(),
            "Detected ROS time jump (dt=%.3f s). Clearing wrist->ee calibration and recalibrating.",
            dt);
      }
    }
    last_time_ = current_time;
    have_last_time_ = true;

    geometry_msgs::msg::TransformStamped world_to_wrist_msg;
    geometry_msgs::msg::TransformStamped world_to_ee_msg;
    try
    {
      world_to_wrist_msg = tf_buffer_.lookupTransform(world_frame_, wrist_frame_, tf2::TimePointZero);
      world_to_ee_msg = tf_buffer_.lookupTransform(world_frame_, ee_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for TF: %s", ex.what());
      return;
    }

    tf2::Transform world_to_wrist_tf;
    tf2::Transform world_to_ee_tf;
    tf2::fromMsg(world_to_wrist_msg.transform, world_to_wrist_tf);
    tf2::fromMsg(world_to_ee_msg.transform, world_to_ee_tf);

    if (!calibrated_)
    {
      wrist_to_ee_offset_tf_ = world_to_wrist_tf.inverse() * world_to_ee_tf;
      world_translation_offset_ = world_to_ee_tf.getOrigin() - world_to_wrist_tf.getOrigin();
      calibrated_wrist_origin_ = world_to_wrist_tf.getOrigin();
      calibrated_ee_origin_ = world_to_ee_tf.getOrigin();
      double wr, wp, wy;
      double er, ep, ey;
      tf2::Matrix3x3(world_to_wrist_tf.getRotation()).getRPY(wr, wp, wy);
      tf2::Matrix3x3(world_to_ee_tf.getRotation()).getRPY(er, ep, ey);
      yaw_offset_ = std::atan2(std::sin(ey - wy), std::cos(ey - wy));
      calibrated_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Calibrated wrist->ee offset using startup transforms (%s -> %s, ee=%s), orientation_mode=%s",
          world_frame_.c_str(), wrist_frame_.c_str(), ee_frame_.c_str(), orientation_mode_.c_str());
    }

    tf2::Transform world_to_target_tf;
    if (orientation_mode_ == "source_rp_yaw_offset")
    {
      // Keep source wrist orientation, then apply a constant yaw offset in world frame.
      // This avoids Euler-angle singularities and preserves source roll/pitch behavior.
      tf2::Quaternion q_world_yaw_offset;
      q_world_yaw_offset.setRPY(0.0, 0.0, yaw_offset_);
      const tf2::Quaternion target_q = q_world_yaw_offset * world_to_wrist_tf.getRotation();

      // In this mode, keep calibrated translation offset while only orientation gets yaw offset.
      // Translation offset is applied in the wrist frame, so it follows wrist motion.
      tf2::Transform translation_only_offset(tf2::Quaternion::getIdentity(), wrist_to_ee_offset_tf_.getOrigin());
      world_to_target_tf = world_to_wrist_tf * translation_only_offset;
      world_to_target_tf.setRotation(target_q);
    }
    else if (orientation_mode_ == "world_translation_only")
    {
      // Apply translation in world frame so source vertical motion remains world-vertical.
      // Keep target orientation equal to the current EE orientation (no orientation tracking).
      const tf2::Vector3 wrist_delta_world = world_to_wrist_tf.getOrigin() - calibrated_wrist_origin_;
      const tf2::Vector3 mapped_delta_world(
          translation_scale_x_ * wrist_delta_world.x(),
          translation_scale_y_ * wrist_delta_world.y(),
          translation_scale_z_ * wrist_delta_world.z());
      world_to_target_tf = world_to_ee_tf;
      world_to_target_tf.setOrigin(calibrated_ee_origin_ + mapped_delta_world);
    }
    else
    {
      // Default behavior: preserve full 6D offset calibrated at startup.
      world_to_target_tf = world_to_wrist_tf * wrist_to_ee_offset_tf_;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = now();
    target_pose.header.frame_id = world_frame_;
    const auto target_tf_msg = tf2::toMsg(world_to_target_tf);
    target_pose.pose.position.x = target_tf_msg.translation.x;
    target_pose.pose.position.y = target_tf_msg.translation.y;
    target_pose.pose.position.z = target_tf_msg.translation.z;
    target_pose.pose.orientation = target_tf_msg.rotation;
    target_pub_->publish(target_pose);

    if (publish_markers_)
    {
      visualization_msgs::msg::MarkerArray marker_array;

      visualization_msgs::msg::Marker target_marker;
      target_marker.header.frame_id = world_frame_;
      target_marker.header.stamp = target_pose.header.stamp;
      target_marker.ns = "xr_target";
      target_marker.id = 0;
      target_marker.type = visualization_msgs::msg::Marker::SPHERE;
      target_marker.action = visualization_msgs::msg::Marker::ADD;
      target_marker.pose = target_pose.pose;
      target_marker.scale.x = 0.04;
      target_marker.scale.y = 0.04;
      target_marker.scale.z = 0.04;
      target_marker.color.r = 1.0f;
      target_marker.color.g = 0.2f;
      target_marker.color.b = 0.2f;
      target_marker.color.a = 0.9f;
      marker_array.markers.push_back(target_marker);

      const tf2::Vector3 ee_origin = world_to_ee_tf.getOrigin();
      const tf2::Vector3 target_origin = world_to_target_tf.getOrigin();

      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = world_frame_;
      line_marker.header.stamp = target_pose.header.stamp;
      line_marker.ns = "xr_target";
      line_marker.id = 1;
      line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.scale.x = 0.008;
      line_marker.color.r = 0.2f;
      line_marker.color.g = 0.9f;
      line_marker.color.b = 0.2f;
      line_marker.color.a = 0.9f;
      geometry_msgs::msg::Point p0;
      p0.x = ee_origin.x();
      p0.y = ee_origin.y();
      p0.z = ee_origin.z();
      geometry_msgs::msg::Point p1;
      p1.x = target_origin.x();
      p1.y = target_origin.y();
      p1.z = target_origin.z();
      line_marker.points.push_back(p0);
      line_marker.points.push_back(p1);
      marker_array.markers.push_back(line_marker);

      const double distance_m = (target_origin - ee_origin).length();
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = world_frame_;
      text_marker.header.stamp = target_pose.header.stamp;
      text_marker.ns = "xr_target";
      text_marker.id = 2;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = 0.5 * (p0.x + p1.x);
      text_marker.pose.position.y = 0.5 * (p0.y + p1.y);
      text_marker.pose.position.z = 0.5 * (p0.z + p1.z) + 0.05;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.05;
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 1.0f;
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << distance_m << " m";
      text_marker.text = ss.str();
      marker_array.markers.push_back(text_marker);

      marker_pub_->publish(marker_array);
    }
  }

  std::string world_frame_;
  std::string wrist_frame_;
  std::string ee_frame_;
  std::string output_topic_;
  double publish_rate_hz_{ 50.0 };
  bool reset_on_time_jump_{ true };
  bool reset_on_forward_time_jump_{ false };
  double time_jump_threshold_sec_{ 1.0 };
  std::string orientation_mode_;
  double translation_scale_x_{ 1.0 };
  double translation_scale_y_{ 1.0 };
  double translation_scale_z_{ 1.0 };
  double yaw_offset_{ 0.0 };
  tf2::Vector3 world_translation_offset_{ 0.0, 0.0, 0.0 };
  tf2::Vector3 calibrated_wrist_origin_{ 0.0, 0.0, 0.0 };
  tf2::Vector3 calibrated_ee_origin_{ 0.0, 0.0, 0.0 };
  bool publish_markers_{ true };
  std::string marker_topic_;
  bool calibrated_{ false };
  bool have_last_time_{ false };
  rclcpp::Time last_time_{ 0, 0, RCL_ROS_TIME };

  tf2::Transform wrist_to_ee_offset_tf_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WristTfToTargetPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
