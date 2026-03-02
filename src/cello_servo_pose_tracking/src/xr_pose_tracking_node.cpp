#include <chrono>
#include <cstdint>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/change_control_dimensions.hpp>
#include <moveit_msgs/srv/change_drift_dimensions.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace
{
constexpr auto kNodeName = "xr_pose_tracking";
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(kNodeName);

  const auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node, "moveit_servo");
  if (!servo_parameters)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load moveit_servo parameters");
    rclcpp::shutdown();
    return 1;
  }

  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to create planning scene");
    rclcpp::shutdown();
    return 1;
  }

  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startSceneMonitor(servo_parameters->monitored_planning_scene_topic);
  planning_scene_monitor->startWorldGeometryMonitor();
  if (servo_parameters->is_primary_planning_scene_monitor)
  {
    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, servo_parameters->monitored_planning_scene_topic);
  }

  auto pose_tracking =
      std::make_unique<moveit_servo::PoseTracking>(node, servo_parameters, planning_scene_monitor);
  pose_tracking->servo_->start();
  pose_tracking->servo_->setPaused(true);

  const bool zero_stamp_means_now = node->declare_parameter("zero_stamp_means_now", true);
  std::atomic<bool> got_first_target_pose{ false };
  std::mutex last_target_mutex;
  geometry_msgs::msg::PoseStamped last_target_pose;
  bool have_last_target_pose = false;
  auto target_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SensorDataQoS());
  auto xr_target_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr_target_pose", rclcpp::SensorDataQoS(),
      [node, target_pose_pub, zero_stamp_means_now, &got_first_target_pose, &last_target_mutex, &last_target_pose, &have_last_target_pose](
          const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        auto out = *msg;
        if (zero_stamp_means_now && out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0)
        {
          out.header.stamp = node->get_clock()->now();
        }
        target_pose_pub->publish(out);
        {
          std::lock_guard<std::mutex> lock(last_target_mutex);
          last_target_pose = out;
          have_last_target_pose = true;
        }
        got_first_target_pose.store(true);
      });

  const double position_tolerance = node->declare_parameter("position_tolerance", 0.005);
  const double orientation_tolerance = node->declare_parameter("orientation_tolerance", 0.01);
  const double target_pose_timeout = node->declare_parameter("target_pose_timeout", 0.25);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto control_dims_client =
      node->create_client<moveit_msgs::srv::ChangeControlDimensions>("change_control_dimensions");
  auto drift_dims_client =
      node->create_client<moveit_msgs::srv::ChangeDriftDimensions>("change_drift_dimensions");
  const bool control_x_translation = node->declare_parameter("control_x_translation", true);
  const bool control_y_translation = node->declare_parameter("control_y_translation", true);
  const bool control_z_translation = node->declare_parameter("control_z_translation", true);
  const bool control_x_rotation = node->declare_parameter("control_x_rotation", false);
  const bool control_y_rotation = node->declare_parameter("control_y_rotation", false);
  const bool control_z_rotation = node->declare_parameter("control_z_rotation", false);
  if (control_dims_client->wait_for_service(std::chrono::seconds(2)))
  {
    auto req = std::make_shared<moveit_msgs::srv::ChangeControlDimensions::Request>();
    req->control_x_translation = control_x_translation;
    req->control_y_translation = control_y_translation;
    req->control_z_translation = control_z_translation;
    req->control_x_rotation = control_x_rotation;
    req->control_y_rotation = control_y_rotation;
    req->control_z_rotation = control_z_rotation;
    auto future = control_dims_client->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
    {
      RCLCPP_INFO(node->get_logger(),
                  "Control dims configured: T[%d %d %d] R[%d %d %d], success=%s",
                  req->control_x_translation, req->control_y_translation, req->control_z_translation,
                  req->control_x_rotation, req->control_y_rotation, req->control_z_rotation,
                  future.get()->success ? "true" : "false");
    }
  }
  if (drift_dims_client->wait_for_service(std::chrono::seconds(2)))
  {
    auto req = std::make_shared<moveit_msgs::srv::ChangeDriftDimensions::Request>();
    req->drift_x_translation = false;
    req->drift_y_translation = false;
    req->drift_z_translation = false;
    req->drift_x_rotation = false;
    req->drift_y_rotation = false;
    req->drift_z_rotation = false;
    req->transform_jog_frame_to_drift_frame.rotation.w = 1.0;
    auto future = drift_dims_client->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
    {
      RCLCPP_INFO(node->get_logger(), "Drift dimensions configured: success=%s",
                  future.get()->success ? "true" : "false");
    }
  }

  const bool enable_startup_home = node->declare_parameter("enable_startup_home", true);
  const auto home_joint_names = node->declare_parameter<std::vector<std::string>>(
      "home_joint_names", { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" });
  const auto home_joint_positions =
      node->declare_parameter<std::vector<double>>("home_joint_positions", { 0.0, -0.6, 1.0, -0.8, 0.5, 0.0 });
  const double home_trajectory_time_sec = node->declare_parameter("home_trajectory_time_sec", 2.0);
  auto home_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_parameters->command_out_topic, 10);

  if (enable_startup_home)
  {
    if (home_joint_names.size() != home_joint_positions.size() || home_joint_names.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "home_joint_names/home_joint_positions size mismatch, skipping startup home");
    }
    else
    {
      for (int i = 0; rclcpp::ok() && i < 20 && home_pub->get_subscription_count() == 0; ++i)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }

      trajectory_msgs::msg::JointTrajectory home_msg;
      home_msg.header.stamp = node->get_clock()->now();
      home_msg.joint_names = home_joint_names;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = home_joint_positions;
      const auto total_ns = static_cast<int64_t>(home_trajectory_time_sec * 1e9);
      point.time_from_start.sec = static_cast<int32_t>(total_ns / 1000000000LL);
      point.time_from_start.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
      home_msg.points.push_back(point);
      home_pub->publish(home_msg);
      RCLCPP_INFO(node->get_logger(), "Published startup home trajectory on %s",
                  servo_parameters->command_out_topic.c_str());
      const auto wait_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(home_trajectory_time_sec + 0.5));
      rclcpp::sleep_for(wait_ns);
    }
  }

  const Eigen::Vector3d positional_tolerance(position_tolerance, position_tolerance, position_tolerance);
  rclcpp::WallRate retry_rate(20.0);
  moveit_servo::PoseTrackingStatusCode last_status = moveit_servo::PoseTrackingStatusCode::INVALID;
  auto last_log_time = node->now();
  auto last_error_log_time = node->now();
  bool servo_unpaused = false;

  while (rclcpp::ok())
  {
    if (!got_first_target_pose.load())
    {
      if ((node->now() - last_log_time).seconds() > 2.0)
      {
        RCLCPP_INFO(node->get_logger(), "Waiting for first /xr_target_pose before enabling pose tracking");
        last_log_time = node->now();
      }
      retry_rate.sleep();
      continue;
    }
    if (!servo_unpaused)
    {
      pose_tracking->servo_->setPaused(false);
      servo_unpaused = true;
      RCLCPP_INFO(node->get_logger(), "First target pose received, pose tracking enabled");
    }

    const auto status = pose_tracking->moveToPose(positional_tolerance, orientation_tolerance, target_pose_timeout);
    const auto now = node->now();
    if (status != last_status || (now - last_log_time).seconds() > 2.0)
    {
      RCLCPP_INFO(
          node->get_logger(), "PoseTracking status: %s",
          moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(status).c_str());
      last_status = status;
      last_log_time = now;
    }
    if (status == moveit_servo::PoseTrackingStatusCode::STOP_REQUESTED)
    {
      break;
    }

    // Debug: periodically print live EE->target error observed by this node.
    if ((now - last_error_log_time).seconds() > 0.5)
    {
      geometry_msgs::msg::PoseStamped target_copy;
      bool have_target = false;
      {
        std::lock_guard<std::mutex> lock(last_target_mutex);
        if (have_last_target_pose)
        {
          target_copy = last_target_pose;
          have_target = true;
        }
      }

      if (have_target)
      {
        planning_scene_monitor::LockedPlanningSceneRO scene_lock(planning_scene_monitor);
        const auto& current_state = scene_lock->getCurrentState();
        const auto ee_tf = current_state.getGlobalLinkTransform(servo_parameters->ee_frame_name);

        const Eigen::Vector3d ee_pos = ee_tf.translation();
        const Eigen::Vector3d target_pos(
            target_copy.pose.position.x, target_copy.pose.position.y, target_copy.pose.position.z);
        const double pos_err = (target_pos - ee_pos).norm();

        Eigen::Quaterniond q_ee(ee_tf.rotation());
        Eigen::Quaterniond q_target(
            target_copy.pose.orientation.w,
            target_copy.pose.orientation.x,
            target_copy.pose.orientation.y,
            target_copy.pose.orientation.z);
        q_ee.normalize();
        q_target.normalize();
        const double dot = std::abs(q_ee.dot(q_target));
        const double ori_err = 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));

        RCLCPP_INFO(
            node->get_logger(),
            "Debug error | frame=%s ee=%s pos_err=%.4f m ori_err=%.4f rad target_xyz=[%.3f %.3f %.3f] ee_xyz=[%.3f %.3f %.3f] status=%s",
            target_copy.header.frame_id.c_str(),
            servo_parameters->ee_frame_name.c_str(),
            pos_err,
            ori_err,
            target_pos.x(), target_pos.y(), target_pos.z(),
            ee_pos.x(), ee_pos.y(), ee_pos.z(),
            moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(status).c_str());
      }
      last_error_log_time = now;
    }

    // When no fresh target pose is available, loop at a controlled retry rate.
    if (status == moveit_servo::PoseTrackingStatusCode::NO_RECENT_TARGET_POSE ||
        status == moveit_servo::PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE)
    {
      retry_rate.sleep();
    }
  }

  (void)xr_target_pose_sub;
  (void)target_pose_pub;
  (void)home_pub;
  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
