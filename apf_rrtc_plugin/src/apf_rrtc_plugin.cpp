/*
 * apf_rrtc_plugin.cpp
 * --------------------
 * Implementation of the C++ Nav2 global planner plugin wrapper.
 */

#include "apf_rrtc_plugin/apf_rrtc_plugin.hpp"

#include <string>
#include <memory>
#include <chrono>

#include "nav2_util/node_utils.hpp"

namespace apf_rrtc_plugin
{

void ApfRrtcPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Store the node handle and plugin name
  node_  = parent;
  name_  = name;

  // Get a shared pointer to the node so we can create a client
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in ApfRrtcPlugin::configure");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] ApfRrtcPlugin configuring. Waiting for Python planner service...",
    name_.c_str());

  /*
   * Create a service client for the Python planner.
   */
  client_ = node->create_client<apf_rrtc_plugin::srv::ComputePath>(
    "/compute_path");

  /*
   * Wait for the Python service to be available.
   */
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[%s] Interrupted while waiting for /compute_path service.",
        name_.c_str());
      return;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "[%s] Waiting for /compute_path service (Python planner node)...",
      name_.c_str());
  }

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] ApfRrtcPlugin configured. Python planner service is ready.",
    name_.c_str());

  // Suppress unused variable warnings — costmap and tf are available
  // to the Python node via its own subscriptions
  (void)tf;
  (void)costmap_ros;
}

void ApfRrtcPlugin::cleanup()
{
  client_.reset();
  RCLCPP_INFO(logger_, "[%s] ApfRrtcPlugin cleaned up.", name_.c_str());
}

void ApfRrtcPlugin::activate()
{
  RCLCPP_INFO(logger_, "[%s] ApfRrtcPlugin activated.", name_.c_str());
}

void ApfRrtcPlugin::deactivate()
{
  RCLCPP_INFO(logger_, "[%s] ApfRrtcPlugin deactivated.", name_.c_str());
}

nav_msgs::msg::Path ApfRrtcPlugin::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();

  RCLCPP_INFO(
    logger_,
    "[%s] createPlan called: (%.2f, %.2f) -> (%.2f, %.2f)",
    name_.c_str(),
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x,  goal.pose.position.y);

  /*
   * Build the service request.
   * Just pass start and goal directly — the Python node handles
   * costmap reading, obstacle extraction, and planning.
   */
  auto request = std::make_shared<apf_rrtc_plugin::srv::ComputePath::Request>();
  request->start = start;
  request->goal  = goal;

  /*
   * Send the request asynchronously then wait for the response.
   */
  auto future = client_->async_send_request(request);

  // Wait up to SERVICE_TIMEOUT_SEC seconds for the Python node to respond
  auto status = future.wait_for(std::chrono::seconds(SERVICE_TIMEOUT_SEC));

  if (status != std::future_status::ready) {
    RCLCPP_WARN(
      logger_,
      "[%s] Python planner service timed out after %d seconds. "
      "Returning empty path.",
      name_.c_str(), SERVICE_TIMEOUT_SEC);
    return nav_msgs::msg::Path();
  }

  auto response = future.get();

  if (!response->success) {
    RCLCPP_WARN(
      logger_,
      "[%s] Python planner failed: %s. Returning empty path.",
      name_.c_str(),
      response->message.c_str());
    return nav_msgs::msg::Path();
  }

  RCLCPP_INFO(
    logger_,
    "[%s] Path received: %zu waypoints.",
    name_.c_str(),
    response->path.poses.size());

  /*
   * Return the path directly — the Python node already built it
   * as a proper nav_msgs/Path with the correct header and frame_id.
   */
  return response->path;
}

}  // namespace apf_rrtc_plugin

/*
 * PLUGINLIB_EXPORT_CLASS macro
 */
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(apf_rrtc_plugin::ApfRrtcPlugin, nav2_core::GlobalPlanner)
