#ifndef APF_RRTC_PLUGIN__APF_RRTC_PLUGIN_HPP_
#define APF_RRTC_PLUGIN__APF_RRTC_PLUGIN_HPP_

/*
 * apf_rrtc_plugin.hpp
 * --------------------
 * Header for the C++ Nav2 global planner plugin wrapper.
 *
 * This class implements nav2_core::GlobalPlanner — the interface
 * that Nav2's pluginlib can discover and load.
 */

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

// Our custom service — generated from srv/ComputePath.srv
#include "apf_rrtc_plugin/srv/compute_path.hpp"

namespace apf_rrtc_plugin
{

class ApfRrtcPlugin : public nav2_core::GlobalPlanner
{
public:
  ApfRrtcPlugin() = default;
  ~ApfRrtcPlugin() = default;

  /*
   * configure() — called once by Nav2 at startup.
   * We create a service client here that connects to the Python node.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /*
   * cleanup() — release resources on shutdown.
   */
  void cleanup() override;

  /*
   * activate() / deactivate() — lifecycle transitions.
   */
  void activate() override;
  void deactivate() override;

  /*
   * createPlan() — called by Nav2 every time a new goal is set.
   * This calls the Python service and returns the resulting path.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // ROS2 node handle — needed to create the service client
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Plugin instance name (e.g. "GridBased") — used for logging
  std::string name_;

  // Service client — calls the Python planner node
  rclcpp::Client<apf_rrtc_plugin::srv::ComputePath>::SharedPtr client_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("ApfRrtcPlugin")};

  // How long to wait for the Python service to respond (seconds)
  // Increase this if planning takes longer than expected
  static constexpr int SERVICE_TIMEOUT_SEC = 20;
};

}  // namespace apf_rrtc_plugin

#endif  // APF_RRTC_PLUGIN__APF_RRTC_PLUGIN_HPP_
