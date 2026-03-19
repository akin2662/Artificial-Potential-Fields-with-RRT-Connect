#!/usr/bin/env python3
"""
planner_service_node.py
------------------------
ROS2 service server that wraps RRTConnectCore.

This node:
  1. Subscribes to /map to get the occupancy grid
  2. Advertises the /compute_path service
  3. When called, converts the costmap to rectangles,
     runs RRTConnectCore.plan(), and returns the path

The C++ Nav2 plugin (ApfRrtcPlugin) calls this service
every time Nav2 needs a new global plan.

"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

# Our custom service generated from srv/ComputePath.srv
from apf_rrtc_plugin.srv import ComputePath

# The pure planning core — no ROS2 in there
from apf_rrtc_plugin.rrt_connect_core import RRTConnectCore

import math
import time


def occupancy_grid_to_rectangles(grid, lethal_threshold=65, inflation_metres=0.1):
    """
    Converts a nav_msgs/OccupancyGrid into obstacle rectangles.
    """
    res      = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    width    = grid.info.width
    height   = grid.info.height

    x_range = (origin_x, origin_x + width  * res)
    y_range = (origin_y, origin_y + height * res)

    rectangles = []
    m = inflation_metres
    data = grid.data  # flat array, row-major order

    for row in range(height):
        in_run    = False
        run_start = 0

        for col in range(width + 1):
            # Get cell value — data is row-major: index = row * width + col
            if col < width:
                val = data[row * width + col]
                # Treat unknown (-1) and occupied (>=threshold) as obstacles
                is_obstacle = (val >= lethal_threshold)
            else:
                is_obstacle = False  # flush last run

            if is_obstacle and not in_run:
                in_run    = True
                run_start = col
            elif not is_obstacle and in_run:
                in_run = False
                wx = origin_x + run_start * res - m
                wy = origin_y + row       * res - m
                ww = (col - run_start)    * res + 2 * m
                wh = res                        + 2 * m
                rectangles.append([wx, wy, ww, wh])

    return rectangles, x_range, y_range


def waypoints_to_path(waypoints, header):
    """Converts list of (x,y) tuples to nav_msgs/Path."""
    path        = Path()
    path.header = header

    for x, y in waypoints:
        pose                    = PoseStamped()
        pose.header             = header
        pose.pose.position.x    = float(x)
        pose.pose.position.y    = float(y)
        pose.pose.position.z    = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    return path


class PlannerServiceNode(Node):
    """
    ROS2 node that exposes the /compute_path service.
    """

    def __init__(self):
        super().__init__('apf_rrtc_planner_node')

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter('step_size',         0.3)
        self.declare_parameter('max_iterations',    2000)
        self.declare_parameter('goal_sample_rate',  0.1)
        self.declare_parameter('connect_tolerance', 0.05)
        self.declare_parameter('apf_C',             10.0)
        self.declare_parameter('apf_K',             5.0)
        self.declare_parameter('apf_R',             0.5)
        self.declare_parameter('lethal_threshold',  65)
        self.declare_parameter('inflation_metres',  0.1)
        self._cached_path = None
        self._cached_goal = None

        self.declare_parameter('random_seed', -1)
        seed = self.get_parameter('random_seed').value
        if seed >= 0:
            import numpy as np
            np.random.seed(seed)
            self.get_logger().info(f'Random seed set to {seed}')


        # ── Read parameters ───────────────────────────────────────────────
        def get(name):
            return self.get_parameter(name).value

        self._lethal_threshold = get('lethal_threshold')
        self._inflation_metres = get('inflation_metres')

        # ── Create the core planner ───────────────────────────────────────
        self._core = RRTConnectCore(
            step_size         = get('step_size'),
            K                 = get('apf_K'),
            R                 = get('apf_R'),
            C                 = get('apf_C'),
            max_iterations    = get('max_iterations'),
            goal_sample_rate  = get('goal_sample_rate'),
            connect_tolerance = get('connect_tolerance'),
        )

        # ── Store latest map ──────────────────────────────────────────────
        # We store the map when it arrives so service callbacks can use it
        self._map = None

        # ── Subscribe to /map ─────────────────────────────────────────────
        # QoS: transient local so we get the map even if we subscribe late
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos,
        )

        # ── Advertise the service ─────────────────────────────────────────
        self._service = self.create_service(
            ComputePath,
            '/compute_path',
            self._plan_callback,
        )

        self.get_logger().info(
            'apf_rrtc_planner_node started. '
            'Waiting for /map and /compute_path requests...'
        )

    def _map_callback(self, msg):
        """Store the latest map when it arrives."""
        self._map = msg
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} cells, '
            f'resolution={msg.info.resolution}m'
        )

    def _plan_callback(self, request, response):
        t0 = time.time()

        # ── Check map ─────────────────────────────────────────────────────
        if self._map is None:
            self.get_logger().error('No map received yet. Cannot plan.')
            response.success = False
            response.message = 'No map available'
            response.path    = Path()
            return response

        # ── Check cache ───────────────────────────────────────────────────
        current_goal = (
            request.goal.pose.position.x,
            request.goal.pose.position.y,
        )

        if self._cached_path is not None and self._cached_goal is not None:
            goal_dist = math.hypot(
                current_goal[0] - self._cached_goal[0],
                current_goal[1] - self._cached_goal[1],
            )
            if goal_dist < 0.1:
                self.get_logger().debug('Returning cached path.')
                response.success = True
                response.message = 'Cached path'
                response.path    = self._cached_path
                return response

        # ── Extract coordinates ───────────────────────────────────────────
        start_xy = (
            request.start.pose.position.x,
            request.start.pose.position.y,
        )
        goal_xy = (
            request.goal.pose.position.x,
            request.goal.pose.position.y,
        )

        self.get_logger().info(
            f'Planning: ({start_xy[0]:.2f},{start_xy[1]:.2f}) → '
            f'({goal_xy[0]:.2f},{goal_xy[1]:.2f})'
        )

        # ── Convert map to obstacles ──────────────────────────────────────
        try:
            rectangles, x_range, y_range = occupancy_grid_to_rectangles(
                self._map,
                lethal_threshold = self._lethal_threshold,
                inflation_metres = self._inflation_metres,
            )
        except Exception as e:
            self.get_logger().error(f'Map conversion failed: {e}')
            response.success = False
            response.message = f'Map conversion error: {e}'
            response.path    = Path()
            return response

        # ── Run planner ───────────────────────────────────────────────────
        waypoints = self._core.plan(
            start_xy, goal_xy, rectangles, x_range, y_range)

        elapsed = time.time() - t0

        # ── Handle failure ────────────────────────────────────────────────
        if waypoints is None:
            self._cached_path = None
            self._cached_goal = None
            self.get_logger().warn(
                f'No path found in {elapsed:.3f}s. '
                f'Try increasing max_iterations or step_size.'
            )
            response.success = False
            response.message = 'No path found within iteration limit'
            response.path    = Path()
            return response

        self.get_logger().info(
            f'Path found: {len(waypoints)} waypoints in {elapsed:.3f}s'
        )

        # ── Cache and return ──────────────────────────────────────────────
        self._cached_goal  = current_goal   
        self._cached_path  = waypoints_to_path(waypoints, request.goal.header)

        response.success = True
        response.message = f'Path found in {elapsed:.3f}s'
        response.path    = self._cached_path
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlannerServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
