"""
rrt_connect_planner.py
-----------------------
Nav2 Global Planner Plugin interface.

THIS FILE CONTAINS NO PLANNING LOGIC.
All planning is done by RRTConnectCore in rrt_connect_core.py.

This files translates the costmap into a version compatible for the planning logic

  Nav2 gives us:
    - A costmap  
    - A start   
    - A goal     

  We give the core planner:
    - Obstacle rectangles  
    - Start/goal           
    - Map bounds           

  The core planner gives us:
    - A path               

  We give Nav2 back:
    - A nav_msgs/Path      
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_core.global_planner import GlobalPlanner
import time


from nav2_rrt_connect.rrt_connect_core import RRTConnectCore


def costmap_to_rectangles(costmap, lethal_threshold=253, inflation_metres=0.1):
    """
    Converts a Nav2 costmap into a list of obstacle rectangles.

    1. Read costmap metadata: resolution, origin, size.
    2. Scan every cell. Mark cells with cost >= lethal_threshold as obstacles.
    3. For each row, find horizontal runs of obstacle cells.
    4. Each run becomes one rectangle in world coordinates.
    5. Add a small inflation margin around each rectangle.

    Args:
        costmap           
        lethal_threshold  
        inflation_metres  

    Returns:
        rectangles 
        x_range    
        y_range    
    """
    # ── Read costmap metadata ────────────────────────────────────────────
    resolution = costmap.getResolution()    # metres per cell, e.g. 0.05
    origin_x   = costmap.getOriginX()       # real-world x of bottom-left cell
    origin_y   = costmap.getOriginY()       # real-world y of bottom-left cell
    width      = costmap.getSizeInCellsX()  # number of columns
    height     = costmap.getSizeInCellsY()  # number of rows

    # World bounds — derived from costmap, NOT hardcoded
    x_range = (origin_x, origin_x + width  * resolution)
    y_range = (origin_y, origin_y + height * resolution)

    # ── Scan costmap and build obstacle rectangles ───────────────────────
    rectangles = []
    m = inflation_metres

    for row in range(height):
        in_run    = False
        run_start = 0

        for col in range(width + 1):  # +1 to flush the last run
            is_obstacle = (
                col < width and
                costmap.getCostXY(col, row) >= lethal_threshold
            )

            if is_obstacle and not in_run:
                in_run    = True
                run_start = col

            elif not is_obstacle and in_run:
                in_run = False
                # Convert run from grid cells to world metres
                # Formula: world_coord = origin + cell_index × resolution
                wx = origin_x + run_start * resolution - m
                wy = origin_y + row       * resolution - m
                ww = (col - run_start)    * resolution + 2 * m
                wh = resolution                         + 2 * m
                rectangles.append([wx, wy, ww, wh])

    return rectangles, x_range, y_range


def waypoints_to_ros_path(waypoints, header):
    """
    Converts a list of (x,y) tuples into a nav_msgs/Path message.

    Args:
        waypoints 
        header    

    Returns:
        nav_msgs/Path
    """
    ros_path        = Path()
    ros_path.header = header

    for x, y in waypoints:
        pose                    = PoseStamped()
        pose.header             = header
        pose.pose.position.x    = float(x)
        pose.pose.position.y    = float(y)
        pose.pose.position.z    = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0   # identity quaternion
        ros_path.poses.append(pose)

    return ros_path


class RRTConnectPlanner(GlobalPlanner):
    """
    Nav2 Global Planner Plugin.

    PARAMETERS 
        step_size         
        max_iterations    
        goal_sample_rate  
        connect_tolerance 
        apf_C             
        apf_K             
        apf_R             
        lethal_threshold  
        inflation_metres  
    """

    def __init__(self):
        # Don't do real work here — ROS2 node not available yet
        self._node             = None
        self._costmap          = None
        self._tf               = None
        self._name             = ''
        self._logger           = None
        self._core             = None
        self._lethal_threshold = 253
        self._inflation_metres = 0.1

    def configure(self, node, name, tf, costmap):
        self._node    = node
        self._name    = name
        self._tf      = tf
        self._costmap = costmap
        self._logger  = node.get_logger()

        # Declare parameters — name, default value
        # If nav2_params.yaml has a value, it overrides the default
        def decl(param, default):
            node.declare_parameter(f'{name}.{param}', default)

        decl('step_size',         0.3)
        decl('max_iterations',    2000)
        decl('goal_sample_rate',  0.1)
        decl('connect_tolerance', 0.05)
        decl('apf_C',             10.0)
        decl('apf_K',             5.0)
        decl('apf_R',             0.5)
        decl('lethal_threshold',  253)
        decl('inflation_metres',  0.1)

        # Read parameters
        def get(param):
            return node.get_parameter(f'{name}.{param}').value

        self._lethal_threshold = get('lethal_threshold')
        self._inflation_metres = get('inflation_metres')

        # Create the core planner — all params from YAML, nothing hardcoded
        self._core = RRTConnectCore(
            step_size         = get('step_size'),
            K                 = get('apf_K'),
            R                 = get('apf_R'),
            C                 = get('apf_C'),
            max_iterations    = get('max_iterations'),
            goal_sample_rate  = get('goal_sample_rate'),
            connect_tolerance = get('connect_tolerance'),
        )

        self._logger.info(
            f'[{name}] RRTConnectPlanner configured | '
            f'step={get("step_size")} iter={get("max_iterations")} '
            f'K={get("apf_K")} R={get("apf_R")} C={get("apf_C")}'
        )

    def activate(self):
        self._logger.info(f'[{self._name}] activated.')

    def deactivate(self):
        self._logger.info(f'[{self._name}] deactivated.')

    def cleanup(self):
        self._core    = None
        self._costmap = None
        self._logger.info(f'[{self._name}] cleaned up.')

    def createPlan(self, start, goal):
        """
        Called by Nav2 every time a new navigation goal is set.

        Args:
            start
            goal  

        Returns:
            nav_msgs/Path
        """
        t0 = time.time()

        # ── Step 1: ROS messages → plain coordinates ─────────────────────
        start_xy = (start.pose.position.x, start.pose.position.y)
        goal_xy  = (goal.pose.position.x,  goal.pose.position.y)

        self._logger.info(
            f'[{self._name}] Planning '
            f'({start_xy[0]:.2f},{start_xy[1]:.2f}) → '
            f'({goal_xy[0]:.2f},{goal_xy[1]:.2f})'
        )

        # ── Step 2: Live costmap → obstacle rectangles ───────────────────
        # Read FRESH every call — costmap updates as sensors see new obstacles
        try:
            rectangles, x_range, y_range = costmap_to_rectangles(
                self._costmap,
                lethal_threshold = self._lethal_threshold,
                inflation_metres = self._inflation_metres,
            )
        except Exception as e:
            self._logger.error(f'[{self._name}] Costmap error: {e}')
            return Path()

        self._logger.debug(
            f'[{self._name}] {len(rectangles)} obstacle rects | '
            f'x={x_range} y={y_range}'
        )

        # ── Step 3: Call core planner ────────────────────────────────────
        waypoints = self._core.plan(
            start_xy, goal_xy, rectangles, x_range, y_range)

        elapsed = time.time() - t0

        # ── Step 4: Handle result ────────────────────────────────────────
        if waypoints is None:
            self._logger.warn(
                f'[{self._name}] No path found in {elapsed:.3f}s. '
                f'Try increasing max_iterations or step_size.'
            )
            return Path()   # Empty path = signal failure to Nav2

        self._logger.info(
            f'[{self._name}] Found {len(waypoints)} waypoints '
            f'in {elapsed:.3f}s'
        )

        # ── Step 5: Plain coordinates → ROS Path message ─────────────────
        return waypoints_to_ros_path(waypoints, goal.header)
