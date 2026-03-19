"""
rrt_connect_core.py
--------------------

"""

import math
import heapq
import numpy as np

try:
    from scipy.interpolate import CubicSpline
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False

try:
    from shapely.geometry import LineString, Point
    from shapely.geometry import box as shapely_box
    _SHAPELY_OK = True
except ImportError:
    _SHAPELY_OK = False


# ─────────────────────────────────────────────────────────────────────────────
# Node — the basic building block of an RRT tree
# ─────────────────────────────────────────────────────────────────────────────

class Node:
    """
    A single point in the RRT tree.

    """
    __slots__ = ('x', 'y', 'parent')

    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


# ─────────────────────────────────────────────────────────────────────────────
# Graph utilities — used for Dijkstra path pruning
# ─────────────────────────────────────────────────────────────────────────────

def build_graph(path):
    """
    Converts an ordered list of waypoints into a weighted adjacency dict.

    """
    graph = {}
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        graph.setdefault(a, {})
        graph.setdefault(b, {})
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        graph[a][b] = dist
        graph[b][a] = dist   # bidirectional — we can traverse either way
    return graph


def dijkstra(graph, start, goal):
    """
    Finds the shortest path from start to goal in the graph.
    """
    distances = {node: float('inf') for node in graph}
    previous  = {node: None         for node in graph}
    distances[start] = 0
    queue = [(0.0, start)]

    while queue:
        current_dist, current_node = heapq.heappop(queue)

        # Stop as soon as we reach the goal — no need to explore further
        if current_node == goal:
            break

        # Skip if we already found a shorter path to this node
        if current_dist > distances[current_node]:
            continue

        for neighbour, weight in graph[current_node].items():
            new_dist = current_dist + weight
            if new_dist < distances[neighbour]:
                distances[neighbour]  = new_dist
                previous[neighbour]   = current_node
                heapq.heappush(queue, (new_dist, neighbour))

    # Reconstruct path by walking backwards from goal using 'previous'
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = previous.get(node)

    # If we couldn't reach the goal, path[-1] won't be start
    if not path or path[-1] != start:
        return []

    return list(reversed(path))   # flip so it goes start → goal


# ─────────────────────────────────────────────────────────────────────────────
# Artificial Potential Field functions
# ─────────────────────────────────────────────────────────────────────────────

def attractive_force(position, goal, C):
    """
    Pulls the tree expansion toward the goal.
    """
    diff = np.array(goal) - np.array(position)
    dist = np.linalg.norm(diff)

    # At the goal exactly, force is zero — no divide by zero
    if dist < 1e-9:
        return np.zeros(2)

    # Force = gain/distance × unit_vector_toward_goal
    return (C / dist) * (diff / dist)


def repulsive_force(position, rectangles, K, R):
    """
    Pushes the tree expansion away from nearby obstacles

    """
    total_force = np.zeros(2)
    pos = np.array(position)

    for obs in rectangles:
        ox, oy, ow, oh = obs

        # Find the closest point on the rectangle boundary to our position.
        # np.clip projects our point onto the rectangle — if we're inside,
        # closest == position and distance == 0 (handled below).
        closest = np.array([
            np.clip(pos[0], ox, ox + ow),
            np.clip(pos[1], oy, oy + oh),
        ])

        dist = np.linalg.norm(pos - closest)

        # Only apply force if within influence radius AND not inside obstacle
        if 0 < dist < R:
            direction = pos - closest   # vector pointing away from obstacle
            # Gradient of repulsive potential field
            total_force += (K / 2.0) * (1.0 / dist - 1.0 / R) \
                           * direction / (dist ** 3)

    return total_force


# ─────────────────────────────────────────────────────────────────────────────
# Collision checking
# ─────────────────────────────────────────────────────────────────────────────

def is_collision_free(start, end, rectangles):
    """
    Returns True if the straight line from start to end
    does not pass through any obstacle rectangle.

    """
    if _SHAPELY_OK:
        # Precise geometric check using Shapely
        segment = LineString([start, end])
        for ox, oy, ow, oh in rectangles:
            if segment.intersects(shapely_box(ox, oy, ox + ow, oy + oh)):
                return False
        return True

    # ── Fallback: sample along segment ──────────────────────────────────
    sx, sy = start
    ex, ey = end
    length = math.hypot(ex - sx, ey - sy)
    if length < 1e-9:
        return True

    # Sample every 2cm along the segment
    n_steps = max(int(length / 0.02), 2)
    for i in range(n_steps + 1):
        t  = i / n_steps
        px = sx + t * (ex - sx)
        py = sy + t * (ey - sy)
        for ox, oy, ow, oh in rectangles:
            if ox <= px <= ox + ow and oy <= py <= oy + oh:
                return False
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Steering function — the heart of APF-guided RRT
# ─────────────────────────────────────────────────────────────────────────────

def steer(start, goal, step_size, rectangles, K, R, C, x_range, y_range):
    """
    Moves from 'start' toward 'goal' by at most step_size metres,
    biased by Artificial Potential Field forces.
    """
    s = np.array(start, dtype=float)
    g = np.array(goal,  dtype=float)

    direction = g - s
    dist      = np.linalg.norm(direction)

    # If already within one step of goal, just go there directly
    if dist < step_size:
        return tuple(g)

    # ── Geometric direction (always points toward goal) ──────────────────
    unit_direction = direction / dist

    # ── APF net force ────────────────────────────────────────────────────
    attr = attractive_force(s, g, C)
    rep  = repulsive_force(s, rectangles, K, R)

    # Safe normalisation — check magnitude before dividing
    rep_mag = np.linalg.norm(rep)
    rep_normalised = rep / rep_mag if rep_mag > 1e-9 else np.zeros(2)

    net        = attr - rep_normalised
    net_mag    = np.linalg.norm(net)
    # Fallback: if forces cancel perfectly, just go toward goal geometrically
    net_normalised = net / net_mag if net_mag > 1e-9 else unit_direction

    # ── Blend and step ────────────────────────────────────────────────────
    new_pos = s + unit_direction * step_size + net_normalised * step_size

    # ── Clip to map bounds ────────────────────────────────────────────────
    # x_range and y_range come from the costmap — dynamic, not hardcoded
    new_pos = np.clip(new_pos,
                      [x_range[0], y_range[0]],
                      [x_range[1], y_range[1]])

    return (float(new_pos[0]), float(new_pos[1]))


# ─────────────────────────────────────────────────────────────────────────────
# Path utilities
# ─────────────────────────────────────────────────────────────────────────────

def trace_to_root(node):
    """
    Walks from a leaf node back to the root of its tree,
    collecting coordinates along the way.
    """
    path = []
    current = node
    while current is not None:
        path.append((current.x, current.y))
        current = current.parent
    return path


def smooth_path(raw_path):
    """
    Takes the raw jagged RRT path and returns a smooth curve.
    """
    # Need at least 4 points to do anything meaningful
    if len(raw_path) < 4:
        return raw_path

    # ── Stage 1: Dijkstra pruning ────────────────────────────────────────
    graph   = build_graph(raw_path)
    pruned  = dijkstra(graph, raw_path[0], raw_path[-1])

    # If Dijkstra failed for some reason, fall back to raw path
    waypoints = pruned if len(pruned) >= 4 else raw_path

    # ── Stage 2: Cubic spline smoothing ─────────────────────────────────
    if not _SCIPY_OK:
        # scipy not installed — return pruned path without smoothing
        return waypoints

    xs, ys = zip(*waypoints)
    # Parameter t goes 0 → len-1, one value per waypoint
    t = np.arange(len(xs), dtype=float)
    # Dense sampling: 100 points or 5× the number of waypoints
    t_fine = np.linspace(0.0, float(len(xs) - 1),
                         max(100, len(xs) * 5))

    try:
        smooth_x = CubicSpline(t, xs)(t_fine)
        smooth_y = CubicSpline(t, ys)(t_fine)
        return list(zip(smooth_x.tolist(), smooth_y.tolist()))
    except Exception:
        # If spline fitting fails for any reason, return pruned path
        return waypoints


def path_length(path):
    """Euclidean length of a path in metres."""
    if not path or len(path) < 2:
        return 0.0
    return sum(
        math.hypot(path[i+1][0] - path[i][0],
                   path[i+1][1] - path[i][1])
        for i in range(len(path) - 1)
    )


# ─────────────────────────────────────────────────────────────────────────────
# RRTConnectCore — the main planner class
# ─────────────────────────────────────────────────────────────────────────────

class RRTConnectCore:
    """
    Bidirectional RRT-Connect planner with APF-guided steering.
    """

    def __init__(self,
                 step_size=0.3,
                 K=5.0,
                 R=0.5,
                 C=10.0,
                 max_iterations=2000,
                 goal_sample_rate=0.1,
                 connect_tolerance=0.05):
        """
        Args:
            step_size      
            K                 
            R                
            C                
            max_iterations   
            goal_sample_rate  
            connect_tolerance 
        """
        self.step_size         = step_size
        self.K                 = K
        self.R                 = R
        self.C                 = C
        self.max_iterations    = max_iterations
        self.goal_sample_rate  = goal_sample_rate
        self.connect_tolerance = connect_tolerance

    def plan(self, start_xy, goal_xy, rectangles, x_range, y_range):
        """
        Find a smooth collision-free path from start_xy to goal_xy.
        """
        # Initialise both trees — one root each at start and goal
        tree_a = [Node(start_xy[0], start_xy[1])]  # grows from start
        tree_b = [Node(goal_xy[0],  goal_xy[1])]   # grows from goal

        for _ in range(self.max_iterations):

            # ── Sample a random point ────────────────────────────────────
            # goal_sample_rate % of the time, aim directly at goal.
            # This biases the tree toward the goal so it doesn't wander
            # around randomly forever in open space.
            if np.random.random() < self.goal_sample_rate:
                sample = goal_xy
            else:
                sample = (
                    np.random.uniform(x_range[0], x_range[1]),
                    np.random.uniform(y_range[0], y_range[1]),
                )

            # ── Extend tree_a toward the sample ─────────────────────────
            nearest_a = self._nearest(tree_a, sample)
            new_pt_a  = steer(
                (nearest_a.x, nearest_a.y), sample,
                self.step_size, rectangles,
                self.K, self.R, self.C,
                x_range, y_range,
            )

            # Only add the new node if the path to it is collision-free
            if not is_collision_free(
                    (nearest_a.x, nearest_a.y), new_pt_a, rectangles):
                # Swap trees so the other tree gets to grow next
                tree_a, tree_b = tree_b, tree_a
                continue

            new_node_a = Node(new_pt_a[0], new_pt_a[1], parent=nearest_a)
            tree_a.append(new_node_a)

            # ── Try to connect tree_b to the new node in tree_a ─────────
            # This is the "Connect" step in RRT-Connect.
            # We find tree_b's closest node to new_node_a and try to
            # grow incrementally toward it until we connect or hit an obstacle.
            nearest_b = self._nearest(tree_b, new_pt_a)

            connected, node_b = self._connect(
                nearest_b, new_node_a, rectangles, x_range, y_range)

            if connected:
                # Both trees met — extract and return the full path
                raw_path = self._extract_path(new_node_a, node_b)
                return smooth_path(raw_path)

            # ── Balance the trees ────────────────────────────────────────
            # Always grow the smaller tree next iteration.
            # This keeps both trees roughly equal in size, which makes
            # the midpoint connection happen faster.
            if len(tree_b) < len(tree_a):
                tree_a, tree_b = tree_b, tree_a

        # Ran out of iterations without finding a path
        return None

    def _connect(self, node_b, target_node, rectangles, x_range, y_range):
        """
        Incrementally grows tree_b toward target_node one step at a time.

        """
        current = node_b
        target  = (target_node.x, target_node.y)

        while True:
            dist_to_target = math.hypot(
                current.x - target_node.x,
                current.y - target_node.y,
            )

            # Close enough — declare connection
            if dist_to_target <= self.connect_tolerance:
                return True, current

            # Take one step toward the target
            next_pt = steer(
                (current.x, current.y), target,
                self.step_size, rectangles,
                self.K, self.R, self.C,
                x_range, y_range,
            )

            # If the step hits an obstacle, stop trying
            if not is_collision_free(
                    (current.x, current.y), next_pt, rectangles):
                return False, current

            # Step is clear — add to tree_b and continue
            next_node = Node(next_pt[0], next_pt[1], parent=current)
            current   = next_node

    def _extract_path(self, node_a, node_b):
        """
        Joins the two trees into a single path.
        """
        path_from_start = trace_to_root(node_a)  # leaf → root (reversed below)
        path_from_goal  = trace_to_root(node_b)  # leaf → root

        # path_from_start is currently [node_a, ..., start]
        # Reverse it so it reads [start, ..., node_a]
        combined = list(reversed(path_from_start)) + path_from_goal
        return combined

    @staticmethod
    def _nearest(tree, point):
        """
        Finds the closest node in the tree to a given point
        """
        return min(
            tree,
            key=lambda n: math.hypot(n.x - point[0], n.y - point[1])
        )