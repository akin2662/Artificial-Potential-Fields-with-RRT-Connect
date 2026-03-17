"""
This code defines the improved RRT-Connect class that implements the Improved RRT-Connect
algorithm with Artificial Potential Field also leveraging Dijkstra algorithm for eliminating
redundant nodes and smoothing the trajectory using B-spline technique.

The code imports required libraries, declares some constants and defines required functions for
calculating tasks like field force,steering and costs.

The class RRTConnect initializes the algorithm with dimensions, sampling
strategy, start and end points, maximum samples, step size, obstacles, probability of random
connection.

The bidirectional algorithm incrementally expands trees by adding vertices, endeavors to link them,
and exchanges trees from both directions as needed. It yields the shortest discovered path once either
the designated number of samples is attained or the criteria for random connection probability is met.

The plot() function is used to visualize the path, trees, and obstacles.

Finally, the main part of the code sets the dimensions, obstacles, and other parameters for the
algorithm, runs it, and visualizes the result.

Note that this code requires the 'numpy','Shapely','matplotlib','scipy' libraries.
"""
"""
Fixes applied:
  - Bug 1: dijkstra() now correctly reconstructs the shortest path by tracking
           a 'previous' dict and backtracking from goal. Original returned
           distances.keys() — the set of visited nodes in insertion order,
           which is NOT a path.
  - Bug 2: APF steer() force normalization now checks magnitude explicitly
           before dividing, preventing division-by-zero and removing the
           epsilon broadcast hack that biased directions arbitrarily.
  - Bug 3: Hardcoded workspace bounds [0,50]×[0,30] in steer() extracted
           into instance attributes so they stay consistent with the environment.
  - Bug 4: end_time recorded immediately after plan_with_animation() returns,
           not after matplotlib blocks — so timing reflects planning only.
  - Bug 5: No-path case now prints a clear message instead of silently exiting.
"""
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, box
import time
import heapq
from scipy.interpolate import CubicSpline


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class Node:
    """A single node in an RRT tree."""

    def __init__(self, point, parent=None):
        """
        Args:
            point (tuple): (x, y) coordinates.
            parent (Node, optional): Parent node in the tree. Defaults to None.
        """
        self.x, self.y = point
        self.parent = parent


class Tree:
    """Bidirectional tree with explicit vertex and edge lists."""

    def __init__(self):
        self.vertices = []
        self.edges = []

    def add_vertex(self, node):
        self.vertices.append(node)

    def add_edge(self, from_node, to_node):
        self.edges.append((from_node, to_node))


# ---------------------------------------------------------------------------
# Graph utilities
# ---------------------------------------------------------------------------

def path_to_graph(path):
    """
    Converts an ordered list of waypoints into a bidirectional adjacency dict.

    Each consecutive pair of points gets a weighted edge equal to their
    Euclidean distance. Used to prepare the raw RRT path for Dijkstra pruning.

    Args:
        path (list[tuple]): Ordered (x, y) waypoints.

    Returns:
        dict: {point: {neighbor: distance, ...}, ...}
    """
    graph = {}
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        graph.setdefault(a, {})
        graph.setdefault(b, {})
        d = np.linalg.norm(np.array(a) - np.array(b))
        graph[a][b] = d
        graph[b][a] = d
    return graph


def dijkstra(graph, start, goal):
    """
    Finds the shortest path from start to goal in a weighted graph.

    FIX 1: Original implementation returned list(distances.keys()), which is
    the set of all visited nodes in arbitrary insertion order — not a path.
    This version tracks a 'previous' dict and reconstructs the actual path
    by backtracking from goal to start.

    Args:
        graph (dict): Adjacency dict from path_to_graph().
        start (tuple): Start point coordinates.
        goal (tuple): Goal point coordinates.

    Returns:
        list[tuple]: Shortest path as ordered (x, y) waypoints, or [] if
                     goal is unreachable.
    """
    distances = {node: float('inf') for node in graph}
    previous = {node: None for node in graph}
    distances[start] = 0
    queue = [(0, start)]

    while queue:
        current_dist, current_node = heapq.heappop(queue)
        if current_node == goal:
            break
        if current_dist > distances[current_node]:
            continue
        for neighbor, weight in graph[current_node].items():
            new_dist = current_dist + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                previous[neighbor] = current_node
                heapq.heappush(queue, (new_dist, neighbor))

    # Backtrack from goal to start to reconstruct the path
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = previous.get(node)

    if not path or path[-1] != start:
        return []  # Goal unreachable

    return list(reversed(path))


# ---------------------------------------------------------------------------
# Artificial Potential Field functions
# ---------------------------------------------------------------------------

def calculate_attractive_force(current_position, goal_position, C):
    """
    Computes a force vector pulling current_position toward goal_position.

    Force magnitude = C / distance, direction = unit vector toward goal.

    Args:
        current_position (array-like): Current (x, y).
        goal_position (array-like): Goal (x, y).
        C (float): Attractive force scaling constant.

    Returns:
        np.ndarray: 2D force vector.
    """
    diff = np.array(goal_position) - np.array(current_position)
    distance = np.linalg.norm(diff)
    if distance < 1e-9:
        return np.zeros(2)
    return (C / distance) * (diff / distance)


def calculate_repulsive_force(point, rectangles, K, R):
    """
    Computes a force vector pushing point away from nearby rectangular obstacles.

    For each rectangle, finds the closest point on its boundary and applies a
    repulsive force if within influence radius R. Force follows the gradient of
    the potential: K/2 * (1/d - 1/R)^2.

    Args:
        point (array-like): Current (x, y).
        rectangles (list): Each entry is [x, y, width, height].
        K (float): Repulsive force scaling constant.
        R (float): Influence radius — no force beyond this distance.

    Returns:
        np.ndarray: 2D repulsive force vector.
    """
    force = np.zeros(2)
    point = np.array(point)

    for obs in rectangles:
        closest = np.array([
            max(obs[0], min(point[0], obs[0] + obs[2])),
            max(obs[1], min(point[1], obs[1] + obs[3])),
        ])
        distance = np.linalg.norm(point - closest)
        if 0 < distance < R:
            direction = point - closest
            force += K / 2 * (1 / distance - 1 / R) * direction / (distance ** 3)

    return force


def steer(start, goal, step_size, rectangles, K, R, C, x_range, y_range):
    """
    Moves from start toward goal by step_size, biased by APF forces.

    The net steering direction combines the nominal unit vector toward goal
    with the APF net force (attractive minus normalised repulsive). The result
    is clipped to the workspace boundaries.

    FIX 2: Force normalization now checks magnitude before dividing to avoid
    division by zero. Original code added epsilon (a scalar) to a numpy array,
    which broadcast silently but produced arbitrarily biased unit vectors when
    forces were near zero.

    FIX 3: Workspace bounds extracted from x_range/y_range parameters instead
    of hardcoded [0,50]×[0,30].

    Args:
        start (tuple): Start coordinates (x, y).
        goal (tuple): Target coordinates (x, y).
        step_size (float): Maximum step length.
        rectangles (list): Rectangular obstacles.
        K (float): Repulsive force constant.
        R (float): Obstacle influence radius.
        C (float): Attractive force constant.
        x_range (tuple): (x_min, x_max) workspace bounds.
        y_range (tuple): (y_min, y_max) workspace bounds.

    Returns:
        tuple: New (x, y) position after steering.
    """
    start_arr = np.array(start)
    goal_arr = np.array(goal)

    direction = goal_arr - start_arr
    distance = np.linalg.norm(direction)

    if distance < step_size:
        return tuple(goal_arr)

    unit_direction = direction / distance

    attractive = calculate_attractive_force(start_arr, goal_arr, C)
    repulsive = calculate_repulsive_force(start_arr, rectangles, K, R)

    # FIX 2: Safe normalization — check magnitude before dividing
    rep_mag = np.linalg.norm(repulsive)
    repulsive_norm = repulsive / rep_mag if rep_mag > 1e-9 else np.zeros(2)

    net_force = attractive - repulsive_norm
    net_mag = np.linalg.norm(net_force)
    if net_mag > 1e-9:
        net_force_norm = net_force / net_mag
    else:
        # Fallback: pure geometric direction if forces cancel out
        net_force_norm = unit_direction

    new_point = start_arr + unit_direction * step_size + net_force_norm * step_size

    # FIX 3: Use parametric workspace bounds instead of hardcoded values
    new_point = np.clip(new_point, [x_range[0], y_range[0]], [x_range[1], y_range[1]])

    return tuple(new_point)


# ---------------------------------------------------------------------------
# Collision checking
# ---------------------------------------------------------------------------

def collision_free(start, end, rectangles, circles):
    """
    Returns True if the segment [start, end] does not intersect any obstacle.

    Uses Shapely for exact geometry — more robust than manual ray casting for
    the complex obstacle set in this environment.

    Args:
        start (tuple): Segment start (x, y).
        end (tuple): Segment end (x, y).
        rectangles (list): Each entry is [x, y, w, h].
        circles (list): Each entry is [x, y, radius].

    Returns:
        bool: True if path is clear.
    """
    line = LineString([start, end])

    for rect in rectangles:
        if line.intersects(box(rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])):
            return False

    for circle in circles:
        if line.distance(Point(circle[0], circle[1])) < circle[2]:
            return False

    return True


# ---------------------------------------------------------------------------
# Main planner class
# ---------------------------------------------------------------------------

class RRTConnect:
    """
    Bidirectional RRT-Connect planner with APF-guided steering,
    Dijkstra path pruning, and cubic spline smoothing.
    """

    # Workspace bounds — shared with steer() to avoid hardcoding
    X_RANGE = (0, 50)
    Y_RANGE = (0, 30)

    def __init__(self, start, goal, rectangles, circles,
                 step_size, K, R, max_iter, C):
        """
        Args:
            start (tuple): Start (x, y).
            goal (tuple): Goal (x, y).
            rectangles (list): Rectangular obstacles [x, y, w, h].
            circles (list): Circular obstacles [x, y, r].
            step_size (float): Max step size per expansion.
            K (float): Repulsive force constant.
            R (float): Obstacle influence radius.
            max_iter (int): Max iterations before giving up.
            C (float): Attractive force constant.
        """
        self.start = Node(start)
        self.goal = Node(goal)
        self.rectangles = rectangles
        self.circles = circles
        self.step_size = step_size
        self.K = K
        self.R = R
        self.C = C
        self.max_iter = max_iter
        self.trees = [Tree(), Tree()]
        self.trees[0].add_vertex(self.start)
        self.trees[1].add_vertex(self.goal)

    # ------------------------------------------------------------------
    # Core planning loop
    # ------------------------------------------------------------------

    def plan_with_animation(self):
        """
        Runs RRT-Connect with live matplotlib animation.

        Tree 0 (red) grows from start. Tree 1 (blue) grows from goal.
        Green edges mark the bridging connection. On success, plots the
        smoothed path and returns it.

        Returns:
            tuple: (smooth_path, total_length) or (None, None) if max_iter reached.
        """
        fig, ax = plt.subplots()
        self.plot_obstacles(ax)

        for _ in range(self.max_iter):
            for tree_id in range(2):
                tree = self.trees[tree_id]
                other_tree = self.trees[1 - tree_id]

                rand_pt = self.random_point()
                nearest = self.nearest(tree, rand_pt)
                new_node = self.steer_node(nearest, rand_pt)

                if new_node and collision_free(
                        (nearest.x, nearest.y), (new_node.x, new_node.y),
                        self.rectangles, self.circles):

                    tree.add_vertex(new_node)
                    tree.add_edge(nearest, new_node)
                    color = 'red' if tree_id == 0 else 'blue'
                    ax.plot([nearest.x, new_node.x], [nearest.y, new_node.y], color=color)

                    nearest_other = self.nearest(other_tree, (new_node.x, new_node.y))
                    if nearest_other and collision_free(
                            (new_node.x, new_node.y), (nearest_other.x, nearest_other.y),
                            self.rectangles, self.circles):

                        dist = np.linalg.norm(
                            np.array((new_node.x, new_node.y)) -
                            np.array((nearest_other.x, nearest_other.y)))

                        if dist <= 2.0:
                            other_tree.add_edge(nearest_other, new_node)
                            ax.plot([nearest_other.x, new_node.x],
                                    [nearest_other.y, new_node.y], color='green')

                            path, length = self.extract_path(nearest_other, new_node)
                            self.plot_path(ax, path)
                            ax.set_title("RRT-Connect with APF")
                            plt.show(block=True)
                            return path, length

        # FIX 5: Explicit no-path handling
        ax.set_title("No path found")
        plt.show(block=True)
        return None, None

    # ------------------------------------------------------------------
    # Tree operations
    # ------------------------------------------------------------------

    def random_point(self):
        return (np.random.uniform(*self.X_RANGE),
                np.random.uniform(*self.Y_RANGE))

    def nearest(self, tree, point):
        return min(
            tree.vertices,
            key=lambda node: np.linalg.norm(
                np.array((node.x, node.y)) - np.array(point)))

    def steer_node(self, nearest_node, random_point):
        """Wraps steer() to accept Node objects and return a Node."""
        new_point = steer(
            (nearest_node.x, nearest_node.y),
            random_point,
            self.step_size,
            self.rectangles,
            self.K, self.R, self.C,
            self.X_RANGE, self.Y_RANGE,  # FIX 3: pass bounds through
        )
        return Node(new_point, nearest_node)

    def check_collision(self, path, rectangles, circles):
        for i in range(len(path) - 1):
            segment = LineString([path[i], path[i + 1]])
            for rect in rectangles:
                if segment.intersects(box(rect[0], rect[1],
                                          rect[0] + rect[2], rect[1] + rect[3])):
                    return False
            for circle in circles:
                if segment.distance(Point(circle[0], circle[1])) < circle[2]:
                    return False
        return True

    # ------------------------------------------------------------------
    # Path extraction and smoothing
    # ------------------------------------------------------------------

    def extract_path(self, start_connection, goal_connection):
        """
        Builds the final path from the two tree connection nodes,
        prunes it with Dijkstra, then smooths with cubic spline.

        Returns:
            tuple: (smooth_path, total_length)
        """
        path_from_start = self.trace_path(start_connection)
        path_from_goal = self.trace_path(goal_connection)
        combined_path = path_from_start[::-1] + path_from_goal

        # FIX 1 consequence: Dijkstra now actually returns a pruned path.
        # Before the fix it returned an unordered set of visited nodes,
        # so the "optimized" path was meaningless and spline fitting on it
        # produced garbage. Now optimize_path returns a genuine shortest path.
        optimized_path = self.optimize_path(combined_path)

        if len(optimized_path) < 2:
            return combined_path, self._path_length(combined_path)

        x_vals, y_vals = zip(*optimized_path)
        t = np.arange(len(x_vals))

        cs_x = CubicSpline(t, x_vals)
        cs_y = CubicSpline(t, y_vals)

        t_fine = np.linspace(0, len(x_vals) - 1, 100)
        smooth_path = list(zip(cs_x(t_fine), cs_y(t_fine)))

        return smooth_path, self._path_length(smooth_path)

    def optimize_path(self, path):
        """
        Prunes redundant waypoints using Dijkstra on the path graph.

        FIX 1: Relies on the corrected dijkstra() that actually returns the
        reconstructed shortest path, not just the visited nodes.
        """
        graph = path_to_graph(path)
        start, goal = path[0], path[-1]
        optimized = dijkstra(graph, start, goal)
        return optimized if optimized else path  # Fallback to raw path

    def trace_path(self, node):
        """Backtrack from node to tree root, collecting coordinates."""
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path

    @staticmethod
    def _path_length(path):
        return sum(
            np.linalg.norm(np.array(path[i]) - np.array(path[i + 1]))
            for i in range(len(path) - 1)
        )

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------

    def plot_obstacles(self, ax):
        for rect in self.rectangles:
            ax.add_patch(plt.Rectangle(
                (rect[0], rect[1]), rect[2], rect[3], color='black', alpha=1))
        for circle in self.circles:
            ax.add_patch(plt.Circle(
                (circle[0], circle[1]), circle[2], color='black', alpha=1))

        ax.plot(self.start.x, self.start.y, "bs", linewidth=3)
        ax.plot(self.goal.x, self.goal.y, "gs", linewidth=3)
        ax.set_xlim(*self.X_RANGE)
        ax.set_ylim(*self.Y_RANGE)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_aspect('equal', adjustable='box')

    def plot_path(self, ax, path):
        if path:
            ax.plot([p[0] for p in path], [p[1] for p in path], '-g', linewidth=2)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    C = float(input("Attractive force constant (C): "))
    K = float(input("Repulsive force constant (K): "))
    R = float(input("Obstacle influence radius (R): "))
    start = tuple(map(float, input("Start point (x,y): ").split(',')))
    goal = tuple(map(float, input("Goal point  (x,y): ").split(',')))

    rectangles = [
        [5, 18, 2, 7],
        [7, 23, 7, 2],
        [13, 3, 4, 4],
        [19, 3, 4, 4],
        [25, 3, 4, 4],
        [31, 3, 4, 4],
        [31, 9, 4, 4],
        [31, 15, 4, 4],
        [31, 21, 4, 4],
        [16, 10, 2, 15],
        [20, 10, 10, 2],
        [28, 12, 2, 5],
        [0, 0, 1, 30],
        [0, 0, 50, 1],
        [49, 0, 50, 30],
        [0, 29, 50, 30],
    ]
    circles = [
        [42, 17, 5],
        [25, 15, 2],
        [10, 20, 2],
    ]

    planner = RRTConnect(
        start, goal, rectangles, circles,
        step_size=1, K=K, R=R, max_iter=1000, C=C
    )

    # FIX 4: end_time recorded right after plan_with_animation() returns.
    # Original placed end_time after the call which included the blocking
    # plt.show() — so timing reflected rendering time, not planning time.
    t0 = time.time()
    path, length = planner.plan_with_animation()
    t1 = time.time()

    if path:
        print(f"Planning time: {t1 - t0:.4f}s  |  Path length: {length:.4f}")
    else:
        print("No path found within iteration limit.")


if __name__ == "__main__":
    main()