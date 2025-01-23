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

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, box
import time
import heapq
from scipy.interpolate import CubicSpline


class Node:
    """Represents a single node in the tree."""
    def __init__(self, point, parent=None):
        """
        Initializes a Node object with coordinates and parent.

        Args:
            point (tuple): A tuple representing the (x, y) coordinates of the node.
            parent (Node, optional): The parent node in the path. Defaults to None.
        """

        self.x, self.y = point
        self.parent = parent

class Tree:
    """Defines a tree structure for the RRT-Connect algorithm with vertices and edges."""
    def __init__(self):
        """
        Initializes a Tree object with empty lists of vertices and edges.
        """
        self.vertices = []
        self.edges = []

    def add_vertex(self, node):
        self.vertices.append(node)

    def add_edge(self, from_node, to_node):
        self.edges.append((from_node, to_node))

def path_to_graph(path):
    """
    Converts a path represented as a list of points into a graph dictionary.

    Args:
        path (list): A list of tuples where each tuple represents a point (x, y).

    Returns:
        dict: A dictionary where each key is a point and values are dictionaries of neighboring points with distances.
    """

    graph = {}
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        if start not in graph:
            graph[start] = {}
        if end not in graph:
            graph[end] = {}
        distance = np.linalg.norm(np.array(start) - np.array(end))
        graph[start][end] = distance
        graph[end][start] = distance  # Assuming bidirectional edges
    return graph

def dijkstra(graph, start, goal):
    """
    Finds the shortest path between two nodes using Dijkstra's algorithm.

    Args:
        graph (dict): A graph dictionary where each key is a node and values are neighbors with distances.
        start (tuple): A tuple representing the starting point coordinates (x, y).
        goal (tuple): A tuple representing the goal point coordinates (x, y).

    Returns:
        list: A list of visited nodes during the path search.
    """

    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    queue = [(0, start)]
    while queue:
        current_distance, current_node = heapq.heappop(queue)
        if current_node == goal:
            break
        if current_distance > distances[current_node]:
            continue
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))


    return list(distances.keys())


def calculate_attractive_force(current_position, goal_position, C):
    """
    Calculates the attractive force from the current position toward the goal.

    Args:
        current_position (array-like): A tuple or list representing the current point (x, y).
        goal_position (array-like): A tuple or list representing the goal point (x, y).
        C (float): The attractive force scaling constant.

    Returns:
        numpy.ndarray: A NumPy array representing the calculated attractive force vector.
    """

    dx = goal_position[0] - current_position[0]
    dy = goal_position[1] - current_position[1]
    distance = np.sqrt(dx**2 + dy**2)
    force = C / distance
    direction = np.array([dx, dy]) / distance
    return force * direction

def calculate_repulsive_force(point, rectangles, K, R):
    """
    Calculates the repulsive force exerted by rectangular obstacles.

    Args:
        point (array-like): A tuple or list representing the current point (x, y).
        rectangles (list): A list of rectangle definitions where each item is [x, y, width, height].
        K (float): The repulsive force scaling constant.
        R (float): The radius of influence of the repulsive force.

    Returns:
        numpy.ndarray: A NumPy array representing the calculated repulsive force vector.
    """

    force = np.zeros(2)
    for obs in rectangles:
        closest_point = np.array([
            max(obs[0], min(point[0], obs[0] + obs[2])),
            max(obs[1], min(point[1], obs[1] + obs[3]))
        ])
        distance = np.linalg.norm(point - closest_point)
        if distance < R and distance > 0:  # Avoid divide-by-zero
            direction = point - closest_point
            force += (K / 2 * (1 / distance - 1 / R) * direction / np.power(distance, 3))
    return force

def steer(start, goal, step_size, rectangles, K, R,C):
    """
    Steer from the start to the goal considering attractive and repulsive forces.

    Args:
        start (tuple): The starting point coordinates (x, y).
        goal (tuple): The goal point coordinates (x, y).
        step_size (float): The maximum step size between points.
        rectangles (list): A list of rectangle obstacles, where each item is [x, y, width, height].
        K (float): The repulsive force scaling constant.
        R (float): The radius of influence of the repulsive force.
        C (float): The attractive force scaling constant.

    Returns:
        tuple: The new coordinates of the point after steering.
    """

    direction = np.array(goal) - np.array(start)
    distance = np.linalg.norm(direction)
    if distance < step_size:
        new_point = goal
    else:
        unit_direction = direction / distance
        repulsive_force = calculate_repulsive_force(np.array(start), rectangles, K, R)
        attractive_force = calculate_attractive_force(np.array(start), np.array(goal), C)
        # Normalize the repulsive force to ensure it's a unit vector
        epsilon = 1e-9
        repulsive_force_normalized = repulsive_force / np.linalg.norm(repulsive_force+epsilon)

        # Calculate the net force by subtracting the repulsive force from the attractive force
        net_force = attractive_force - repulsive_force_normalized
        net_force_normalized = net_force / np.linalg.norm(net_force + epsilon)

        # Calculate the new point by applying the net force
        new_point = np.array(start) + unit_direction * step_size + net_force_normalized * step_size

        # Clip the new point to ensure it stays within the workspace boundaries
        new_point = np.clip(new_point, [0, 0], [50, 30])  # Adjusted to workspace boundaries

    return tuple(new_point)


def collision_free(start, end, rectangles, circles):
    """
    Check if a direct path between two points is free of obstacles.

    Args:
        start (tuple): The starting point coordinates (x, y).
        end (tuple): The ending point coordinates (x, y).
        rectangles (list): A list of rectangle obstacles.
        circles (list): A list of circular obstacles, each defined as [x, y, radius].

    Returns:
        bool: True if the path is clear of obstacles, False otherwise.
    """

    line = LineString([start, end])
    for rect in rectangles:
        if line.intersects(box(rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])):
            return False
    for circle in circles:
        center = Point(circle[0], circle[1])  # Convert to a Point object
        if line.distance(center) < circle[2]:
            return False
    return True


class RRTConnect:
    """Implements the RRT-Connect algorithm with artificial potential fields."""
    def __init__(self, start, goal, rectangles, circles, step_size, K, R, max_iter,C):
        """
        Initialize the RRT-Connect algorithm with key parameters.

        Args:
            start (tuple): The starting point coordinates (x, y).
            goal (tuple): The goal point coordinates (x, y).
            rectangles (list): A list of rectangle obstacles, where each item is [x, y, width, height].
            circles (list): A list of circular obstacles, each defined as [x, y, radius].
            step_size (float): The maximum step size between points.
            K (float): The repulsive force scaling constant.
            R (float): The radius of influence of the repulsive force.
            max_iter (int): The maximum number of iterations to run the algorithm.
            C (float): The attractive force scaling constant.
        """

        self.start = Node(start)
        self.goal = Node(goal)
        self.rectangles = rectangles
        self.circles = circles
        self.step_size = step_size
        self.K = K
        self.R = R
        self.max_iter = max_iter
        self.trees = [Tree(), Tree()]  # Two trees for bi-directional search
        self.trees[0].add_vertex(self.start)
        self.trees[1].add_vertex(self.goal)
        self.C = C

    def plan_with_animation(self):
        """
        Plan the path using RRT-Connect with visualization/animation.

        Returns:
            tuple: The final path as a list of coordinates and its total length.
        """

        fig, ax = plt.subplots()
        self.plot_obstacles(ax)

        for _ in range(self.max_iter):
            for tree_id in range(2):  # Alternate between trees
                tree = self.trees[tree_id]
                other_tree = self.trees[1 - tree_id]
                random_point = self.random_point()
                nearest_node = self.nearest(tree, random_point)
                new_node = self.steer_node(nearest_node, random_point)

                if new_node and collision_free((nearest_node.x, nearest_node.y), (new_node.x, new_node.y), self.rectangles, self.circles):
                    tree.add_vertex(new_node)
                    tree.add_edge(nearest_node, new_node)
                    ax.plot([nearest_node.x, new_node.x], [nearest_node.y, new_node.y], color='red' if tree_id == 0 else 'blue')

                    # Try to connect to the other tree
                    nearest_other = self.nearest(other_tree, (new_node.x, new_node.y))
                    if nearest_other and collision_free((new_node.x, new_node.y), (nearest_other.x, nearest_other.y), self.rectangles, self.circles):
                        if np.linalg.norm(np.array((new_node.x, new_node.y)) - np.array((nearest_other.x, nearest_other.y))) <= 2:
                            other_tree.add_edge(nearest_other, new_node)
                            ax.plot([nearest_other.x, new_node.x], [nearest_other.y, new_node.y], color='green')
                            path,length = self.extract_path(nearest_other, new_node)
                            self.plot_path(ax, path)
                            ax.set_title("RRT-Connect with APF")
                            plt.show(block=True)  # Keep the plot open after finding the path
                            return path,length



        ax.set_title("No Path Found")
        plt.show(block=True)
        return None

    def random_point(self):
        """
        Sample a random point in the workspace.

        Returns:
            tuple: A tuple representing the random coordinates (x, y).
        """

        return (np.random.uniform(0, 50), np.random.uniform(0, 30))

    def nearest(self, tree, point):
        """
        Find the nearest node in the given tree to the specified point.

        Args:
            tree (Tree): The tree in which to search for the nearest node.
            point (tuple): A tuple representing the target point coordinates (x, y).

        Returns:
            Node: The nearest Node object in the tree.
        """

        return min(tree.vertices, key=lambda node: np.linalg.norm(np.array((node.x, node.y)) - np.array(point)))

    def steer_node(self, nearest_node, random_point):
        """
        Steer from the nearest node toward a random point, considering repulsive forces.

        Args:
            nearest_node (Node): The nearest node from which to steer.
            random_point (tuple): The random target point coordinates (x, y).

        Returns:
            Node: The newly created Node object at the steered position.
        """

        new_point = steer((nearest_node.x, nearest_node.y), random_point, self.step_size, self.rectangles, self.K, self.R,self.C)
        return Node(new_point, nearest_node)

    def check_collision(self,path, rectangles, circles):
        """
        Check if the given path is collision-free with the specified obstacles.

        Args:
            path (list): A list of coordinates representing the path points.
            rectangles (list): A list of rectangle obstacles.
            circles (list): A list of circular obstacles, each defined as [x, y, radius].

        Returns:
            bool: True if the path is collision-free, False otherwise.
        """

        for i in range(len(path) - 1):
            segment = LineString([path[i], path[i + 1]])
            for rect in rectangles:
                if segment.intersects(box(rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])):
                    return False
            for circle in circles:
                center = Point(circle[0], circle[1])
                if segment.distance(center) < circle[2]:
                    return False
        return True

    def optimize_path(self, path):
        """
        Optimize the path by converting it into a graph and finding the shortest path.

        Args:
            path (list): A list of tuples representing the initial path points.

        Returns:
            list: The optimized path using Dijkstra's algorithm.
        """

        graph = path_to_graph(path)
        start = path[0]
        goal = path[-1]
        optimized_path = dijkstra(graph, start, goal)
        return optimized_path
    def extract_path(self, start_connection, goal_connection):
        """
        Extract the path from start to goal by backtracking and smoothing the trajectory.

        Args:
            start_connection (Node): The node connecting to the starting tree.
            goal_connection (Node): The node connecting to the goal tree.

        Returns:
            tuple: A tuple containing the smoothed path as a list of coordinates and its total length.
        """

        path_from_start = self.trace_path(start_connection)
        path_from_goal = self.trace_path(goal_connection)
        combined_path = path_from_start[::-1] + path_from_goal

        optimized_path = self.optimize_path(combined_path)
        # Smooth th path using Cubic BSpline

        x, y = zip(*optimized_path)
        cs_x = CubicSpline(np.arange(len(x)), x)
        cs_y = CubicSpline(np.arange(len(y)), y)

        x_smooth = cs_x(np.linspace(0, len(x) - 1, 100))
        y_smooth = cs_y(np.linspace(0, len(y) - 1, 100))

        smooth_path = list(zip(x_smooth, y_smooth))

        #Calculate the total length of the smooth path
        total_length = sum(np.linalg.norm(np.array(smooth_path[i]) - np.array(smooth_path[i + 1])) for i in
                           range(len(smooth_path) - 1))

        return smooth_path, total_length
    def trace_path(self, node):
        """
        Trace the path back from a node to the root of its tree.

        Args:
            node (Node): The node from which to trace back.

        Returns:
            list: A list of coordinates representing the traced path.
        """

        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path

    def plot_obstacles(self, ax):
        """
        Plot rectangular and circular obstacles on the provided axis.

        Args:
            ax (matplotlib.axes.Axes): The axis object to draw the obstacles on.
        """

        for rect in self.rectangles:
            ax.add_patch(plt.Rectangle((rect[0], rect[1]), rect[2], rect[3], color='black', alpha=1))
        for circle in self.circles:
            ax.add_patch(plt.Circle((circle[0], circle[1]), circle[2], color='black', alpha=1))

        ax.plot(self.start.x, self.start.y, "bs", linewidth=3)
        ax.plot(self.goal.x, self.goal.y, "gs", linewidth=3)
        ax.set_xlim(0, 50)
        ax.set_ylim(0, 30)
        ax.set_xlabel("X Range")
        ax.set_ylabel("Y Range")
        ax.set_aspect('equal', adjustable='box')

    def plot_path(self, ax, path):
        """
        Plot the final smoothed path.

        Args:
            ax (matplotlib.axes.Axes): The axis object to draw the path on.
            path (list): A list of tuples representing the path coordinates.
        """

        if len(path) != 0:
            ax.plot([x[0] for x in path], [x[1] for x in path], '-g', linewidth=2)

def main():
    """
    Main function to run the RRT-Connect algorithm.

    Prompts the user for input parameters and executes the path planning.
    """

    C = float(input("Enter the attractive force constant (C): "))
    K = float(input("Enter the repulsive force constant (K): "))
    R = float(input("Enter the obstacle radius of influence (R): "))
    STARTING = tuple(map(float, input("Enter the starting point (x, y): ").split(',')))
    GOAL = tuple(map(float, input("Enter the goal point (x, y): ").split(',')))

    # Obstacles (rectangles and circles)
    rectangles = [
        [5,18,2,7],
        [7,23,7,2],
        [13,3,4,4],
        [19, 3, 4, 4],
        [25, 3, 4, 4],
        [31, 3, 4, 4],
        [31, 9, 4, 4],
        [31, 15, 4, 4],
        [31, 21, 4, 4],
        [16,10,2,15],
        [20,10,10,2],
        [28,12,2,5],
        [0, 0, 1, 30],
        [0, 0, 50, 1],
        [49, 0, 50, 30],
        [0, 29, 50, 30],
    ]
    circles = [
        [42, 17, 5],
        [25, 15, 2],
        [10, 20, 2]
    ]
    step_size = 1
    num_iterations = 1000

    rrt_connect = RRTConnect(STARTING, GOAL, rectangles, circles, step_size, K, R, num_iterations,C)
    start_time = time.time()
    path,length = rrt_connect.plan_with_animation()
    end_time = time.time()

    if path:
        print(f"Time:{end_time-start_time} \t Path length:{length}")

    else:
        print("No path found.")


if __name__ == "__main__":
    main()
