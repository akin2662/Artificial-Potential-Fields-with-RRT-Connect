# Improved RRT-Connect with APF — 2D Planner and Nav2 Global Planner Plugin

Inspired by Y. Zhu, Y. Tang, Y. Zhang and Y. Huang, "Path Planning of Manipulator Based on Improved RRT-Connect Algorithm," ICBASE 2021, pp. 44-47, doi:10.1109/ICBASE53849.2021.00016.


## What is this?

This started as a course project implementing RRT-Connect with Artificial Potential Fields for 2D path planning. I extended it into a fully functional ROS2 Nav2 global planner plugin that runs on a Turtlebot3 in Gazebo simulation.

The repository has two parts. The first is a standalone Python planner you can run directly to visualize paths in 2D. The second is a Nav2 plugin that plugs the same algorithm into ROS2's navigation stack so a real (or simulated) robot can use it.


## Nav2 Plugin Demo

[Plugin Demo](https://drive.google.com/file/d/1_0Tn8NCuhYe3tkSEEy9UtCTKNoestbSg/view?usp=sharing)

The robot plans a path using RRT-Connect with APF, follows it, and automatically replans if an obstacle appears in the way.

### How it works

The tricky part about making a Python planner work with Nav2 is that Nav2's plugin system (pluginlib) only loads C++ shared libraries. So I wrote a thin C++ wrapper that implements the Nav2 GlobalPlanner interface and calls my Python planning code through a ROS2 service. The planning logic stays in Python, untouched.

```
Nav2 planner server
      |
      | pluginlib loads C++ shared library
      v
apf_rrtc_plugin.cpp  (C++ Nav2 plugin wrapper)
      |
      | ROS2 service call to /compute_path
      v
planner_service_node.py  (Python ROS2 service)
      |
      | calls planning algorithm
      v
rrt_connect_core.py  (pure Python, no ROS2)
      |
      v
nav_msgs/Path returned to Nav2 controller
```

### Results

| Metric | Value |
|--------|-------|
| Mean planning time | ~35ms |
| Success rate | 100% across 20 random trials |
| Replanning | Automatic when path is blocked |
| ROS2 version | Humble |
| Robot | Turtlebot3 Burger in Gazebo |

### Getting started

Install the dependencies:

```bash
sudo apt install ros-humble-nav2-core \
                 ros-humble-nav2-costmap-2d \
                 ros-humble-nav2-util \
                 ros-humble-pluginlib \
                 ros-humble-nav2-bringup \
                 ros-humble-turtlebot3-gazebo
pip install numpy scipy shapely
```

Build the package:

```bash
mkdir workspace_name
cd workspace_name && mkdir src
cd src
git clone https://github.com/YOUR_USERNAME/apf_rrtc_plugin
cd ~/workspace_name
source /opt/ros/humble/setup.bash
colcon build 
source install/setup.bash
```

Launch everything:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch apf_rrtc_plugin navigation.launch.py
```

Once RViz opens, set the 2D Pose Estimate on the robot's location, then send a Nav2 Goal anywhere on the free space. The planner runs, the path appears, and the robot follows it.

### Tunable parameters

These are set in `params/nav2_params.yaml` under `apf_rrtc_planner_node`:

| Parameter | Default | What it does |
|-----------|---------|-------------|
| `step_size` | 0.3 | How far the tree grows per step (metres) |
| `max_iterations` | 2000 | Planning budget before giving up |
| `goal_sample_rate` | 0.1 | How often to sample toward goal vs random |
| `apf_C` | 10.0 | How strongly the planner is attracted to goal |
| `apf_K` | 2.0 | How strongly it pushes away from obstacles |
| `apf_R` | 0.35 | Obstacle influence radius in metres |
| `connect_tolerance` | 0.05 | How close trees need to be to connect |

### Repository structure
```
apf_rrtc_plugin/
├── apf_rrtc_plugin/
│   ├── rrt_connect_core.py       # planning algorithm, no ROS2
│   ├── planner_service_node.py   # ROS2 service server
│   └── rrt_connect_planner.py    # reference Nav2 plugin attempt
├── src/
│   └── apf_rrtc_plugin.cpp       # C++ Nav2 plugin wrapper
├── include/apf_rrtc_plugin/
│   └── apf_rrtc_plugin.hpp
├── srv/
│   └── ComputePath.srv
├── params/
│   └── nav2_params.yaml
├── launch/
│   └── navigation.launch.py
├── behavior_trees/
│   └── navigate_once.xml
├── CMakeLists.txt
└── package.xml
```


## Standalone 2D Planner

This is the original version that runs without ROS2. You give it a start point, a goal point, and some tuning parameters, and it shows you the tree exploration and final path in matplotlib.

The algorithm combines bidirectional RRT-Connect (two trees growing simultaneously from start and goal) with Artificial Potential Fields that bias expansion away from obstacles and toward the goal. Once a path is found, Dijkstra removes unnecessary waypoints and a cubic B-spline smooths the result.

### Dependencies

```bash
pip install numpy shapely matplotlib scipy
```

### Running it

```bash
python3 proj5_raajith_advait_APF.py
```

You'll be prompted for the APF constants and start/goal coordinates:

```
Enter the attractive force constant (C): 300
Enter the repulsive force constant (K): 50
Enter the obstacle radius of influence (R): 10
Enter the starting point (x, y): 2,2
Enter the goal point (x, y): 48,24
Time: 7.199s    Path length: 73.737m
```

![rrtconnect_apf](https://github.com/user-attachments/assets/6246cdef-0534-4c85-a6f4-dada1913f1dd)

Note: the tree exploration can look like it happens instantly because the planner is fast. The visualization code does render it step by step but you may need to slow it down if you want to watch the tree grow.

---

## Version history

**Version 2.0** — Nav2 plugin

I extended the standalone planner into a full ROS2 Nav2 global planner plugin. The main engineering challenge was that Nav2's pluginlib only supports C++ plugins, so I wrote a C++ wrapper that delegates all planning to the Python code via a ROS2 service. The core algorithm was refactored to work with live Nav2 costmaps instead of hardcoded obstacle lists, and replanning was added so the robot responds to new obstacles automatically.

**Version 1.1** — Bug fixes

Going through the original code carefully, I found five bugs that were silently causing incorrect behavior:

Bug 1: `dijkstra()` was returning `list(distances.keys())` which is just the set of all visited nodes in insertion order, not an actual path. Fixed by adding a `previous` dictionary and reconstructing the path by backtracking from goal to start.

Bug 2: Force normalization in `steer()` was adding a scalar `epsilon` to a numpy array before dividing, which biased directions arbitrarily when forces were near zero. Fixed by checking magnitude explicitly before dividing and using a geometric fallback when forces cancel out.

Bug 3: Workspace bounds in `steer()` were hardcoded as `[0,50]x[0,30]`. These are now passed as parameters so the planner works on any map size.

Bug 4: `end_time` was recorded after the matplotlib animation finished, not after planning. The timing printed to terminal was including however long you looked at the plot. Fixed by moving the timestamp to immediately after `planning()` returns.

Bug 5: When planning failed (iteration limit reached), the code returned `None` and the next line tried to unpack it, causing a crash. Fixed by returning `(None, None)` and guarding against it before plotting.

**Version 1.0** — Initial course project implementation


## Citation

```bibtex
@inproceedings{zhu2021path,
  title={Path Planning of Manipulator Based on Improved RRT-Connect Algorithm},
  author={Zhu, Y. and Tang, Y. and Zhang, Y. and Huang, Y.},
  booktitle={2021 2nd International Conference on Big Data and Artificial 
             Intelligence and Software Engineering (ICBASE)},
  pages={44--47},
  year={2021},
  doi={10.1109/ICBASE53849.2021.00016}
}
```
