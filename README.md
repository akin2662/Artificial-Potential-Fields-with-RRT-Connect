# Implementation of the Improved RRT-Connect algorithm integrating Artificial Potential Field technique along with Path Optimization through B-spline and Dijkstra.
**Note:** This project was a requirement for the course ENPM 661- Planning for Autonomous Robots at University of Maryland, College Park and was done in collaboration with Raajith Gadam (raajithg@umd.edu). It is inspired from Y. Zhu, Y. Tang, Y. Zhang and Y. Huang, ”Path Planning of Manipulator Based on Improved RRT-Connect Algorithm,” 2021 2nd International Conference on Big Data & Artificial Intelligence & Software Engineering (ICBASE), Zhuhai, China, 2021, pp. 44-47, doi:10.1109/ICBASE53849.2021.00016.


## Project Description:

This script contains an implementation of the Rapidly-exploring Random Trees connect (RRT-connect) algorithm integrated with artificial potential field and path optimization through cubic spline for robotic path planning. The goal of the algorithm is to find an optimal path between a start and a goal position, while avoiding obstacles and optimizing the path length.

## Features:
* Improved RRT-connect algorithm implementation for efficient path planning.
* Artificial Potential Field integration for guided tree exploration and obstacle avoidance.
* Path optimization through Dijkstra to remove redundant points and shorten the path.
* B-spline for smoothing the path to avoid rapid changes in direction.

## Dependencies:

* python 3.11 (any version above 3 should work)
* Python running IDE (We used Pycharm)

## Libraries used:
* NumPy
* Time
* Shapely
* Matplotlib
* Scipy


## Instructions:
1. Download the zip file and extract it
	
2. Install python and the required dependencies: 

	`pip install numpy shapely matplotlib scipy`
	
3. Run the code or use desired python IDE:

	`$python3 proj5_raajith_advait_APF.py`
	
4. Input the Attractive Force Constant (C), Repulsive Force Constant (K), Obstacle influence Radius (R) start and goal point(x y).
5. The optimal path will be displayed on the screen and the number of samples taken along with the runtime will be printed in the terminal.

### Mock Output:
Enter the attractive force constant (C): 300
Enter the repulsive force constant (K): 50
Enter the obstacle radius of influence (R): 10
Enter the starting point (x, y): 2,2
Enter the goal point (x, y): 48,24
Time:7.199061870574951 	 Path length:73.73739875422808
![rrtconnect_apf](https://github.com/user-attachments/assets/6246cdef-0534-4c85-a6f4-dada1913f1dd)

### **Note**
Since the algorithm for the improved RRTConnect path is fast, it may happen that the you may not be able to see the visualization i.e tree exploration. But, the code is written in such a way that the tree exploration can be seen (similar code as one implemented for RRT Connect). 
