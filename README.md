# SimplePlanner
<p align="left">
  <strong>From grid to path: A* planning with clearance-based costs</strong>
</p>

<p align="left">
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT">
  </a>
  <a href="https://www.ros.org/">
    <img src="https://img.shields.io/badge/ROS2-Jazzy-blue.svg" alt="ROS2 Jazzy">
  </a>
  <a href="https://isocpp.org/">
    <img src="https://img.shields.io/badge/C%2B%2B-17-green.svg" alt="C++17">
  </a>
</p>

<p align="center">
  <img src="/assests/demo.gif" width="600" alt="Path planning in RViz"/>
</p>


## Overview

**Simple Planner** is a ROS2 global planner, implemented in C++, that addresses the problem of path planning on grid-based maps.  
The system takes as input a 2D occupancy grid, a discrete representation of the environment where each cell is labeled as free, occupied, or unknown.  
From this map, it derives a costmap that penalizes cells located close to obstacles, and applies the A* search algorithm to compute an optimal path between the start and the goal.  
The resulting trajectories are not only feasible but also preserve safety margins from walls and obstacles.


## Planning Pipeline

The planning process in **Simple Planner** can be described as a sequence of transformations that convert a raw occupancy map into a collision-free and clearance-aware path.


### 1. From Occupancy Grid to MapData
The initial map is provided as a `nav_msgs/OccupancyGrid`, a 2D grid where each cell can take one of the following values:
- `0` → free  
- `100` → occupied  
- `-1` → unknown  

This representation is converted into an internal structure (`MapData`) that stores metadata (dimensions, resolution, origin) and a binary 2D grid distinguishing free space from obstacles.


### 2. From Obstacles to Distance Map
Starting from occupied cells, a *distance map* is computed using a **multi-source Breadth-First Search (BFS)**.  
For each free cell $i$, the algorithm assigns the Euclidean distance to the nearest obstacle:

$$
d(i) = \min_{j \in \mathcal{O}} \; \| p_i - p_j \|_2
$$

where $\mathcal{O}$ is the set of obstacle cells and $p_i$ is the metric position of cell $i$.  
This provides a *clearance measure*: the larger $d(i)$, the safer the cell.


### 3. From Distance to Cost
The distance map is transformed into a *costmap* by assigning traversal penalties according to the chosen cost function:

- **Exponential decay**  

$$
c(i) =
\begin{cases}
W \, e^{-\alpha \, d(i)}, & d(i) < R \\
0, & d(i) \geq R
\end{cases}
$$

- **Linear decay**  

$$
c(i) =
\begin{cases}
W \left( 1 - \frac{d(i)}{R} \right), & d(i) < R \\
0, & d(i) \geq R
\end{cases}
$$

- **Inverse**  
  
$$
c(i) = \frac{W}{d(i) + \varepsilon}
$$
  

where $R$ is the *inflation radius*, $\alpha$ a decay coefficient, $W$ a global weight, and $\varepsilon$ a small constant to avoid division by zero.  
As a result, cells near obstacles are assigned higher traversal costs.


### 4. From Costs to Path (A*)
The A* search algorithm is applied on the costmap to compute the optimal path.  
The cumulative cost of moving from a cell $i$ to a neighbor $j$ is:

$$
g(j) = g(i) + \Delta d(i,j) + \lambda \, c(j)
$$

where:  
- $\Delta d(i,j)$ is the step cost (resolution for orthogonal moves, $\sqrt{2}$ times resolution for diagonal moves),  
- $c(j)$ is the cost of the target cell,  
- $\lambda$ is a weight balancing the influence of the costmap.  

The heuristic is defined as the Euclidean distance from cell $j$ to the goal:

$$
h(j) = \sqrt{(x_j - x_g)^2 + (y_j - y_g)^2}
$$

The A* priority function is therefore:

$$
f(j) = g(j) + h(j)
$$

Diagonal moves are allowed only if the adjacent orthogonal cells are free, preventing *corner-cutting* through obstacles.


### 5. From Grid to World
The resulting path is a sequence of cells \((r,c)\).  
These are converted into metric world coordinates as follows:

$$
x = x_{\text{origin}} + (c + 0.5) \cdot \text{resolution}, \quad
y = y_{\text{origin}} + (r + 0.5) \cdot \text{resolution}
$$

The path is published as a `nav_msgs/Path` message in the `map` frame and visualized in RViz together with markers for start and goal.


## ROS Interfaces

The planner communicates through standard ROS2 topics and TF2 transforms.

<table>
<tr>
<td style="padding-right:40px; vertical-align:top">

<b>Subscribers</b>  
<div style="font-size:90%">

| Topic          | Msg type                  | Purpose              |
|----------------|---------------------------|----------------------|
| `/map`         | `OccupancyGrid`           | 2D map input         |
| `/goal_pose`   | `PoseStamped`             | Goal position        |
| `/initialpose` | `PoseWithCovarianceStamped` | Manual start pose  |

</div>

</td>
<td style="padding-left:40px; vertical-align:top">

<b>Publishers</b>  
<div style="font-size:90%">

| Topic                  | Msg type        | Purpose                          |
|-------------------------|-----------------|----------------------------------|
| `/planned_path`         | `Path`          | Planned path (sequence of poses) |
| `/visualization_marker` | `Marker`        | Start/goal markers in RViz       |
| `/simple_planner/costmap_debug` | `Image` | Costmap debug visualization      |

</div>

</td>
</tr>
</table>

### TF2
- If `use_manual_start = false`, the current robot pose is obtained from TF2, using the transform:  `map -> base_link`
- If `use_manual_start = true`, the robot pose is set manually via `/initialpose`.

## Parameters

<div style="font-size:90%">

| Parameter            | Type    | Default      | Description |
|----------------------|---------|--------------|-------------|
| `use_manual_start`   | bool    | `true`      | If true, robot start pose is set manually from `/initialpose` instead of TF2 |
| `cost_function_type` | string  | `"exponential"` | Type of cost function: `exponential`, `linear`, or `inverse` |
| `inflation_radius`   | float   | `0.5`        | Radius (m) around obstacles where costs are inflated |
| `alpha`              | float   | `5.0`        | Decay coefficient for exponential cost function |
| `W`                  | float   | `20.0`       | Global weight applied to the costmap function |
| `epsilon`            | float   | `0.1`        | Small constant to avoid division by zero in inverse cost |
| `lethal_cost`        | float   | `1e9`        | Cost value assigned to occupied/unknown cells (treated as lethal) |
| `lambda_weight`      | float   | `2.0`        | Weight for costmap term in A* path cost (g + λ·cost + h) |
</div>


## Project Structure
```
SimplePlanner/
├── config/ # Parameters and YAML configuration
├── include/simple_planner/ # C++ headers
├── maps/ # Example maps
├── rviz/ # RViz configuration
├── src/ # C++ source files
├── run_simple_planner.sh # Helper launch script
├── CMakeLists.txt # Build configuration
├── package.xml # ROS2 package metadata
└── README.md # Project documentation
```

## Running the Planner
Clone the repository inside a ROS2 workspace (e.g. `~/Documents/GITHUB_PROJECTS`):

```bash
cd ~/Documents/GITHUB_PROJECTS
git clone <your-repo-url> SimplePlanner
cd SimplePlanner
```

Build with colcon:
```bash
colcon build
source install/setup.bash
```

Run the script from the root of the workspace:
```bash
chmod +x run simple_planner.sh 
./run_simple_planner.sh
```


## Examples 
The following experiments were obtained with common parameters  
(`inflation_radius = 0.5 m`, `alpha = 5.0`, `W = 20.0`, `lambda_weight = 2.0`).  
The only difference lies in the cost function type.
<table>
<tr>
<td align="center">
  <img src="/assests/exponential.png" width="400"><br>
  <sub>(a) Exponential cost function</sub>
</td>
<td align="center">
  <img src="/assests/zoom_inverse.png" width="400"><br>
  <sub>(b) Linear cost function</sub>
</td>
</tr>
</table>

