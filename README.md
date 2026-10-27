# Comparison between Dijkstra and A* algorithms for motion planning (Technologies for Autonomous Vehicles)

A comprehensive path planning and network graph analysis project implementing, benchmarking, and visualizing **Dijkstra** and **A\*** (A-Star) search algorithms on real-world urban road topologies extracted from OpenStreetMap using OSMnx. Developed for the **Technologies for Autonomous Vehicles** course at Politecnico di Torino.

---

## Table of contents
1. [Overview](#overview)
2. [Requirements](#requirements)
3. [Architecture](#architecture)
   - [Road Network Extraction & Modeling](#1-road-network-extraction--modeling)
   - [Dijkstra's Algorithm Implementation](#2-dijkstras-algorithm-implementation)
   - [A* Heuristic Search Implementation](#3-a-heuristic-search-implementation)
   - [Cost Function & Admissible Heuristic Formulation](#4-cost-function--admissible-heuristic-formulation)
   - [Edge Usage & Heatmap Reconstruction](#5-edge-usage--heatmap-reconstruction)
4. [Specifications](#specifications)
5. [Project structure](#project-structure)
6. [Usage](#usage)
   - [Running the Algorithms](#running-the-algorithms)
   - [Expected Output & Visualization](#expected-output--visualization)

---

## Overview

Global path planning is a primary tier in the autonomy software stack for automated vehicles, computing time- or distance-optimal trajectories from an origin to a target location over a static global map before local reactive planners manage dynamic obstacle avoidance.

This project implements and evaluates two foundational graph-search algorithms applied to the driveable road network of **Turin, Italy**:
* **Dijkstra's Algorithm:** Uniform-cost search exploring vertices radially by current cumulative travel time.
* **A\* Search Algorithm:** Informed heuristic search directing expansion toward the target node using an admissible lower-bound Euclidean travel-time estimate.
* **Empirical Benchmarking:** Direct side-by-side comparison of computational complexity (expanded node iterations, search space area, and path reconstruction traversal).

---

## Requirements

* **Python:** 3.8 to 3.11 recommended.
* **Core Libraries:**
  * `osmnx` ($\ge 1.3.0$)
  * `matplotlib` ($\ge 3.5.0$)
  * Standard Python packages: `heapq`, `math`, `random`
* **Dependencies:** Geographic projection libraries (`geopandas`, `shapely`, `pyproj`, `networkx`) bundled with OSMnx.

To install the required dependencies:
```bash
pip install osmnx matplotlib
```

---

## Architecture

```plaintext
[ OpenStreetMap ("Turin, Italy") ] ──> [ osmnx.graph_from_place(network_type="drive") ]
                                                    │
                                                    ▼
                                       [ Projected Graph (UTM) ]
                                                    │
                                 ┌──────────────────┴──────────────────┐
                                 ▼                                     ▼
                      [ Attribute Parsing ]                  [ Node Random Sample ]
                      - Clean "maxspeed"                     - Origin ID ($v_{\text{orig}}$)
                      - Compute $w = L / v$                  - Target ID ($v_{\text{dest}}$)
                                 │                                     │
                                 └──────────────────┬──────────────────┘
                                                    │
                           ┌────────────────────────┴────────────────────────┐
                           ▼                                                 ▼
             [ Dijkstra's Search ]                                 [ A* Informed Search ]
             - Priority: $g(n)$                                    - Priority: $f(n) = g(n) + h(n)$
             - Min-Heap ($O((V+E)\log V)$)                         - Admissible Heuristic: $h(n) = d_{\text{euc}} / v_{\text{max}}$
                           │                                                 │
                           └────────────────────────┬────────────────────────┘
                                                    │
                                                    ▼
                                    [ Path & Heatmap Reconstruction ]
                                    - Traversal: destination -> origin
                                    - Visited Nodes & Path Overlay Plots
```

### 1. Road Network Extraction & Modeling
* Road network geometry and metadata are queried from OpenStreetMap via OSMnx for `Turin, Piedmont, Italy` using the `drive` network filter.
* The graph is projected to a local metric coordinate system (Universal Transverse Mercator, UTM) via `ox.project_graph(G)`, converting angular coordinates (degrees) into spatial Cartesian coordinates in meters.
* **Speed & Travel Time Imputation:**
  * Raw `maxspeed` attributes (which can appear as strings, lists, or missing values) are parsed, standardized, converted from $\text{km/h}$ to $\text{m/s}$, and defaulted to $50\text{ km/h}$ ($13.89\text{ m/s}$) when unspecified.
  * Each directed edge $e = (u, v)$ is assigned a dynamic traversal cost:
    $$\text{weight}(u, v) = \frac{\text{length}(u, v)}{v_{\text{max}}(u, v)} \quad [\text{seconds}]$$

### 2. Dijkstra's Algorithm Implementation
* Maintains a min-priority queue (binary min-heap via Python's `heapq`) sorted by the shortest path estimate $g(n)$.
* Explores the graph radially without directional bias:
  1. Initializes $g(s) = 0$ for start node $s$ and $g(v) = \infty$ for all $v \neq s$.
  2. In each iteration, extracts the vertex $u$ with the minimum $g(u)$.
  3. Relaxes all outgoing edges $(u, v)$:
     $$g(v) > g(u) + \text{weight}(u, v) \implies g(v) \leftarrow g(u) + \text{weight}(u, v)$$
  4. Records predecessor pointer `previous[v] = u` for backtracking.

### 3. A* Heuristic Search Implementation
* Uses an informed total cost function $f(n) = g(n) + h(n)$, where:
  * $g(n)$ is the exact cumulative travel time from origin to node $n$.
  * $h(n)$ is the heuristic estimate of travel time from node $n$ to destination $d$.
* Significantly prunes the search frontier by prioritizing nodes positioned along the spatial corridor toward the target.

### 4. Cost Function & Admissible Heuristic Formulation
To preserve the **admissibility** ($h(n) \le h^*(n)$) and **consistency** (monotonicity) of the A* search:
* The spatial distance between node coordinates $(x_1, y_1)$ and destination $(x_2, y_2)$ is computed via Euclidean metric:
  $$d_{\text{linear}} = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2} \quad [\text{m}]$$
* The heuristic travel time estimate assumes travel at a conservative upper speed bound:
  $$h(n) = \frac{d_{\text{linear}}}{v_{\text{heuristic}}} \quad [\text{s}], \quad \text{with } v_{\text{heuristic}} = \frac{50}{3.6} \approx 13.89\text{ m/s}$$
* Since the straight-line distance is always less than or equal to actual road network distance, and the divisor reflects city-speed bounds, the heuristic satisfies:
  $$h(n) \le h^*(n) \quad \forall n \in \mathcal{V}$$
  guaranteeing that A* returns an optimal solution identical to Dijkstra's algorithm while expanding fewer nodes.

### 5. Edge Usage & Heatmap Reconstruction
* Path reconstruction tracks `previous[curr]` starting from destination back to origin, computing cumulative route length and travel duration.
* Edge styling differentiates:
  * **Unvisited edges:** Dark orange (`#d36206`, thin)
  * **Explored edges:** Green (`green`, medium width)
  * **Frontier / Active edges:** Red (`red`, active wave)
  * **Optimal route edges:** Thick white (`white`, linewidth = 5)
* Side-by-side subplot visualization directly displays search space reduction between both methods.

---

## Specifications

| Parameter | Value | Unit | Description |
| :--- | :---: | :---: | :--- |
| Geographic Target | `Turin, Piedmont, Italy` | $-$ | OpenStreetMap urban bounding boundary |
| Road Network Type | `drive` | $-$ | Vehicle-accessible roadways |
| Default Urban Speed Limit | $50.0$ | $\text{km/h}$ | Imputed default speed when tag is absent |
| Heuristic Travel Speed | $50 / 3.6 \approx 13.89$ | $\text{m/s}$ | Upper-bound velocity for admissible $h(n)$ |
| Graph Projection | UTM | $-$ | Projected metric coordinate reference system |
| Priority Queue Structure | Binary Min-Heap (`heapq`) | $-$ | $O(\log V)$ node extraction and relaxation |
| Visualization Background | `#18080e` | $-$ | High-contrast dark graph canvas |

---

## Project structure

```plaintext
.
├── SCRIPT-Dijkstra_A_star.py     # Main Python script for map loading, search execution, and plotting
├── REPORT-Dijkstra_A_star.pdf    # Technical report with analytical comparison and iteration metrics
└── README.md                     # Project documentation
```

---

## Usage

### Running the Algorithms

1. Ensure Python and all dependencies (`osmnx`, `matplotlib`) are installed.
2. In your terminal, navigate to the project directory and execute:
   ```bash
   python SCRIPT-Dijkstra_A_star.py
   ```
3. The script will:
   * Download and project the road graph of Turin (cached on subsequent runs).
   * Sample random or configured origin and destination nodes.
   * Execute Dijkstra's search and display total expansion iterations.
   * Execute A* search and print the comparative expansion iterations.
   * Render side-by-side search footprint heatmaps and optimal routes.

### Expected Output & Visualization

* **Terminal Output:**
  ```plaintext
  Origin ID: <Node_ID_Origin>
  Destination ID: <Node_ID_Destination>
  Running Dijkstra
  Iterations: <N_Dijkstra>
  Running A*
  Iterations: <N_A_Star>
  ```
* **Graphical Output:** A Matplotlib window displaying two dark-canvas network maps:
  * **Left Plot (Dijkstra):** Shows a broad, circular/radial expansion pattern of explored nodes.
  * **Right Plot (A\*):** Demonstrates an ellipsoidal, goal-directed search corridor, verifying significant exploration reduction while reaching the same optimal path.
