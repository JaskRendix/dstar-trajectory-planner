# D\* Dynamic Trajectory Planner (Rust Port)

A Rust re‑implementation of the original ROS C++ global trajectory planner based on the D\* incremental search algorithm.

The original version was developed as a `move_base` / Navigation2 plugin by Andrey Vukolov at Elettra Sincrotrone Trieste and presented at the 16th IFToMM World Congress.  
This project refactors the core algorithm into a standalone Rust library and application.

Upstream C++ repository:  
[https://github.com/ElettraSciComp/DStar-Trajectory-Planner](https://github.com/ElettraSciComp/DStar-Trajectory-Planner)

---

## Overview

The Rust port focuses on a clear and predictable implementation of D\*.  
The codebase removes ROS2 dependencies and exposes the planner as a simple Rust crate.  
The planner supports JSON configuration for virtual walls and passable or non‑passable zones.

The Rust version uses a flattened grid, safe neighbor traversal, and a binary‑heap open list.  
These changes reduce memory overhead and improve runtime behavior on large maps.

---

## Key Differences from the C++ Version

### Standalone Rust Engine
The planner no longer depends on ROS2 build systems or navigation interfaces.  
The algorithm runs as a pure Rust library with a small CLI wrapper.

### Flattened Grid
The original nested vector structure is replaced with a single contiguous `Vec<StatePoint>`.  
This improves cache locality and simplifies indexing.

### Binary‑Heap Open List
The open list uses `BinaryHeap<Reverse<(k, x, y)>>`.  
This replaces repeated sorting and reduces the cost of state insertion and extraction.

### Safe Map Access
All map access is bounds‑checked.  
The planner avoids panics and returns structured errors when states are invalid.

### JSON Virtual Walls
The planner loads virtual walls and configuration files through `serde`.

---

## Parameters

The planner uses the following configuration parameters:

| Parameter | Unit | Default | Description |
| --- | --- | --- | --- |
| `goal_distance_threshold` | m | `0.3` | Distance from origin to consider the goal reached |
| `neighbor_distance_threshold` | m | `0.1` | Distance threshold for replanning |
| `occupancy_threshold` | - | `64` | Costmap cell weight threshold for obstacles |
| `cutoff_distance` | cells | `16` | Raytracing cutoff distance |
| `trajectory_optimizer/repulsion_gain` | - | `50.0` | Repulsive potential gain |
| `trajectory_optimizer/potential_field_radius` | cells | `10` | Radius for potential field |
| `erosion/enable` | - | `false` | Enable map erosion |
| `erosion/erosion_gap` | cells | `2` | Erosion gap |

---

## Performance Notes

The binary‑heap open list reduces the cost of D\* state management.  
On medium and large maps, the Rust port shows significant reductions in planning time.

Example results on a mid‑range laptop:

| Scenario | Rust (Vec + sort) | Rust (BinaryHeap) |
| --- | --- | --- |
| 200×200 map | ~192 ms | ~12 ms |
| 500×500 map | ~2.95 s | ~85 ms |

These results depend on hardware and map structure.

---

## Usage

Run the planner:

```bash
cargo run
```

Show CLI options:

```bash
cargo run -- --help
```

Run benchmarks:

```bash
cargo bench
```
