# **D\* Dynamic Trajectory Planner (Rust Port)**

A Rust re‑implementation of the original ROS C++ global trajectory planner based on the D\* incremental search algorithm.

The original version was developed as a `move_base` / Navigation2 plugin by Andrey Vukolov at Elettra Sincrotrone Trieste and presented at the 16th IFToMM World Congress.  
This project refactors the core algorithm into a standalone Rust library and application.

Upstream C++ repository:  
[https://github.com/ElettraSciComp/DStar-Trajectory-Planner](https://github.com/ElettraSciComp/DStar-Trajectory-Planner)

---

## **Overview**

This Rust port provides a clean, predictable, and self‑contained implementation of the D\* global planner.  
All ROS2 dependencies have been removed, and the planner is exposed as a simple Rust crate with optional CLI examples.

The Rust version includes the **full trajectory pipeline** from the original C++ implementation:

- **Draft path** (backpointer chain)  
- **Reduced path** (ray‑tracing visibility pruning)  
- **Optimized path** (potential‑field smoothing using repulsion forces)

The planner also supports:

- JSON virtual walls and virtual paths  
- safe, bounds‑checked map access  
- flattened grid storage for cache‑efficient traversal  
- binary‑heap open list  
- multiple examples and Criterion benchmarks  

These additions produce smoother, more natural trajectories and match the behavior of the upstream algorithm.

---

## **Key Differences from the C++ Version**

### **Standalone Rust Engine**
Runs as a pure Rust library with optional CLI examples.  
No ROS2 build system, no navigation stack dependencies.

### **Flattened Grid**
The nested vector structure is replaced with a single contiguous `Vec<StatePoint>`.  
This improves cache locality and simplifies indexing.

### **Binary‑Heap Open List**
The open list uses `BinaryHeap<Reverse<(k, x, y)>>`, reducing insertion/extraction overhead compared to repeated sorting.

### **Safe Map Access**
All map access is bounds‑checked, preventing panics and returning structured errors for invalid states.

### **Full Path Optimization Pipeline**
The Rust port includes the complete post‑processing stages:

- **Ray‑tracing reduction** using `cutoff_distance`  
- **Potential‑field smoothing** using `repulsion_gain` and `potential_field_radius`  

These stages were part of the original C++ planner but are often omitted in ports.

### **JSON Virtual Walls**
Virtual walls and virtual paths are loaded through `serde`, allowing external configuration of passable and non‑passable zones.

---

## **Parameters**

| Parameter | Unit | Default | Description |
| --- | --- | --- | --- |
| `goal_distance_threshold` | m | `0.3` | Distance from origin to consider the goal reached |
| `neighbor_distance_threshold` | m | `0.1` | Distance threshold for replanning |
| `occupancy_threshold` | - | `64` | Costmap cell weight threshold for obstacles |
| `cutoff_distance` | cells | `16` | Ray‑tracing cutoff distance for path reduction |
| `trajectory_optimizer/repulsion_gain` | - | `50.0` | Repulsive potential gain |
| `trajectory_optimizer/potential_field_radius` | cells | `10` | Radius for potential‑field smoothing |
| `erosion/enable` | - | `false` | Enable map erosion |
| `erosion/erosion_gap` | cells | `2` | Erosion gap |

---

## **Examples**

All examples are documented in:

**`examples/README.md`**

This includes:

- basic usage  
- dynamic obstacles  
- maze navigation  
- virtual walls  
- potential‑field optimization  
- weighted costmaps  
- neighbor modes  
- large‑map stress tests  

Run any example:

```bash
cargo run --example minimal
```

---

## **Benchmarks**

Criterion benchmarks are provided under `benches/`:

- core D\* performance  
- costmap initialization  
- path planning  
- potential‑field optimization  
- large‑map stress tests  
- neighbor‑mode comparison  
- weighted costmap detours  

Run all benchmarks:

```bash
cargo bench
```

---

## **Performance Notes**

The binary‑heap open list significantly reduces the cost of D\* state management.  
Flattened grid storage and safe neighbor traversal further improve performance.

Example results on a mid‑range laptop:

| Scenario | Rust (Vec + sort) | Rust (BinaryHeap) |
| --- | --- | --- |
| 200×200 map | ~192 ms | ~12 ms |
| 500×500 map | ~2.95 s | ~85 ms |

Actual performance depends on hardware, map structure, and whether path optimization is enabled.

---

## **Usage**

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
