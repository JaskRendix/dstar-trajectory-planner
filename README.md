# D\* Dynamic Trajectory Planner (Rust Port)

A Rust re‑implementation of the original ROS C++ global trajectory planner based on the D\* incremental search algorithm.

The original version was developed as a `move_base` / Navigation2 plugin by Andrey Vukolov at Elettra Sincrotrone Trieste and presented at the 16th IFToMM World Congress.  
This project refactors the core algorithm into a standalone Rust library and application.

Upstream C++ repository:  
[https://github.com/ElettraSciComp/DStar-Trajectory-Planner](https://github.com/ElettraSciComp/DStar-Trajectory-Planner)

---

## Overview

This Rust port provides a clean and predictable implementation of the D\* global planner.  
All ROS2 dependencies have been removed, and the planner is exposed as a Rust crate with optional CLI examples.

The Rust version includes the full trajectory pipeline from the original C++ implementation:

- Draft path (backpointer chain)  
- Reduced path (ray‑tracing visibility pruning)  
- Optimized path (potential‑field smoothing using repulsion forces)

The planner supports:

- JSON virtual walls and virtual paths  
- bounds‑checked map access  
- flattened grid storage  
- binary‑heap open list  
- multiple examples and Criterion benchmarks  

These components match the behavior of the upstream algorithm.

---

## Key Differences from the C++ Version

### Standalone Rust Engine
Runs as a Rust library with optional CLI examples.  
No ROS2 build system or navigation stack dependencies.

### Flattened Grid
The nested vector structure is replaced with a contiguous `Vec<StatePoint>`.  
This improves cache locality and simplifies indexing.

### Binary‑Heap Open List
The open list uses `BinaryHeap<Reverse<(k, x, y)>>`, reducing insertion and extraction cost.

### Safe Map Access
All map access is bounds‑checked and returns structured errors for invalid states.

### Full Path Optimization Pipeline
Includes the complete post‑processing stages:

- Ray‑tracing reduction using `cutoff_distance`  
- Potential‑field smoothing using `repulsion_gain` and `potential_field_radius`

### JSON Virtual Walls
Virtual walls and virtual paths are loaded through `serde`.

### Neighbor Modes
The planner supports configurable neighbor connectivity:

- Four‑connected (Manhattan)  
- Eight‑connected (diagonal)

This affects path shape, obstacle avoidance, and cost propagation.

Neighbor mode can be selected through:

```rust
planner.set_neighbor_mode(NeighborMode::Four);
planner.set_neighbor_mode(NeighborMode::Eight);
```

The default mode is `Eight`.

A dedicated example is available in `examples/neighbor_modes.rs`.

---

## Parameters

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
| `neighbor_mode` | - | `Eight` | Connectivity mode (Four or Eight) |

---

## Examples

All examples are documented in:

`examples/README.md`

Run any example:

```bash
cargo run --example minimal
```

---

### **CLI Interface**

The project includes a configurable CLI tool (`src/main.rs`) that exposes all major planner parameters at runtime.  
This allows testing different optimization settings, neighbor modes, erosion, and map sizes without modifying code.

Run the CLI:

```bash
cargo run -- <options>
```

Show all available flags:

```bash
cargo run -- --help
```

### **Available CLI Parameters**

| Flag | Default | Description |
| --- | --- | --- |
| `--repulsion-gain <f64>` | `50.0` | Potential‑field repulsion gain |
| `--potential-radius <i32>` | `10` | Radius for potential‑field smoothing |
| `--cutoff-distance <i32>` | `16` | Ray‑tracing cutoff distance |
| `--occupancy-threshold <i32>` | `64` | Costmap obstacle threshold |
| `--neighbor-mode <four\|eight>` | `eight` | Connectivity mode (case-insensitive) |
| `--erosion <bool>` | `false` | Enable map erosion |
| `--erosion-gap <i64>` | `2` | Erosion radius |
| `--start-x <f64>` | `0.0` | Start coordinate X |
| `--start-y <f64>` | `0.0` | Start coordinate Y |
| `--goal-x <f64>` | `50.0` | Goal coordinate X |
| `--goal-y <f64>` | `50.0` | Goal coordinate Y |
| `--width <i64>` | `60` | Map width |
| `--height <i64>` | `60` | Map height |
| `--paths-file <path>` | `""` | JSON virtual paths file |
| `--enable-ready-paths <bool>` | `false` | Enable predefined JSON paths |
| `--verbose` | off | Enable verbose logging |

### **Example Usage**

Run with diagonal mode and custom smoothing:

```bash
cargo run -- \
    --neighbor-mode eight \
    --repulsion-gain 80 \
    --potential-radius 15 \
    --cutoff-distance 20 \
    --verbose
```

Run with Manhattan mode and erosion:

```bash
cargo run -- \
    --neighbor-mode four \
    --erosion true \
    --erosion-gap 3
```

Run with custom start/goal:

```bash
cargo run -- \
    --start-x 5 --start-y 5 \
    --goal-x 55 --goal-y 10
```

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

---

## Benchmarks

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

## Performance Notes

The binary‑heap open list reduces the cost of D\* state management.  
Flattened grid storage and safe neighbor traversal improve performance.

Example results on a mid‑range laptop:

| Scenario | Rust (Vec + sort) | Rust (BinaryHeap) |
| --- | --- | --- |
| 200×200 map | ~192 ms | ~12 ms |
| 500×500 map | ~2.95 s | ~85 ms |

Actual performance depends on hardware, map structure, and whether path optimization is enabled.
