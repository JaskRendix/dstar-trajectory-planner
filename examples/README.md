## Corridor example  
- **corridor.rs** — Demonstrates path planning through a narrow horizontal passage. The map is blocked except for a central corridor. Useful for testing how the planner behaves when free space is tightly constrained.

## Dynamic obstacle example  
- **dynamic.rs** — Shows how the planner reacts when the environment changes. It computes an initial path, injects a new obstacle, and replans. Highlights D*’s ability to adapt to updated costmaps.

## Maze example  
- **maze.rs** — Uses a simple maze‑like grid with repeated barriers and openings. The planner must navigate around dead ends and find a valid route. Stresses neighbor exploration and backpointer correctness.

## Minimal example  
- **minimal.rs** — A minimal demonstration on an empty grid. Initializes a small map and computes a direct path from start to goal. Intended as the simplest reference for new users.

## Planner example  
- **planner.rs** — A general demonstration of the global planner interface. Shows how to initialize the planner, feed a costmap, and request a trajectory. Useful for integration into external applications.

## U‑shaped obstacle example  
- **u_shape.rs** — Constructs a U‑shaped obstacle that forces the planner to route around a trap. Highlights how the algorithm handles enclosed or partially enclosed structures.

## Virtual paths example  
- **virtual_paths.rs** — Demonstrates virtual free‑corridor definitions loaded from JSON. The map is fully blocked except for a path carved by virtual path entries.

## Virtual walls JSON example  
- **virtual_walls_json.rs** — Loads virtual walls and paths from a JSON file and applies them to the costmap. Shows how obstacle and corridor definitions can be provided externally.

## Weighted costmap example  
- **weighted_costmap.rs** — Adds a horizontal band of higher cost to the map. Demonstrates how D* avoids weighted regions and chooses a longer but cheaper path. Useful for testing cost‑based detours.

## Potential field example  
- **potential_field.rs** — Benchmarks the potential‑field optimizer. Shows the optimized path and prints the potential values assigned along the path. Useful for verifying the optimizer port.

## Neighbor modes example  
- **neighbor_modes.rs** — Compares diagonal neighbor mode with a simulated Manhattan mode. Demonstrates how neighbor connectivity affects path length and behavior.

## Large‑map stress test  
- **large_map.rs** — Runs the planner on a 1000×1000 grid. Useful for benchmarking performance, memory behavior, and optimizer efficiency on large maps.
