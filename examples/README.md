## Corridor example  
- **corridor.rs** — Demonstrates path planning through a narrow passage. The map is mostly blocked except for a central horizontal corridor. Useful for testing how the planner behaves when free space is tightly constrained and both start and goal must be placed inside the corridor to produce a valid path.

## Dynamic obstacle example  
- **dynamic.rs** — Shows how the planner reacts when the environment changes. It computes an initial path, modifies the map by adding a new obstacle, reinitializes the planner, and generates a new trajectory. This highlights D*’s ability to adapt to updated costmaps.

## Maze example  
- **maze.rs** — Uses a simple maze‑like grid with repeated horizontal barriers and openings. The planner must navigate around dead ends and find a valid route through the available gaps. This stresses neighbor exploration and backpointer correctness.

## Minimal example  
- **minimal.rs** — A minimal demonstration of the planner on an empty grid. It initializes a small map and computes a direct path from start to goal. Intended as the simplest reference for users who want to understand basic usage.

## Planner example  
- **planner.rs** — A general demonstration of the global planner interface. Shows how to initialize the planner, feed a costmap, and request a trajectory. Useful as a baseline for integrating the planner into other applications.

## U‑shaped obstacle example  
- **u_shape.rs** — Constructs a U‑shaped obstacle that forces the planner to route around a trap. This example highlights how the algorithm handles enclosed or partially enclosed structures and finds a valid detour.

## Virtual paths example  
- **virtual_paths.rs** — Demonstrates the use of virtual free‑corridor definitions loaded from JSON. The map is fully blocked except for a path carved by virtual path entries. Shows how external configuration can define navigable routes.

## Virtual walls JSON example  
- **virtual_walls_json.rs** — Loads virtual walls and paths from a JSON file and applies them to the costmap. Intended to show how obstacle and corridor definitions can be provided externally. Requires a valid `virtual_walls.json` file to produce a path.
