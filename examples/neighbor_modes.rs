use dstar_trajectory_planner::{DStarGlobalPlanner, NeighborMode};

fn main() {
    let width = 40;
    let height = 40;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (2.0, 2.0);
    let goal = (37.0, 37.0);

    // Default: Eight-connected neighbors
    println!("Diagonal neighbor mode:");
    planner.set_neighbor_mode(NeighborMode::Eight);
    let path_diag = planner.make_plan(start, goal).unwrap();
    println!("Path length: {}", path_diag.len());

    // Manhattan mode
    println!("Manhattan neighbor mode:");
    let mut planner2 = DStarGlobalPlanner::new();
    planner2.initialize(width, height, &map_data, false, "");
    planner2.set_neighbor_mode(NeighborMode::Four);
    let path_manhattan = planner2.make_plan(start, goal).unwrap();
    println!("Path length: {}", path_manhattan.len());
}
