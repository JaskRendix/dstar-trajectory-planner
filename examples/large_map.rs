use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 1000;
    let height = 1000;

    // Empty map for maximum performance throughput
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (10.0, 10.0);
    let goal = (990.0, 990.0);

    println!("Running large-map stress test...");

    let path = planner.make_plan(start, goal).unwrap();
    println!("Path length: {}", path.len());
}
