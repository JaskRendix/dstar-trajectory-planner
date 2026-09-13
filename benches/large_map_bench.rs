use dstar_trajectory_planner::DStarGlobalPlanner;
use std::time::Instant;

fn large_map_bench() {
    let width = 1000;
    let height = 1000;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (10.0, 10.0);
    let goal = (990.0, 990.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Large-map stress test:");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);
}

fn main() {
    large_map_bench();
}
