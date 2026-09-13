use dstar_trajectory_planner::DStarGlobalPlanner;
use std::time::Instant;

fn weighted_costmap_bench() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Weighted band
    for y in 20..40 {
        for x in 0..60 {
            map_data[(y * width + x) as usize] = 30;
        }
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Weighted costmap benchmark:");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);
}

fn main() {
    weighted_costmap_bench();
}
