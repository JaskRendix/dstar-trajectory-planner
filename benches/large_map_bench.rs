use dstar_trajectory_planner::DStarGlobalPlanner;
use std::time::Instant;

fn large_map_bench() {
    let width = 1000;
    let height = 1000;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Add a vertical wall down the middle at x = 500, with a gap from y = 400 to 600
    for y in 0..height {
        if !(400..=600).contains(&y) {
            let idx = (y * width + 500) as usize;
            map_data[idx] = 100; // Lethal obstacle cost
        }
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.set_verbose(false);
    planner.initialize(width, height, &map_data, false, "");

    let start = (10.0, 10.0);
    let goal = (990.0, 990.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Large-map stress test (1000x1000 with obstacle wall):");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);
}

fn main() {
    large_map_bench();
}
