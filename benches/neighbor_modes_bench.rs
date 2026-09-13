use dstar_trajectory_planner::DStarGlobalPlanner;
use std::time::Instant;

fn bench_diagonal() {
    let width = 80;
    let height = 80;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (75.0, 75.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Diagonal mode:");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);
}

fn bench_manhattan() {
    let width = 80;
    let height = 80;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Simulate Manhattan by blocking diagonal cells
    for y in 0..height {
        for x in 0..width {
            if (x + y) % 2 == 1 {
                map_data[(y * width + x) as usize] = 100;
            }
        }
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (75.0, 75.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Manhattan mode (simulated):");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);
}

fn main() {
    bench_diagonal();
    bench_manhattan();
}
