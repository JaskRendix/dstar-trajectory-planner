use dstar_trajectory_planner::DStarGlobalPlanner;
use std::time::Instant;

fn potential_field_bench() {
    let width = 60;
    let height = 60;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Potential field benchmark:");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);

    // Optional: inspect potentials if you expose get_grid()
}

fn main() {
    potential_field_bench();
}
