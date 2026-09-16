use dstar_trajectory_planner::DStarGlobalPlanner;
use std::time::Instant;

fn potential_field_bench() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    for x in 25..=35 {
        for y in 25..=35 {
            let idx = (y * width + x) as usize;
            map_data[idx] = 100; // Lethal obstacle cost
        }
    }

    let mut planner = DStarGlobalPlanner::new();

    planner.set_repulsion_gain(60.0);
    planner.set_r_field(12);

    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    let t0 = Instant::now();
    let path = planner.make_plan(start, goal).unwrap();
    let dt = t0.elapsed();

    println!("Potential field benchmark (with central obstacles):");
    println!("Path length: {}", path.len());
    println!("Time: {:.3?}", dt);
}

fn main() {
    potential_field_bench();
}
