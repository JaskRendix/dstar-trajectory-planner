use dstar_trajectory_planner::{DStarGlobalPlanner, VirtualWallsModule};

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Create a diagonal barrier instead of a straight line
    for i in 20..40 {
        let idx = (i * width + i) as usize;
        map_data[idx] = 100;
    }

    let walls_module = VirtualWallsModule::new(0);
    walls_module.apply_to_grid(
        &mut map_data,
        width as usize,
        height as usize,
        1.0,
        0.0,
        0.0,
        100,
        0,
    );

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    // Different start and goal across the diagonal barrier
    let start = (10.0, 50.0);
    let goal = (50.0, 10.0);

    println!(
        "Running diagonal obstacle test from {:?} to {:?}...",
        start, goal
    );

    match planner.make_plan(start, goal) {
        Ok(path) => {
            println!("Alternative path generated with {} waypoints:", path.len());
        }
        Err(e) => {
            println!("Path planning failed: {}", e);
        }
    }
}
