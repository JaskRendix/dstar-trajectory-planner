use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Build a U-shaped obstacle
    for x in 10..50 {
        map_data[(10 * width + x) as usize] = 100; // top bar
    }
    for y in 10..40 {
        map_data[(y * width + 10) as usize] = 100; // left bar
        map_data[(y * width + 49) as usize] = 100; // right bar
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (30.0, 55.0);
    let goal = (30.0, 5.0);

    println!("Running U-shaped obstacle test...");

    match planner.make_plan(start, goal) {
        Ok(path) => {
            println!("Path generated with {} waypoints.", path.len());
        }
        Err(e) => println!("Failed: {}", e),
    }
}
