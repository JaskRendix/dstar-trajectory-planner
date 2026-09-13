use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 80;
    let height = 40;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Create a narrow corridor
    for y in 0..height {
        for x in 0..width {
            if !(30..=50).contains(&x) {
                map_data[(y * width + x) as usize] = 100;
            }
        }
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (10.0, 20.0);
    let goal = (70.0, 20.0);

    println!("Running corridor test...");

    match planner.make_plan(start, goal) {
        Ok(path) => println!("Path generated with {} waypoints.", path.len()),
        Err(e) => println!("Failed: {}", e),
    }
}
