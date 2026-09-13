use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Simple maze pattern
    for y in 5..55 {
        if y % 10 != 0 {
            for x in 10..50 {
                map_data[(y * width + x) as usize] = 100;
            }
        }
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    println!("Running maze test...");

    match planner.make_plan(start, goal) {
        Ok(path) => println!("Maze path generated with {} waypoints.", path.len()),
        Err(e) => println!("Failed: {}", e),
    }
}
