use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    // Create a weighted band that is not an obstacle
    for y in 20..40 {
        for x in 0..60 {
            map_data[(y * width + x) as usize] = 30; // medium cost
        }
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    println!("Running weighted costmap example...");

    let path = planner.make_plan(start, goal).unwrap();
    println!("Path length: {}", path.len());
}
