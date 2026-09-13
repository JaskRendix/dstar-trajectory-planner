use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    println!("Initial plan...");
    let path1 = planner.make_plan(start, goal).unwrap();
    println!("Initial path length: {}", path1.len());

    // Add a new obstacle blocking the path
    for x in 10..50 {
        map_data[(30 * width + x) as usize] = 100;
    }

    planner.initialize(width, height, &map_data, false, "");

    println!("Replanning after obstacle update...");
    let path2 = planner.make_plan(start, goal).unwrap();
    println!("Updated path length: {}", path2.len());
}
