use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 20;
    let height = 20;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (1.0, 1.0);
    let goal = (18.0, 18.0);

    let path = planner.make_plan(start, goal).unwrap();
    println!("Minimal example path length: {}", path.len());
}
