use dstar_trajectory_planner::{DStarGlobalPlanner, VirtualWallsModule};

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    let mut walls = VirtualWallsModule::new(100);
    walls.parse_walls_json("virtual_walls.json");
    walls.apply_to_grid(
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

    let start = (5.0, 55.0);
    let goal = (55.0, 5.0);

    println!("Running virtual walls JSON test...");

    match planner.make_plan(start, goal) {
        Ok(path) => println!("Path generated with {} waypoints.", path.len()),
        Err(e) => println!("Failed: {}", e),
    }
}
