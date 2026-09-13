use dstar_trajectory_planner::{DStarGlobalPlanner, VirtualWallsModule};
use std::fs::File;
use std::io::Write;

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![100u8; (width * height) as usize]; // fully blocked

    // Create a temporary JSON file describing a virtual path
    let json = r#"
    {
        "vwalls": [],
        "vpaths": [
            {
                "name": "free_corridor",
                "polygon": [5,5, 10,10, 20,20, 40,40, 55,55]
            }
        ]
    }
    "#;

    let filename = "virtual_paths_tmp.json";
    let mut file = File::create(filename).expect("Cannot create temp JSON");
    file.write_all(json.as_bytes()).expect("Cannot write JSON");

    // Load virtual paths from JSON
    let mut walls = VirtualWallsModule::new(100);
    walls.parse_walls_json(filename);

    // Apply virtual paths to the grid
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

    let start = (5.0, 5.0);
    let goal = (55.0, 55.0);

    println!("Running virtual paths test...");

    match planner.make_plan(start, goal) {
        Ok(path) => println!("Path generated with {} waypoints.", path.len()),
        Err(e) => println!("Failed: {}", e),
    }
}
