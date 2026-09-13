use dstar_trajectory_planner::{DStarGlobalPlanner, map_loader::load_mock_map};

const START_X: f64 = 5.0;
const START_Y: f64 = 5.0;
const GOAL_X: f64 = 45.0;
const GOAL_Y: f64 = 45.0;
const ENABLE_READY_PATHS: bool = false;
const PATHS_FILE_PATH: &str = "";

fn main() {
    let (width, height, map_data) = load_mock_map();

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(
        width,
        height,
        &map_data,
        ENABLE_READY_PATHS,
        PATHS_FILE_PATH,
    );

    let start = (START_X, START_Y);
    let goal = (GOAL_X, GOAL_Y);

    println!("Calculating trajectory from {:?} to {:?}...", start, goal);

    match planner.make_plan(start, goal) {
        Ok(path) => {
            println!(
                "Trajectory successfully generated with {} waypoints:",
                path.len()
            );
            for (i, pt) in path.iter().enumerate() {
                println!("  [{}] X: {}, Y: {}", i, pt.0, pt.1);
            }
        }
        Err(e) => {
            println!("Path planning failed: {}", e);
        }
    }
}
