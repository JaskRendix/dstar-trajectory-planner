use dstar_trajectory_planner::{DStarGlobalPlanner, load_mock_map};
use std::io::Write;
use tempfile::NamedTempFile;

#[test]
fn test_planner_default_instantiation() {
    let _planner = DStarGlobalPlanner::new();
}

#[test]
fn test_planner_initialization_and_successful_path() {
    let (width, height, map_data) = load_mock_map();
    let mut planner = DStarGlobalPlanner::new();

    planner.initialize(width, height, &map_data, false, "");

    let start = (2.0, 2.0);
    let goal = (45.0, 45.0);

    let result = planner.make_plan(start, goal);
    assert!(result.is_ok(), "Expected valid trajectory generation");

    let path = result.unwrap();
    assert!(!path.is_empty());
    assert_eq!(path.first().copied(), Some((2.0, 2.0)));
}

#[test]
fn test_planner_uninitialized_execution() {
    let mut planner = DStarGlobalPlanner::new();
    let result = planner.make_plan((0.0, 0.0), (10.0, 10.0));

    assert!(result.is_err(), "Planning without initialization must fail");
}

#[test]
fn test_planner_goal_change_triggers_reset() {
    let (width, height, map_data) = load_mock_map();
    let mut planner = DStarGlobalPlanner::new();

    planner.initialize(width, height, &map_data, false, "");

    let _ = planner.make_plan((2.0, 2.0), (10.0, 10.0));

    let result = planner.make_plan((2.0, 2.0), (40.0, 40.0));
    assert!(
        result.is_err(),
        "Changing destination past threshold should return reset error indicator"
    );
}

#[test]
fn test_planner_with_ready_paths_json() {
    let json_data = r#"{
        "vpaths": [
            {
                "name": "corridor_1",
                "polygon": [1.0, 1.0, 15.0, 15.0]
            }
        ]
    }"#;

    let mut temp_file = NamedTempFile::new().unwrap();
    temp_file.write_all(json_data.as_bytes()).unwrap();
    let path_str = temp_file.path().to_str().unwrap();

    let mut planner = DStarGlobalPlanner::new();
    let width = 30;
    let height = 30;
    let map_data = vec![0u8; (width * height) as usize];

    planner.initialize(width, height, &map_data, true, path_str);

    let result = planner.make_plan((2.0, 2.0), (25.0, 25.0));
    assert!(result.is_ok());
}
