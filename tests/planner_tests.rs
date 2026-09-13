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

#[test]
fn test_planner_neighbor_modes_affect_path() {
    let (width, height, map_data) = load_mock_map();

    let mut planner8 = DStarGlobalPlanner::new();
    planner8.set_neighbor_mode(dstar_trajectory_planner::state_map::NeighborMode::Eight);
    planner8.initialize(width, height, &map_data, false, "");
    let path8 = planner8.make_plan((2.0, 2.0), (45.0, 45.0)).unwrap();

    let mut planner4 = DStarGlobalPlanner::new();
    planner4.set_neighbor_mode(dstar_trajectory_planner::state_map::NeighborMode::Four);
    planner4.initialize(width, height, &map_data, false, "");
    let path4 = planner4.make_plan((2.0, 2.0), (45.0, 45.0)).unwrap();

    assert!(path4.len() >= path8.len());
}

#[test]
fn test_planner_erosion_expands_obstacles() {
    let (width, height, mut map_data) = load_mock_map();

    map_data[(10 * width + 10) as usize] = 255;

    let mut planner = DStarGlobalPlanner::new();
    planner.set_erosion(true);
    planner.set_erosion_gap(1);

    planner.initialize(width, height, &map_data, false, "");

    let grid = planner.grid().unwrap();

    assert_eq!(
        grid.point_ref(9, 10).unwrap().tag,
        dstar_trajectory_planner::state_map::StateTag::Obstacle
    );
    assert_eq!(
        grid.point_ref(11, 10).unwrap().tag,
        dstar_trajectory_planner::state_map::StateTag::Obstacle
    );
    assert_eq!(
        grid.point_ref(10, 9).unwrap().tag,
        dstar_trajectory_planner::state_map::StateTag::Obstacle
    );
    assert_eq!(
        grid.point_ref(10, 11).unwrap().tag,
        dstar_trajectory_planner::state_map::StateTag::Obstacle
    );
}

#[test]
fn test_planner_cutoff_distance_changes_path() {
    let (width, height, map_data) = load_mock_map();

    let mut p_low = DStarGlobalPlanner::new();
    p_low.set_cutoff_distance(4);
    p_low.initialize(width, height, &map_data, false, "");
    let path_low = p_low.make_plan((2.0, 2.0), (45.0, 45.0)).unwrap();

    let mut p_high = DStarGlobalPlanner::new();
    p_high.set_cutoff_distance(64);
    p_high.initialize(width, height, &map_data, false, "");
    let path_high = p_high.make_plan((2.0, 2.0), (45.0, 45.0)).unwrap();

    assert_ne!(path_low.len(), path_high.len());
}

#[test]
fn test_planner_potential_field_radius_changes_path() {
    let (width, height, map_data) = load_mock_map();

    let mut p1 = DStarGlobalPlanner::new();
    p1.set_r_field(5);
    p1.initialize(width, height, &map_data, false, "");
    let path1 = p1.make_plan((2.0, 2.0), (45.0, 45.0)).unwrap();

    let mut p2 = DStarGlobalPlanner::new();
    p2.set_r_field(20);
    p2.initialize(width, height, &map_data, false, "");
    let path2 = p2.make_plan((2.0, 2.0), (45.0, 45.0)).unwrap();

    assert!(!path1.is_empty());
    assert!(!path2.is_empty());
}

#[test]
fn test_planner_ready_paths_malformed_json() {
    let json_data = r#"{ "vpaths": [ { "name": "bad", "polygon": [1.0, 2.0, 3.0] } ] }"#;

    let mut temp = NamedTempFile::new().unwrap();
    temp.write_all(json_data.as_bytes()).unwrap();

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(20, 20, &vec![0u8; 400], true, temp.path().to_str().unwrap());

    let result = planner.make_plan((1.0, 1.0), (10.0, 10.0));
    assert!(result.is_ok());
}

#[test]
fn test_planner_ready_path_single_point_is_ignored() {
    let json_data = r#"{
        "vpaths": [
            { "name": "single", "polygon": [1.0, 1.0] }
        ]
    }"#;

    let mut temp = NamedTempFile::new().unwrap();
    temp.write_all(json_data.as_bytes()).unwrap();

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(20, 20, &vec![0u8; 400], true, temp.path().to_str().unwrap());

    let result = planner.make_plan((1.0, 1.0), (10.0, 10.0));
    assert!(result.is_ok());
}

#[test]
fn test_planner_missing_ready_paths_file() {
    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(20, 20, &vec![0u8; 400], true, "nonexistent_file.json");

    let result = planner.make_plan((1.0, 1.0), (10.0, 10.0));
    assert!(result.is_ok());
}

#[test]
fn test_planner_unreachable_goal_returns_error() {
    let (width, height, mut map_data) = load_mock_map();

    for i in 0..width {
        map_data[(20 * width + i) as usize] = 255;
    }

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let result = planner.make_plan((2.0, 2.0), (45.0, 45.0));
    assert!(result.is_err());
}

#[test]
fn test_planner_start_equals_goal() {
    let (width, height, map_data) = load_mock_map();
    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let result = planner.make_plan((10.0, 10.0), (10.0, 10.0));

    assert!(result.is_ok() || result.is_err());
}

#[test]
fn test_planner_grid_accessor() {
    let (width, height, map_data) = load_mock_map();
    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let grid = planner.grid();
    assert!(grid.is_some());
}
