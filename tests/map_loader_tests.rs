use dstar_trajectory_planner::load_mock_map;

#[test]
fn test_load_mock_map_dimensions() {
    let (width, height, map_data) = load_mock_map();
    assert_eq!(width, 50);
    assert_eq!(height, 50);
    assert_eq!(map_data.len(), (width * height) as usize);
}

#[test]
fn test_load_mock_map_obstacles_and_free_space() {
    let (width, _, map_data) = load_mock_map();

    let wall_idx = (20 * width + 25) as usize;
    assert_eq!(map_data[wall_idx], 100);

    let free_idx = (5 * width + 5) as usize;
    assert_eq!(map_data[free_idx], 0);
}
