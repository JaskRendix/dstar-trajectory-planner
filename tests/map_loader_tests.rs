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

#[test]
fn test_load_mock_map_virtual_walls_do_not_modify_free_cells() {
    let (width, _height, map_data) = load_mock_map();

    let free_cells = [(0, 0), (5, 10), (49, 49), (10, 30)];

    for (x, y) in free_cells {
        let idx = (y * width + x) as usize;
        assert_eq!(map_data[idx], 0);
    }
}

#[test]
fn test_load_mock_map_obstacle_wall_continuity() {
    let (width, _height, map_data) = load_mock_map();

    for y in 10..40 {
        let idx = (y * width + 25) as usize;
        assert_eq!(map_data[idx], 100);
    }
}

#[test]
fn test_load_mock_map_no_unintended_obstacles() {
    let (width, height, map_data) = load_mock_map();

    for y in 0..height {
        for x in 0..width {
            if x != 25 || !(10..40).contains(&y) {
                let idx = (y * width + x) as usize;
                assert_eq!(map_data[idx], 0);
            }
        }
    }
}
