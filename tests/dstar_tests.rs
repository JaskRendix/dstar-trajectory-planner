use dstar_trajectory_planner::{
    dstar::DStar,
    state_map::{StateMap, StateTag},
};

#[test]
fn test_uninitialized_origin_destination() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    // Generating trajectory without init_targets should return an empty path successfully
    let result = dstar.generate_trajectory();
    assert!(result.is_ok());
    assert!(result.unwrap().is_empty());
}

#[test]
fn test_blocked_path_returns_error() {
    let width = 10;
    let height = 10;
    let mut map = StateMap::new(width, height);

    // Completely wall off column 5 with obstacles
    for y in 0..height {
        if let Some(p) = map.point(5, y) {
            p.tag = StateTag::Obstacle;
        }
    }

    let mut dstar = DStar::new(map);
    // Try to route from (2, 2) to (8, 8) through the wall
    dstar.init_targets(2, 2, 8, 8, false);

    let result = dstar.generate_trajectory();
    assert!(
        result.is_err(),
        "Expected an error when path is completely blocked"
    );
}

#[test]
fn test_same_start_and_destination() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    dstar.init_targets(5, 5, 5, 5, false);
    let result = dstar.generate_trajectory();

    // When start equals destination, trace_path usually produces a single point or empty path
    // which should trigger the draft_path.len() <= 1 error condition.
    assert!(result.is_err());
}

#[test]
fn test_boundary_coordinates() {
    let width = 5;
    let height = 5;
    let map = StateMap::new(width, height);
    let mut dstar = DStar::new(map);

    // Initialize targets at extreme corner boundaries
    dstar.init_targets(0, 0, 4, 4, true);
    let result = dstar.generate_trajectory();

    assert!(result.is_ok());
    let path = result.unwrap();
    assert!(!path.is_empty());
    assert_eq!(path.first().copied(), Some((0, 0)));
}

#[test]
fn test_setter_boundaries() {
    let map = StateMap::new(5, 5);
    let mut dstar = DStar::new(map);

    // Test extreme or out-of-bounds parameters to ensure no panics occur
    dstar.set_cutoff_distance(-5);
    dstar.set_cutoff_distance(1000);

    dstar.set_repulsion_gain(-10.0); // Should clamp or handle gracefully
    dstar.set_repulsion_gain(500.0);

    dstar.set_r_field(1); // Below minimum threshold bounds check
    dstar.set_r_field(50);

    dstar.init_targets(0, 0, 4, 4, false);
    assert!(dstar.generate_trajectory().is_ok());
}
