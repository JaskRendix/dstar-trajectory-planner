use dstar_trajectory_planner::{
    dstar::DStar,
    state_map::{StateMap, StateTag},
};

#[test]
fn test_uninitialized_origin_destination() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    let result = dstar.generate_trajectory();
    assert!(result.is_ok());
    assert!(result.unwrap().is_empty());
}

#[test]
fn test_blocked_path_returns_error() {
    let width = 10;
    let height = 10;
    let mut map = StateMap::new(width, height);

    for y in 0..height {
        if let Some(p) = map.point(5, y) {
            p.tag = StateTag::Obstacle;
        }
    }

    let mut dstar = DStar::new(map);
    dstar.init_targets(2, 2, 8, 8, false);

    let result = dstar.generate_trajectory();
    assert!(result.is_err());
}

#[test]
fn test_same_start_and_destination() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    dstar.init_targets(5, 5, 5, 5, false);
    let result = dstar.generate_trajectory();

    assert!(result.is_err());
}

#[test]
fn test_boundary_coordinates() {
    let width = 5;
    let height = 5;
    let map = StateMap::new(width, height);
    let mut dstar = DStar::new(map);

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

    dstar.set_cutoff_distance(-5);
    dstar.set_cutoff_distance(1000);

    dstar.set_repulsion_gain(-10.0);
    dstar.set_repulsion_gain(500.0);

    dstar.set_r_field(1);
    dstar.set_r_field(50);

    dstar.init_targets(0, 0, 4, 4, false);
    assert!(dstar.generate_trajectory().is_ok());
}

#[test]
fn test_weighted_init_targets_increases_weights() {
    let mut map = StateMap::new(10, 10);

    if let Some(p) = map.point(3, 3) {
        p.weight = 1;
        p.weight_previous = 1;
    }

    let mut dstar = DStar::new(map);
    dstar.init_targets(0, 0, 9, 9, true);

    let p = dstar.grid().point_ref(3, 3).unwrap();
    assert!(p.weight > 1);
}

#[test]
fn test_min_state_empty_via_generate_trajectory() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    let result = dstar.generate_trajectory();
    assert!(result.is_ok());
    assert!(result.unwrap().is_empty());
}

#[test]
fn test_reduce_path_error_via_generate_trajectory() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    dstar.init_targets(5, 5, 5, 5, false);
    let result = dstar.generate_trajectory();

    assert!(result.is_err());
}

#[test]
fn test_iterate_state_stops_when_origin_reached() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    dstar.init_targets(3, 3, 3, 3, false);
    let result = dstar.generate_trajectory();

    assert!(result.is_err());
}

#[test]
fn test_destination_change_resets_search_state() {
    let map = StateMap::new(10, 10);
    let mut dstar = DStar::new(map);

    dstar.init_targets(0, 0, 5, 5, false);
    let first = dstar.generate_trajectory();

    dstar.init_targets(0, 0, 7, 7, false);
    let second = dstar.generate_trajectory();

    assert!(first.is_ok());
    assert!(second.is_ok());
    assert_ne!(first.unwrap(), second.unwrap());
}
