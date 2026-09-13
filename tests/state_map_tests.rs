use dstar_trajectory_planner::state_map::{NeighborMode, StateMap, StatePoint, StateTag};

#[test]
fn test_state_map_dimensions() {
    let map = StateMap::new(15, 25);
    assert_eq!(map.get_width(), 15);
    assert_eq!(map.get_height(), 25);
}

#[test]
fn test_point_access_and_bounds() {
    let mut map = StateMap::new(10, 10);

    let point = map.point(5, 5);
    assert!(point.is_some());
    let p = point.unwrap();
    assert_eq!(p.x, 5);
    assert_eq!(p.y, 5);
    assert_eq!(p.tag, StateTag::New);

    assert!(map.point(-1, 5).is_none());
    assert!(map.point(5, 10).is_none());
    assert!(map.point(10, 5).is_none());
    assert!(map.point_ref(99, 99).is_none());
}

#[test]
fn test_neighbor_generation_counts() {
    let map = StateMap::new(5, 5);

    let top_left_neighbors = map.neighbors(0, 0, NeighborMode::Eight);
    assert_eq!(top_left_neighbors.len(), 3);
    assert!(top_left_neighbors.contains(&(1, 0)));
    assert!(top_left_neighbors.contains(&(0, 1)));
    assert!(top_left_neighbors.contains(&(1, 1)));

    let edge_neighbors = map.neighbors(2, 0, NeighborMode::Eight);
    assert_eq!(edge_neighbors.len(), 5);

    let inner_neighbors = map.neighbors(2, 2, NeighborMode::Eight);
    assert_eq!(inner_neighbors.len(), 8);
}

#[test]
fn test_state_point_methods() {
    let mut point = StatePoint::new(3, 4);
    assert_eq!(point.float_coords(), (3.0, 4.0));

    assert_eq!(point.k(), -1);

    point.tag = StateTag::Open;
    point.cost_actual = 12;
    point.cost_previous = 10;
    assert_eq!(point.k(), 10);

    point.cost_actual = 5;
    assert_eq!(point.k(), 5);
}

#[test]
fn test_map_reset_functionality() {
    let mut map = StateMap::new(5, 5);

    if let Some(p) = map.point(1, 1) {
        p.tag = StateTag::Obstacle;
        p.weight_previous = 100;
        p.weight = 100;
    }

    if let Some(p) = map.point(2, 2) {
        p.tag = StateTag::Open;
        p.cost_actual = 5;
        p.cost_previous = 4;
        p.backpointer = Some((2, 1));
        p.potential = 1.5;
    }

    map.reset();

    let obs = map.point_ref(1, 1).unwrap();
    assert_eq!(obs.tag, StateTag::Obstacle);
    assert_eq!(obs.weight, 100);

    let normal = map.point_ref(2, 2).unwrap();
    assert_eq!(normal.tag, StateTag::New);
    assert_eq!(normal.cost_actual, -1);
    assert_eq!(normal.cost_previous, -1);
    assert!(normal.backpointer.is_none());
    assert_eq!(normal.potential, 0.0);
}
