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

#[test]
fn test_neighbor_mode_four() {
    let map = StateMap::new(5, 5);

    let n = map.neighbors(2, 2, NeighborMode::Four);
    assert_eq!(n.len(), 4);
    assert!(n.contains(&(3, 2)));
    assert!(n.contains(&(1, 2)));
    assert!(n.contains(&(2, 3)));
    assert!(n.contains(&(2, 1)));
}

#[test]
fn test_neighborhood_radius() {
    let map = StateMap::new(10, 10);

    let cells = map.neighborhood(5, 5, 2);
    // radius 2 circle contains 13 cells
    assert_eq!(cells.len(), 13);

    // ensure center is included
    assert!(cells.contains(&(5, 5)));

    // ensure a point outside radius is excluded
    assert!(!cells.contains(&(5, 5 + 3)));
}

#[test]
fn test_point_ref_indexing() {
    let map = StateMap::new(10, 10);

    assert!(map.point_ref(0, 0).is_some());
    assert!(map.point_ref(1, 0).is_some());
    assert!(map.point_ref(0, 1).is_some());
    assert!(map.point_ref(9, 9).is_some());

    assert!(map.point_ref(10, 0).is_none());
    assert!(map.point_ref(0, 10).is_none());
}

#[test]
fn test_reset_preserves_all_obstacles() {
    let mut map = StateMap::new(5, 5);

    for (x, y) in [(1, 1), (2, 2), (3, 3)] {
        let p = map.point(x, y).unwrap();
        p.tag = StateTag::Obstacle;
        p.weight_previous = 99;
        p.weight = 99;
    }

    map.reset();

    for (x, y) in [(1, 1), (2, 2), (3, 3)] {
        let p = map.point_ref(x, y).unwrap();
        assert_eq!(p.tag, StateTag::Obstacle);
        assert_eq!(p.weight, 99);
    }
}

#[test]
fn test_state_point_new_initialization() {
    let p = StatePoint::new(7, 8);

    assert_eq!(p.x, 7);
    assert_eq!(p.y, 8);
    assert_eq!(p.tag, StateTag::New);
    assert_eq!(p.weight, 0);
    assert_eq!(p.weight_previous, 0);
    assert_eq!(p.cost_actual, -1);
    assert_eq!(p.cost_previous, -1);
    assert_eq!(p.potential, 0.0);
    assert!(p.backpointer.is_none());
}

#[test]
fn test_neighbors_all_corners() {
    let map = StateMap::new(5, 5);

    // top-left
    assert_eq!(map.neighbors(0, 0, NeighborMode::Eight).len(), 3);

    // top-right
    assert_eq!(map.neighbors(4, 0, NeighborMode::Eight).len(), 3);

    // bottom-left
    assert_eq!(map.neighbors(0, 4, NeighborMode::Eight).len(), 3);

    // bottom-right
    assert_eq!(map.neighbors(4, 4, NeighborMode::Eight).len(), 3);
}

#[test]
fn test_neighbor_symmetry() {
    let map = StateMap::new(10, 10);

    for x in 0..10 {
        for y in 0..10 {
            let neigh = map.neighbors(x, y, NeighborMode::Eight);
            for (nx, ny) in neigh {
                let back = map.neighbors(nx, ny, NeighborMode::Eight);
                assert!(back.contains(&(x, y)));
            }
        }
    }
}

#[test]
fn test_point_and_point_ref_consistency() {
    let mut map = StateMap::new(10, 10);

    if let Some(p_mut) = map.point(4, 4) {
        p_mut.tag = StateTag::Closed;
    }

    let p_ref = map.point_ref(4, 4).unwrap();
    assert_eq!(p_ref.tag, StateTag::Closed);
}

#[test]
fn test_reset_large_map() {
    let mut map = StateMap::new(200, 200);

    // Imposta alcuni ostacoli
    for i in 0..200 {
        if let Some(p) = map.point(i, i) {
            p.tag = StateTag::Obstacle;
            p.weight_previous = 99;
            p.weight = 99;
        }
    }

    map.reset();

    for i in 0..200 {
        let p = map.point_ref(i, i).unwrap();
        assert_eq!(p.tag, StateTag::Obstacle);
        assert_eq!(p.weight, 99);
    }
}

#[test]
fn test_backpointer_reset() {
    let mut map = StateMap::new(5, 5);

    if let Some(p) = map.point(3, 3) {
        p.backpointer = Some((2, 2));
    }

    map.reset();

    assert!(map.point_ref(3, 3).unwrap().backpointer.is_none());
}

#[test]
fn test_potential_reset() {
    let mut map = StateMap::new(5, 5);

    if let Some(p) = map.point(2, 2) {
        p.potential = 42.0;
    }

    map.reset();

    assert_eq!(map.point_ref(2, 2).unwrap().potential, 0.0);
}
