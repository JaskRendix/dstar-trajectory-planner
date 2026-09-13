use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 40;
    let height = 40;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (5.0, 5.0);
    let goal = (35.0, 35.0);

    println!("Generating path to populate potential field...");
    let path = planner.make_plan(start, goal).unwrap();
    println!("Optimized path length: {}", path.len());

    let grid = planner.grid().unwrap();

    // Convert path to a set for quick lookup
    let mut path_points = std::collections::HashSet::new();
    for &(x, y) in &path {
        path_points.insert((x as i64, y as i64));
    }

    println!("Potential field (ASCII):");
    for y in 0..height {
        for x in 0..width {
            let p = grid.point_ref(x, y).unwrap();

            let c = if path_points.contains(&(x, y)) {
                // Show potential only on optimized path
                let pot = p.potential;
                if pot < 1.0 {
                    '·'
                } else if pot < 5.0 {
                    'o'
                } else if pot < 10.0 {
                    'O'
                } else {
                    '#'
                }
            } else {
                // Non-path cells have no potential
                '.'
            };

            print!("{}", c);
        }
        println!();
    }
}
