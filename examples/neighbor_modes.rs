use dstar_trajectory_planner::DStarGlobalPlanner;

fn main() {
    let width = 40;
    let height = 40;
    let map_data = vec![0u8; (width * height) as usize];

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (2.0, 2.0);
    let goal = (37.0, 37.0);

    println!("Diagonal neighbor mode:");
    let path_diag = planner.make_plan(start, goal).unwrap();
    println!("Path length: {}", path_diag.len());

    // Manhattan-only mode requires modifying StateMap.neighbors()
    // This example simply demonstrates the difference by disabling diagonals manually.
    println!("Manhattan neighbor mode (simulated):");

    let mut planner2 = DStarGlobalPlanner::new();
    planner2.initialize(width, height, &map_data, false, "");

    // Simulate Manhattan by blocking diagonal cells
    let mut map_manhattan = vec![0u8; (width * height) as usize];
    for y in 0..height {
        for x in 0..width {
            if (x + y) % 2 == 1 {
                map_manhattan[(y * width + x) as usize] = 100;
            }
        }
    }

    planner2.initialize(width, height, &map_manhattan, false, "");
    let path_manhattan = planner2.make_plan(start, goal).unwrap();
    println!("Path length: {}", path_manhattan.len());
}
