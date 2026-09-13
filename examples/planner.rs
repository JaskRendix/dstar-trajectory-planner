use dstar_trajectory_planner::{DStarGlobalPlanner, VirtualWallsModule};

fn main() {
    let width = 60;
    let height = 60;
    let mut map_data = vec![0u8; (width * height) as usize];

    for i in 20..40 {
        let idx = (i * width + i) as usize;
        map_data[idx] = 100;
    }

    let walls_module = VirtualWallsModule::new(0);
    walls_module.apply_to_grid(
        &mut map_data,
        width as usize,
        height as usize,
        1.0,
        0.0,
        0.0,
        100,
        0,
    );

    let mut planner = DStarGlobalPlanner::new();
    planner.initialize(width, height, &map_data, false, "");

    let start = (10.0, 50.0);
    let goal = (50.0, 10.0);

    println!(
        "Running diagonal obstacle test from {:?} to {:?}...",
        start, goal
    );

    match planner.make_plan(start, goal) {
        Ok(path) => {
            println!("Alternative path generated with {} waypoints.", path.len());

            println!("\nPath waypoints:");
            for (i, p) in path.iter().enumerate() {
                println!("{:3}: {:?}", i, p);
            }

            let mut grid = map_data.clone();

            let w = width as usize;

            for (x, y) in &path {
                let idx = (*y as usize) * w + (*x as usize);
                grid[idx] = 200;
            }

            println!("\nGrid visualization:");
            for y in 0..height {
                for x in 0..width {
                    let v = grid[(y * width + x) as usize];
                    let ch = if v >= 200 {
                        '*' // path
                    } else if v >= 100 {
                        '#' // obstacle
                    } else {
                        '.' // free
                    };
                    print!("{}", ch);
                }
                println!();
            }
        }
        Err(e) => {
            println!("Path planning failed: {}", e);
        }
    }
}
