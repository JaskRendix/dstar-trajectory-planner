use clap::Parser;
use dstar_trajectory_planner::{DStarGlobalPlanner, NeighborMode};

#[derive(Parser)]
#[command(name = "dstar-cli", about = "CLI interface for D* trajectory planner")]
struct Args {
    #[arg(long, default_value = "50.0")]
    repulsion_gain: f64,

    #[arg(long, default_value = "10")]
    potential_radius: i32,

    #[arg(long, default_value = "16")]
    cutoff_distance: i32,

    #[arg(long, default_value = "64")]
    occupancy_threshold: i32,

    #[arg(long, default_value = "eight")]
    neighbor_mode: String,

    #[arg(long)]
    verbose: bool,

    #[arg(long, default_value = "false")]
    erosion: bool,

    #[arg(long, default_value = "2")]
    erosion_gap: i64,

    #[arg(long, default_value = "0.0")]
    start_x: f64,

    #[arg(long, default_value = "0.0")]
    start_y: f64,

    #[arg(long, default_value = "50.0")]
    goal_x: f64,

    #[arg(long, default_value = "50.0")]
    goal_y: f64,

    #[arg(long, default_value = "60")]
    width: i64,

    #[arg(long, default_value = "60")]
    height: i64,

    #[arg(long, default_value = "")]
    paths_file: String,

    #[arg(long, default_value = "false")]
    enable_ready_paths: bool,
}

fn main() {
    let args = Args::parse();

    // Build map
    let mut map_data = vec![0u8; (args.width * args.height) as usize];
    for i in 20..40 {
        let idx = (i * args.width + i) as usize;
        map_data[idx] = args.occupancy_threshold as u8;
    }

    // Planner
    let mut planner = DStarGlobalPlanner::new();
    planner.set_verbose(args.verbose);
    planner.set_repulsion_gain(args.repulsion_gain);
    planner.set_r_field(args.potential_radius);
    planner.set_cutoff_distance(args.cutoff_distance);
    planner.set_erosion(args.erosion);
    planner.set_erosion_gap(args.erosion_gap);

    match args.neighbor_mode.as_str() {
        "four" => planner.set_neighbor_mode(NeighborMode::Four),
        "eight" => planner.set_neighbor_mode(NeighborMode::Eight),
        _ => {
            println!(
                "Unknown neighbor mode '{}', using default (eight)",
                args.neighbor_mode
            );
            planner.set_neighbor_mode(NeighborMode::Eight);
        }
    }

    planner.initialize(
        args.width,
        args.height,
        &map_data,
        args.enable_ready_paths,
        &args.paths_file,
    );

    let start = (args.start_x, args.start_y);
    let goal = (args.goal_x, args.goal_y);

    println!("Running D* from {:?} to {:?}", start, goal);

    match planner.make_plan(start, goal) {
        Ok(path) => {
            println!("Generated {} waypoints:", path.len());

            let mut grid = map_data.clone();
            let w = args.width as usize;

            for (x, y) in &path {
                let idx = (*y as usize) * w + (*x as usize);
                grid[idx] = 200;
            }

            for y in 0..args.height {
                for x in 0..args.width {
                    let v = grid[(y * args.width + x) as usize];
                    let ch = if v >= 200 {
                        '*'
                    } else if v >= args.occupancy_threshold as u8 {
                        '#'
                    } else {
                        '.'
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
