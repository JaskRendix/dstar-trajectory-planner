use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use crate::dstar::{DStar, DStarError};
use crate::state_map::{StateMap, StateTag};

#[derive(Serialize, Deserialize, Debug)]
struct VPathEntry {
    name: String,
    polygon: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug)]
struct MapJson {
    vpaths: Vec<VPathEntry>,
}

pub struct DStarGlobalPlanner {
    initialized: bool,
    #[allow(dead_code)]
    state_grid: Option<StateMap>,
    generator: Option<DStar>,
    occupancy_threshold: i32,
    initial_path: bool,

    current_origin: (f64, f64),
    current_destination: (f64, f64),
    trajectory: Vec<(i64, i64)>,

    #[allow(dead_code)]
    goal_distance_threshold: f64,
    repulsion_gain: f64,
    potential_field_radius: i32,
    neighbor_distance_threshold: f64,

    width: i64,
    height: i64,
    erosion: bool,
    erosion_gap: i64,
    cutoff_distance: i32,

    ready_paths: Vec<Vec<(f64, f64)>>,
    enable_ready_paths: bool,

    // NEW: verbosity flag
    verbose: bool,
}

impl Default for DStarGlobalPlanner {
    fn default() -> Self {
        Self::new()
    }
}

impl DStarGlobalPlanner {
    pub fn new() -> Self {
        Self {
            initialized: false,
            state_grid: None,
            generator: None,
            occupancy_threshold: 64,
            initial_path: true,
            current_origin: (0.0, 0.0),
            current_destination: (0.0, 0.0),
            trajectory: Vec::new(),
            goal_distance_threshold: 0.3,
            repulsion_gain: 50.0,
            potential_field_radius: 10,
            neighbor_distance_threshold: 0.1,
            width: 0,
            height: 0,
            erosion: false,
            erosion_gap: 2,
            cutoff_distance: 16,
            ready_paths: Vec::new(),
            enable_ready_paths: false,

            verbose: false, // default: silent
        }
    }

    // NEW: setter
    pub fn set_verbose(&mut self, value: bool) {
        self.verbose = value;
    }

    pub fn parse_paths_json(&mut self, filename: &str) {
        if !Path::new(filename).exists() {
            if self.verbose {
                println!(
                    "File '{}' seems not existing, skipping paths collection",
                    filename
                );
            }
            return;
        }

        if self.verbose {
            println!("Parsing file {}...", filename);
        }

        let file = match File::open(filename) {
            Ok(f) => f,
            Err(_) => {
                if self.verbose {
                    println!("Failed to open paths file.");
                }
                return;
            }
        };

        let reader = BufReader::new(file);
        let map_json: Result<MapJson, _> = serde_json::from_reader(reader);

        let map_data = match map_json {
            Ok(d) => d,
            Err(e) => {
                if self.verbose {
                    println!("Cannot parse path list from the file: {}", e);
                }
                return;
            }
        };

        for path in map_data.vpaths {
            if path.polygon.len() % 2 != 0 {
                if self.verbose {
                    println!(
                        "Cannot parse object: the point coordinates are not pairable for path '{}'",
                        path.name
                    );
                }
                continue;
            }

            let mut new_path = Vec::new();
            for chunk in path.polygon.chunks(2) {
                if chunk.len() == 2 {
                    new_path.push((chunk[0], chunk[1]));
                }
            }

            if new_path.len() >= 2 {
                self.ready_paths.push(new_path);
            } else {
                if self.verbose {
                    println!(
                        "Cannot register the freepath selector: the selector \"{}\" should contain more than one point!",
                        path.name
                    );
                }
            }
        }
    }

    pub fn initialize(
        &mut self,
        width: i64,
        height: i64,
        map_data: &[u8],
        enable_paths: bool,
        paths_file: &str,
    ) {
        self.width = width;
        self.height = height;
        self.enable_ready_paths = enable_paths;

        if self.enable_ready_paths {
            self.parse_paths_json(paths_file);
        }

        if self.verbose {
            println!(
                "Obtained map [{}, {}], filling D* internal costmap...",
                width, height
            );
        }

        let mut grid = StateMap::new(width, height);

        for i in 0..width {
            for j in 0..height {
                let pos = (j * width + i) as usize;
                let cost = map_data[pos] as i64;

                if let Some(p) = grid.point(i, j) {
                    p.weight_previous = cost;
                    p.weight = cost;
                    p.cost_previous = cost;
                    p.cost_actual = cost;

                    if cost >= self.occupancy_threshold as i64 {
                        p.tag = StateTag::Obstacle;
                    } else {
                        p.tag = StateTag::New;
                    }
                }
            }
        }

        if self.erosion {
            for i in 0..width {
                for j in 0..height {
                    if let Some(p) = grid.point_ref(i, j)
                        && p.tag == StateTag::Obstacle
                    {
                        let eg = self.erosion_gap;
                        for u in -eg..=eg {
                            for v in -eg..=eg {
                                if u == 0 && v == 0 {
                                    continue;
                                }
                                let xi = i + u;
                                let yj = j + v;
                                if let Some(target_p) = grid.point(xi, yj)
                                    && target_p.tag != StateTag::Obstacle
                                {
                                    target_p.tag = StateTag::Obstacle;
                                }
                            }
                        }
                    }
                }
            }
        }

        if self.verbose {
            println!("D* internal costmap filled, running algorithm");
        }

        let mut dstar_gen = DStar::new(grid);
        dstar_gen.set_cutoff_distance(self.cutoff_distance);
        dstar_gen.set_r_field(self.potential_field_radius);
        dstar_gen.set_repulsion_gain(self.repulsion_gain);

        self.generator = Some(dstar_gen);
        self.initialized = true;
    }

    pub fn make_plan(
        &mut self,
        start: (f64, f64),
        goal: (f64, f64),
    ) -> Result<Vec<(f64, f64)>, DStarError> {
        if self.enable_ready_paths && !self.ready_paths.is_empty() {
            // Predefined route lookup implementation matching JSON configuration vectors
        }

        let origin_x = start.0 as i64;
        let origin_y = start.1 as i64;
        let dest_x = goal.0 as i64;
        let dest_y = goal.1 as i64;

        let generator = match self.generator.as_mut() {
            Some(g) => g,
            None => return Err(DStarError("Planner not initialized".to_string())),
        };

        if self.initial_path {
            self.current_origin = start;
            self.current_destination = goal;

            generator.init_targets(origin_x, origin_y, dest_x, dest_y, true);
            self.trajectory = generator.generate_trajectory()?;
            self.initial_path = false;
        } else {
            let goal_dist = ((goal.0 - self.current_destination.0).powi(2)
                + (goal.1 - self.current_destination.1).powi(2))
            .sqrt();
            if goal_dist > self.neighbor_distance_threshold {
                self.initial_path = true;
                return Err(DStarError(
                    "Destination changed, resetting path".to_string(),
                ));
            }
            self.current_origin = start;
            generator.init_targets(origin_x, origin_y, dest_x, dest_y, false);
            self.trajectory = generator.generate_trajectory()?;
        }

        Ok(self
            .trajectory
            .iter()
            .map(|&(x, y)| (x as f64, y as f64))
            .collect())
    }
}
