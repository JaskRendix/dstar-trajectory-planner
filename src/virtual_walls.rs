use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Serialize, Deserialize, Debug)]
pub struct VirtualWallEntry {
    pub name: String,
    pub polygon: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VirtualPathEntry {
    pub name: String,
    pub polygon: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct VirtualWallsJson {
    pub vwalls: Option<Vec<VirtualWallEntry>>,
    pub vpaths: Option<Vec<VirtualPathEntry>>,
}

pub struct VirtualWallsModule {
    walls: Vec<Vec<(f64, f64)>>,
    paths: Vec<Vec<(f64, f64)>>,
    walls_available: bool,
    paths_available: bool,
    default_cost: u8,
}

impl VirtualWallsModule {
    pub fn new(default_cost: u8) -> Self {
        Self {
            walls: Vec::new(),
            paths: Vec::new(),
            walls_available: false,
            paths_available: false,
            default_cost,
        }
    }

    pub fn parse_walls_json(&mut self, filename: &str) {
        if !Path::new(filename).exists() {
            println!(
                "File '{}' does not exist, skipping virtual walls collection",
                filename
            );
            return;
        }
        println!("Parsing virtual walls from {}...", filename);

        let file = match File::open(filename) {
            Ok(f) => f,
            Err(_) => {
                println!("Failed to open virtual walls file.");
                return;
            }
        };

        let reader = BufReader::new(file);
        let map_json: Result<VirtualWallsJson, _> = serde_json::from_reader(reader);

        let walls_data = match map_json {
            Ok(d) => d,
            Err(e) => {
                println!("Cannot parse virtual walls from the file: {}", e);
                return;
            }
        };

        if let Some(vwalls) = walls_data.vwalls {
            for wall in vwalls {
                if wall.polygon.len() % 2 != 0 {
                    println!(
                        "Cannot parse wall '{}': coordinates are not pairable",
                        wall.name
                    );
                    continue;
                }

                let mut points = Vec::new();
                for chunk in wall.polygon.chunks(2) {
                    if chunk.len() == 2 {
                        points.push((chunk[0], chunk[1]));
                    }
                }

                if !points.is_empty() {
                    self.walls.push(points);
                    self.walls_available = true;
                }
            }
        }

        if let Some(vpaths) = walls_data.vpaths {
            for path in vpaths {
                if path.polygon.len() % 2 != 0 {
                    println!(
                        "Cannot parse path '{}': coordinates are not pairable",
                        path.name
                    );
                    continue;
                }

                let mut points = Vec::new();
                for chunk in path.polygon.chunks(2) {
                    if chunk.len() == 2 {
                        points.push((chunk[0], chunk[1]));
                    }
                }

                if points.len() >= 2 {
                    self.paths.push(points);
                    self.paths_available = true;
                } else {
                    println!(
                        "Freepath selector '{}' must contain more than one point",
                        path.name
                    );
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn apply_to_grid(
        &self,
        grid: &mut [u8],
        width: usize,
        height: usize,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
        lethal_cost: u8,
        free_cost: u8,
    ) {
        // Helper to convert world coordinates to grid cell indices
        let world_to_map = |wx: f64, wy: f64| -> Option<(usize, usize)> {
            let mx = ((wx - origin_x) / resolution).round() as isize;
            let my = ((wy - origin_y) / resolution).round() as isize;
            if mx >= 0 && mx < width as isize && my >= 0 && my < height as isize {
                Some((mx as usize, my as usize))
            } else {
                None
            }
        };

        // Fill default cost if specified
        if self.default_cost > 0 {
            for cell in grid.iter_mut() {
                if *cell <= self.default_cost {
                    *cell = self.default_cost;
                }
            }
        }

        // Apply virtual walls (Lethal Obstacles via Raycasting / Point Inclusion)
        if self.walls_available {
            for polygon in &self.walls {
                let map_points: Vec<(usize, usize)> = polygon
                    .iter()
                    .filter_map(|&(wx, wy)| world_to_map(wx, wy))
                    .collect();

                if map_points.len() > 2 {
                    // Simple bounding box or scanline fill can be implemented here using pure Rust geometry
                    for &(mx, my) in &map_points {
                        let idx = my * width + mx;
                        if idx < grid.len() {
                            grid[idx] = lethal_cost;
                        }
                    }
                }
            }
        }

        // Apply virtual paths (Free space corridors)
        if self.paths_available {
            for path in &self.paths {
                let map_points: Vec<(usize, usize)> = path
                    .iter()
                    .filter_map(|&(wx, wy)| world_to_map(wx, wy))
                    .collect();

                for window in map_points.windows(2) {
                    let (x0, y0) = window[0];
                    let (x1, y1) = window[1];

                    // Bresenham's line algorithm in pure Rust for path clearing
                    let dx = (x1 as isize - x0 as isize).abs();
                    let dy = (y1 as isize - y0 as isize).abs();
                    let sx = if x0 < x1 { 1 } else { -1 };
                    let sy = if y0 < y1 { 1 } else { -1 };
                    let mut err = dx - dy;
                    let mut cx = x0 as isize;
                    let mut cy = y0 as isize;

                    loop {
                        if cx >= 0 && cx < width as isize && cy >= 0 && cy < height as isize {
                            let idx = (cy as usize) * width + (cx as usize);
                            if idx < grid.len() {
                                grid[idx] = free_cost;
                            }
                        }
                        if cx == x1 as isize && cy == y1 as isize {
                            break;
                        }
                        let e2 = 2 * err;
                        if e2 > -dy {
                            err -= dy;
                            cx += sx;
                        }
                        if e2 < dx {
                            err += dx;
                            cy += sy;
                        }
                    }
                }
            }
        }
    }
}
