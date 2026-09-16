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

    /// Helper to parse raw polygon f64 chunks into coordinate pairs
    fn parse_polygon_points(polygon: &[f64], name: &str, is_path: bool) -> Option<Vec<(f64, f64)>> {
        if !polygon.len().is_multiple_of(2) {
            println!(
                "Cannot parse {}: coordinates are not pairable for '{}'",
                if is_path { "path" } else { "wall" },
                name
            );
            return None;
        }

        let points: Vec<(f64, f64)> = polygon
            .chunks(2)
            .filter(|chunk| chunk.len() == 2)
            .map(|chunk| (chunk[0], chunk[1]))
            .collect();

        if is_path && points.len() < 2 {
            println!(
                "Freepath selector '{}' must contain more than one point",
                name
            );
            return None;
        }

        if !is_path && points.is_empty() {
            return None;
        }

        Some(points)
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
                if let Some(points) = Self::parse_polygon_points(&wall.polygon, &wall.name, false) {
                    self.walls.push(points);
                    self.walls_available = true;
                }
            }
        }

        if let Some(vpaths) = walls_data.vpaths {
            for path in vpaths {
                if let Some(points) = Self::parse_polygon_points(&path.polygon, &path.name, true) {
                    self.paths.push(points);
                    self.paths_available = true;
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
        let world_to_map = |wx: f64, wy: f64| -> Option<(usize, usize)> {
            let mx = ((wx - origin_x) / resolution).round() as isize;
            let my = ((wy - origin_y) / resolution).round() as isize;
            if mx >= 0 && mx < width as isize && my >= 0 && my < height as isize {
                Some((mx as usize, my as usize))
            } else {
                None
            }
        };

        // Fill default cost if specified (mutably borrows grid)
        if self.default_cost > 0 {
            for cell in grid.iter_mut() {
                if *cell <= self.default_cost {
                    *cell = self.default_cost;
                }
            }
        }

        // Helper to rasterize a line using Bresenham's algorithm
        // (Defined AFTER the iter_mut loop to satisfy the borrow checker)
        let mut draw_line = |x0: usize, y0: usize, x1: usize, y1: usize, cost: u8| {
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
                        grid[idx] = cost;
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
        };

        // Apply virtual walls (Lethal Obstacles via Line Segments)
        if self.walls_available {
            for polygon in &self.walls {
                let map_points: Vec<(usize, usize)> = polygon
                    .iter()
                    .filter_map(|&(wx, wy)| world_to_map(wx, wy))
                    .collect();

                // Draw lines between consecutive points
                for window in map_points.windows(2) {
                    draw_line(
                        window[0].0,
                        window[0].1,
                        window[1].0,
                        window[1].1,
                        lethal_cost,
                    );
                }
                // Close the polygon loop if it has 3 or more points
                if map_points.len() > 2 {
                    let first = *map_points.first().unwrap();
                    let last = *map_points.last().unwrap();
                    draw_line(last.0, last.1, first.0, first.1, lethal_cost);
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
                    draw_line(
                        window[0].0,
                        window[0].1,
                        window[1].0,
                        window[1].1,
                        free_cost,
                    );
                }
            }
        }
    }
}
