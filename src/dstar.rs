use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::dstar_error::DStarError;
use crate::state_map::{NeighborMode, StateMap, StateTag};

pub struct DStar {
    neighbor_mode: NeighborMode,
    map: StateMap,
    origin: Option<(i64, i64)>,
    destination: Option<(i64, i64)>,
    open_list: BinaryHeap<Reverse<(i64, i64, i64)>>,
    cutoff_distance: i32,
    r_field: i32,
    repulsion_gain: f64,
}

impl DStar {
    pub fn new(map: StateMap) -> Self {
        Self {
            neighbor_mode: NeighborMode::Eight,
            map,
            origin: None,
            destination: None,
            open_list: BinaryHeap::new(),
            cutoff_distance: 10,
            r_field: 5,
            repulsion_gain: 20.0,
        }
    }

    pub fn set_neighbor_mode(&mut self, mode: NeighborMode) {
        self.neighbor_mode = mode;
    }

    pub fn set_cutoff_distance(&mut self, val: i32) {
        self.cutoff_distance = val;
    }

    pub fn set_repulsion_gain(&mut self, val: f64) {
        self.repulsion_gain = if val > 1.0 { val } else { 1.0 };
    }

    pub fn set_r_field(&mut self, val: i32) {
        self.r_field = if val > 2 { val } else { 2 };
    }

    pub fn grid(&self) -> &StateMap {
        &self.map
    }

    fn get_kmin(&self) -> i64 {
        match self.open_list.peek() {
            Some(Reverse((k, _, _))) => *k,
            None => -1,
        }
    }

    fn min_state(&self) -> Option<(i64, i64)> {
        self.open_list.peek().map(|Reverse((_, x, y))| (*x, *y))
    }

    fn insert_state(&mut self, coord: (i64, i64)) {
        if let Some(p) = self.map.point(coord.0, coord.1) {
            p.tag = StateTag::Open;
            let k = p.k();
            self.open_list.push(Reverse((k, coord.0, coord.1)));
        }
    }

    fn pop_state(&mut self) -> Option<(i64, i64)> {
        self.open_list.pop().map(|Reverse((_, x, y))| (x, y))
    }

    pub fn init_targets(
        &mut self,
        origin_x: i64,
        origin_y: i64,
        dest_x: i64,
        dest_y: i64,
        weighted: bool,
    ) {
        if origin_x < self.map.get_width()
            && origin_y < self.map.get_height()
            && dest_x < self.map.get_width()
            && dest_y < self.map.get_height()
        {
            if self.destination.is_some() {
                self.open_list.clear();
                self.map.reset();
            }

            self.destination = Some((dest_x, dest_y));
            if let Some(dest) = self.map.point(dest_x, dest_y) {
                dest.cost_actual = 0;
                dest.cost_previous = 0;
                dest.backpointer = None;
                dest.tag = StateTag::Open;
            }

            if let Some(dest) = self.map.point(dest_x, dest_y) {
                let k = dest.k();
                self.open_list.push(Reverse((k, dest_x, dest_y)));
            }

            if let Some((ox, oy)) = self.origin
                && let Some(orig) = self.map.point(ox, oy)
            {
                orig.weight = orig.weight_previous;
            }

            self.origin = Some((origin_x, origin_y));
            let origin_coords = (origin_x as f64, origin_y as f64);
            let dest_coords = (dest_x as f64, dest_y as f64);
            let dist = ((dest_coords.0 - origin_coords.0).powi(2)
                + (dest_coords.1 - origin_coords.1).powi(2))
            .sqrt();

            let width = self.map.get_width();
            let height = self.map.get_height();

            for i in 0..width {
                for j in 0..height {
                    if (i != origin_x || j != origin_y) && (i != dest_x || j != dest_y) {
                        let dist_dest = ((i as f64 - dest_coords.0).powi(2)
                            + (j as f64 - dest_coords.1).powi(2))
                        .sqrt();
                        let dot_origin = ((i - dest_x) * (origin_x - dest_x)) as f64
                            + ((j - dest_y) * (origin_y - dest_y)) as f64;
                        let projection = dot_origin / dist;
                        let inner = dist_dest.powi(2) - projection.powi(2);
                        let cost_addition = if inner >= 0.0 {
                            dist_dest + inner.sqrt()
                        } else {
                            dist_dest
                        };
                        let l_cost = cost_addition as i64;
                        if weighted && let Some(p) = self.map.point(i, j) {
                            p.weight += if l_cost < 0 { 10 } else { l_cost };
                        }
                    }
                }
            }
        }
    }

    fn iterate_state(&mut self) -> i64 {
        let x_coord = match self.min_state() {
            Some(c) => c,
            None => return -1,
        };

        if Some(x_coord) == self.origin {
            return -1;
        }

        let k_old = self.get_kmin();
        let _ = self.pop_state();

        let neighbors = self.map.neighbors(x_coord.0, x_coord.1, self.neighbor_mode);
        let x_cost_actual = self
            .map
            .point_ref(x_coord.0, x_coord.1)
            .unwrap()
            .cost_actual;

        for y_coord in neighbors {
            let y_point = match self.map.point_ref(y_coord.0, y_coord.1) {
                Some(p) => p,
                None => continue,
            };
            let y_tag = y_point.tag;
            if y_tag == StateTag::Obstacle {
                continue;
            }

            let y_cost_actual = y_point.cost_actual;
            let y_weight = y_point.weight;

            if y_tag == StateTag::Closed
                && y_cost_actual < k_old
                && x_cost_actual > y_cost_actual + y_weight
            {
                let p = self.map.point(x_coord.0, x_coord.1).unwrap();
                p.backpointer = Some(y_coord);
                p.cost_actual = y_cost_actual + y_weight;
            }
        }

        let neighbors = self.map.neighbors(x_coord.0, x_coord.1, self.neighbor_mode);
        for y_coord in neighbors {
            let y_point = self.map.point_ref(y_coord.0, y_coord.1).unwrap();
            let y_tag = y_point.tag;
            if y_tag == StateTag::Obstacle {
                continue;
            }

            let y_weight = y_point.weight;

            if y_tag == StateTag::New {
                let x_act = self
                    .map
                    .point_ref(x_coord.0, x_coord.1)
                    .unwrap()
                    .cost_actual;
                let p = match self.map.point(y_coord.0, y_coord.1) {
                    Some(p) => p,
                    None => continue,
                };
                p.backpointer = Some(x_coord);
                p.cost_actual = y_weight + x_act;
                p.cost_previous = p.cost_actual;
                let yc = y_coord;
                self.insert_state(yc);
            } else {
                let y_bp = y_point.backpointer;
                let y_act = y_point.cost_actual;
                let x_act = self
                    .map
                    .point_ref(x_coord.0, x_coord.1)
                    .unwrap()
                    .cost_actual;
                let x_w = self.map.point_ref(x_coord.0, x_coord.1).unwrap().weight;

                if y_bp == Some(x_coord) && y_act != x_act + x_w {
                    let p = match self.map.point(y_coord.0, y_coord.1) {
                        Some(p) => p,
                        None => continue,
                    };
                    if p.tag == StateTag::Open {
                        if p.cost_actual < p.cost_previous {
                            p.cost_previous = p.cost_actual;
                        }
                        p.cost_actual = x_act + x_w;
                    } else {
                        p.cost_actual = x_act + x_w;
                        p.cost_previous = p.cost_actual;
                    }
                    self.insert_state(y_coord);
                } else if y_bp != Some(x_coord) && y_act > x_act + x_w {
                    let x_prev = self
                        .map
                        .point_ref(x_coord.0, x_coord.1)
                        .unwrap()
                        .cost_previous;
                    let x_act_val = self
                        .map
                        .point_ref(x_coord.0, x_coord.1)
                        .unwrap()
                        .cost_actual;
                    if x_prev >= x_act_val {
                        let p = match self.map.point(y_coord.0, y_coord.1) {
                            Some(p) => p,
                            None => continue,
                        };
                        p.backpointer = Some(x_coord);
                        p.cost_actual = x_act + x_w;
                        if p.tag == StateTag::Closed {
                            p.cost_previous = p.cost_actual;
                        }
                        self.insert_state(y_coord);
                    } else {
                        let p = self.map.point(x_coord.0, x_coord.1).unwrap();
                        p.cost_previous = p.cost_actual;
                        let xc = x_coord;
                        self.insert_state(xc);
                    }
                } else if y_bp != Some(x_coord)
                    && x_act > y_act + y_weight
                    && y_tag == StateTag::Closed
                    && y_act > k_old
                {
                    let p = match self.map.point(y_coord.0, y_coord.1) {
                        Some(p) => p,
                        None => continue,
                    };
                    p.cost_previous = p.cost_actual;
                    self.insert_state(y_coord);
                }
            }
        }

        self.get_kmin()
    }

    pub fn generate_trajectory(&mut self) -> Result<Vec<(i64, i64)>, DStarError> {
        if self.origin.is_none() || self.destination.is_none() {
            return Ok(Vec::new());
        }
        while self.iterate_state() != -1 {}
        self.trace_path()
    }

    /// Draft path: follow backpointers from origin to destination.
    fn build_draft_path(&self) -> Vec<(i64, i64)> {
        let mut draft_path = Vec::new();
        let mut current = self.origin;

        while let Some((x, y)) = current {
            let p = match self.map.point_ref(x, y) {
                Some(p) => p,
                None => break,
            };
            draft_path.push((x, y));
            current = p.backpointer;
        }

        draft_path
    }

    /// Reduced path: ray-tracing with cutoff_distance, ported from C++.
    fn reduce_path(&self, draft: &[(i64, i64)]) -> Result<Vec<(i64, i64)>, DStarError> {
        if draft.len() <= 1 {
            return Err(DStarError(
                "The draft path list is empty, check target position".into(),
            ));
        }

        let origin = draft[0];
        let destination = *draft.last().unwrap();

        let mut reduced = Vec::new();
        reduced.push(origin);

        let mut current_viewpoint = origin;
        let mut prev_visible = draft.get(1).copied();
        let mut last_visible = draft.get(2).copied();

        if prev_visible.is_none() || last_visible.is_none() {
            return Err(DStarError(
                "Nothing to reduce, check target position".into(),
            ));
        }

        while last_visible != Some(destination) {
            let (cx, cy) = current_viewpoint;
            let (px, py) = prev_visible.unwrap();
            let (lx, ly) = last_visible.unwrap();

            let (current_x, current_y) = (cx as f64, cy as f64);
            let (proposed_x, proposed_y) = (lx as f64, ly as f64);
            let (prev_x, prev_y) = (px as f64, py as f64);

            let hypo = ((current_x - proposed_x).powi(2) + (current_y - proposed_y).powi(2)).sqrt();
            let cos_theta = (proposed_x - current_x) / hypo;
            let sin_theta = (proposed_y - current_y) / hypo;

            let mut ray_x = current_x;
            let mut ray_y = current_y;
            let mut ray_traced = true;

            while ((ray_x - proposed_x).powi(2) + (ray_y - proposed_y).powi(2)).sqrt() >= 1.0 {
                ray_x += cos_theta;
                ray_y += sin_theta;

                let dx = ray_x - prev_x;
                let dy = ray_y - prev_y;
                let dist_prev = (dx * dx + dy * dy).sqrt();

                let xi = ray_x.round() as i64;
                let yj = ray_y.round() as i64;

                let cutoff = self.cutoff_distance as f64;

                if dist_prev > cutoff {
                    ray_traced = false;
                    break;
                }

                if let Some(p) = self.map.point_ref(xi, yj) {
                    if p.tag == StateTag::Obstacle {
                        ray_traced = false;
                        break;
                    }
                } else {
                    ray_traced = false;
                    break;
                }
            }

            if !ray_traced {
                reduced.push(prev_visible.unwrap());
                current_viewpoint = prev_visible.unwrap();
            }

            prev_visible = last_visible;
            let idx = draft
                .iter()
                .position(|&(x, y)| x == last_visible.unwrap().0 && y == last_visible.unwrap().1)
                .unwrap();
            last_visible = draft.get(idx + 1).copied();
            if last_visible.is_none() {
                break;
            }
        }

        reduced.push(destination);
        Ok(reduced)
    }

    /// Optimized path: potential-field smoothing, ported from C++.
    fn optimize_path(&mut self, reduced: &[(i64, i64)]) -> Vec<(i64, i64)> {
        let mut optimized = Vec::new();
        if reduced.is_empty() {
            return optimized;
        }

        optimized.push(reduced[0]); // origin

        for &(cx, cy) in reduced.iter().skip(1).take(reduced.len().saturating_sub(2)) {
            let radius = self.r_field as i64;

            let mut free = Vec::new();
            let mut occupied = Vec::new();

            for (nx, ny) in self.map.neighborhood(cx, cy, radius) {
                if let Some(p) = self.map.point_ref(nx, ny) {
                    if p.tag == StateTag::Obstacle {
                        occupied.push((nx, ny));
                    } else {
                        free.push((nx, ny));
                    }
                }
            }

            if free.is_empty() || occupied.is_empty() {
                optimized.push((cx, cy));
                continue;
            }

            let mut potentials = Vec::with_capacity(free.len());

            for &(fx, fy) in &free {
                let (px, py) = self.map.point_ref(fx, fy).unwrap().float_coords();
                let mut repulsion = 0.0;

                for &(ox, oy) in &occupied {
                    let (oxf, oyf) = self.map.point_ref(ox, oy).unwrap().float_coords();
                    let dx = oxf - px;
                    let dy = oyf - py;
                    let dist2 = dx * dx + dy * dy;
                    if dist2 > 0.0 {
                        repulsion += self.repulsion_gain / dist2;
                    }
                }

                potentials.push(repulsion);
            }

            if let Some((min_idx, min_val)) = potentials
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            {
                let (nx, ny) = free[min_idx];
                if let Some(p) = self.map.point(nx, ny) {
                    p.potential = *min_val;
                }
                optimized.push((nx, ny));
            } else {
                optimized.push((cx, cy));
            }
        }

        if let Some(&last) = reduced.last() {
            optimized.push(last); // destination
        }

        optimized
    }

    fn trace_path(&mut self) -> Result<Vec<(i64, i64)>, DStarError> {
        let draft_path = self.build_draft_path();

        if draft_path.len() <= 1 {
            return Err(DStarError(
                "The draft path list is empty, check target position".into(),
            ));
        }

        let reduced = self.reduce_path(&draft_path)?;
        let optimized = self.optimize_path(&reduced);

        Ok(optimized)
    }
}
