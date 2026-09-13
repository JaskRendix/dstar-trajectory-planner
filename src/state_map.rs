#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StateTag {
    New,
    Closed,
    Open,
    Raise,
    Lower,
    Obstacle,
}

#[derive(Clone, Copy, Debug)]
pub enum NeighborMode {
    Four,
    Eight,
}

#[derive(Clone, Debug)]
pub struct StatePoint {
    pub x: i64,
    pub y: i64,
    pub tag: StateTag,
    pub weight: i64,
    pub weight_previous: i64,
    pub cost_previous: i64,
    pub cost_actual: i64,
    pub potential: f64,
    pub backpointer: Option<(i64, i64)>,
}

impl StatePoint {
    pub fn new(x: i64, y: i64) -> Self {
        Self {
            x,
            y,
            tag: StateTag::New,
            weight: 0,
            weight_previous: 0,
            cost_previous: -1,
            cost_actual: -1,
            potential: 0.0,
            backpointer: None,
        }
    }

    pub fn k(&self) -> i64 {
        if self.tag == StateTag::Open {
            std::cmp::min(self.cost_actual, self.cost_previous)
        } else {
            -1
        }
    }

    pub fn float_coords(&self) -> (f64, f64) {
        (self.x as f64, self.y as f64)
    }
}

pub struct StateMap {
    width: i64,
    height: i64,
    grid: Vec<StatePoint>,
}

impl StateMap {
    /// Returns all cells within a circular neighborhood of radius `radius`.
    pub fn neighborhood(&self, x: i64, y: i64, radius: i64) -> Vec<(i64, i64)> {
        let mut result = Vec::new();
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                let nx = x + dx;
                let ny = y + dy;
                if nx >= 0
                    && nx < self.width
                    && ny >= 0
                    && ny < self.height
                    && dx * dx + dy * dy <= radius * radius
                {
                    result.push((nx, ny));
                }
            }
        }
        result
    }

    pub fn new(width: i64, height: i64) -> Self {
        let mut grid = Vec::with_capacity((width * height) as usize);

        for y in 0..height {
            for x in 0..width {
                grid.push(StatePoint::new(x, y));
            }
        }

        Self {
            width,
            height,
            grid,
        }
    }

    pub fn get_width(&self) -> i64 {
        self.width
    }

    pub fn get_height(&self) -> i64 {
        self.height
    }

    #[inline]
    fn idx(&self, x: i64, y: i64) -> Option<usize> {
        if x >= 0 && x < self.width && y >= 0 && y < self.height {
            Some((y * self.width + x) as usize)
        } else {
            None
        }
    }

    pub fn point(&mut self, x: i64, y: i64) -> Option<&mut StatePoint> {
        self.idx(x, y).map(|i| &mut self.grid[i])
    }

    pub fn point_ref(&self, x: i64, y: i64) -> Option<&StatePoint> {
        self.idx(x, y).map(|i| &self.grid[i])
    }

    pub fn neighbors(&self, x: i64, y: i64, mode: NeighborMode) -> Vec<(i64, i64)> {
        let mut result = Vec::with_capacity(8);

        // 4‑connected
        const FOUR: &[(i64, i64)] = &[(1, 0), (-1, 0), (0, 1), (0, -1)];

        // 8‑connected (includes diagonals)
        const EIGHT: &[(i64, i64)] = &[
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ];

        let deltas = match mode {
            NeighborMode::Four => FOUR,
            NeighborMode::Eight => EIGHT,
        };

        for (dx, dy) in deltas {
            let nx = x + dx;
            let ny = y + dy;

            if nx >= 0 && nx < self.width && ny >= 0 && ny < self.height {
                result.push((nx, ny));
            }
        }

        result
    }

    pub fn reset(&mut self) {
        for cell in &mut self.grid {
            cell.backpointer = None;
            if cell.tag != StateTag::Obstacle {
                cell.tag = StateTag::New;
            }
            cell.potential = 0.0;
            cell.cost_actual = -1;
            cell.cost_previous = -1;
            cell.weight = cell.weight_previous;
        }
    }
}
