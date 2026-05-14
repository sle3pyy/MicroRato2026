use std::collections::HashMap;

use super::heading::{Heading, normalize_angle};

const CELL_SIZE_UM: f64 = 2.0;
const ROBOT_DIAMETER_UM: f64 = 1.0;

// EKF noise terms tuned from simulator model:
// motor output has ~1.5% multiplicative noise, compass has 2 deg std dev and 4 tick latency.
const LINEAR_NOISE_BASE: f64 = 0.01;
const LINEAR_NOISE_GAIN: f64 = 0.08;
const ROTATION_NOISE_BASE: f64 = 0.02;
const ROTATION_NOISE_GAIN: f64 = 0.12;
const COMPASS_SIGMA_RAD: f64 = 2.0_f64.to_radians();
const MIN_COMPASS_SPEED_FOR_UPDATE: f64 = 0.02;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Cell {
    pub x: i32,
    pub y: i32,
}

impl Cell {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

pub struct PositionTracker {
    state: [f64; 3], // x_um, y_um, theta_rad
    covariance: [[f64; 3]; 3],
    last_cell: Cell,
    visit_counts: HashMap<Cell, u32>,
}

impl Default for PositionTracker {
    fn default() -> Self {
        let start = Cell::new(0, 0);
        let mut visit_counts = HashMap::new();
        visit_counts.insert(start, 1);

        Self {
            state: [0.0, 0.0, 0.0],
            covariance: [
                [0.05, 0.0, 0.0],
                [0.0, 0.05, 0.0],
                [0.0, 0.0, 0.08],
            ],
            last_cell: start,
            visit_counts,
        }
    }
}

impl PositionTracker {
    pub fn update(
        &mut self,
        compass_deg: Option<f64>,
        left_out_pow: f64,
        right_out_pow: f64,
        collided: bool,
    ) {
        self.predict(left_out_pow, right_out_pow, collided);

        if let Some(compass_deg) = compass_deg {
            let speed = left_out_pow.abs() + right_out_pow.abs();
            if speed <= MIN_COMPASS_SPEED_FOR_UPDATE || (left_out_pow - right_out_pow).abs() > 0.02 {
                self.correct_compass(compass_deg.to_radians());
            }
        }

        let current_cell = self.current_cell();
        if current_cell != self.last_cell {
            *self.visit_counts.entry(current_cell).or_insert(0) += 1;
            self.last_cell = current_cell;
        }
    }

    fn predict(&mut self, left_out_pow: f64, right_out_pow: f64, collided: bool) {
        let linear_um = if collided {
            0.0
        } else {
            (left_out_pow + right_out_pow) / 2.0
        };
        let rotation_rad = (right_out_pow - left_out_pow) / ROBOT_DIAMETER_UM;

        let theta = self.state[2];
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        self.state[0] += linear_um * cos_theta;
        self.state[1] += linear_um * sin_theta;
        self.state[2] = normalize_angle((theta + rotation_rad).to_degrees()).to_radians();

        let f = [
            [1.0, 0.0, -linear_um * sin_theta],
            [0.0, 1.0, linear_um * cos_theta],
            [0.0, 0.0, 1.0],
        ];

        let linear_sigma = LINEAR_NOISE_BASE + linear_um.abs() * LINEAR_NOISE_GAIN;
        let rotation_sigma = ROTATION_NOISE_BASE + rotation_rad.abs() * ROTATION_NOISE_GAIN;
        let q = [
            [linear_sigma * linear_sigma, 0.0, 0.0],
            [0.0, linear_sigma * linear_sigma, 0.0],
            [0.0, 0.0, rotation_sigma * rotation_sigma],
        ];

        self.covariance = mat3_add(mat3_mul(mat3_mul(f, self.covariance), mat3_transpose(f)), q);
    }

    fn correct_compass(&mut self, measured_theta_rad: f64) {
        let r = COMPASS_SIGMA_RAD * COMPASS_SIGMA_RAD;
        let innovation = normalize_rad(measured_theta_rad - self.state[2]);

        let p = &self.covariance;
        let s = p[2][2] + r;
        if s <= f64::EPSILON {
            return;
        }

        let k = [p[0][2] / s, p[1][2] / s, p[2][2] / s];

        self.state[0] += k[0] * innovation;
        self.state[1] += k[1] * innovation;
        self.state[2] = normalize_rad(self.state[2] + k[2] * innovation);

        let h = [0.0, 0.0, 1.0];
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let kh = outer_product(k, h);
        let i_minus_kh = mat3_sub(identity, kh);
        self.covariance = mat3_mul(i_minus_kh, self.covariance);
    }

    pub fn current_cell(&self) -> Cell {
        Cell::new(
            (self.state[0] / CELL_SIZE_UM).round() as i32,
            (self.state[1] / CELL_SIZE_UM).round() as i32,
        )
    }

    pub fn visit_count(&self, cell: Cell) -> u32 {
        self.visit_counts.get(&cell).copied().unwrap_or(0)
    }

    pub fn neighbor_cell(&self, heading: Heading) -> Cell {
        let current = self.current_cell();
        match heading {
            Heading::East => Cell::new(current.x + 1, current.y),
            Heading::North => Cell::new(current.x, current.y + 1),
            Heading::West => Cell::new(current.x - 1, current.y),
            Heading::South => Cell::new(current.x, current.y - 1),
        }
    }

    pub fn should_prefer_left(&self, heading: Heading) -> bool {
        let forward_visits = self.visit_count(self.neighbor_cell(heading));
        let left_visits = self.visit_count(self.neighbor_cell(heading.left()));

        left_visits < forward_visits
    }

    pub fn visited_cells(&self) -> usize {
        self.visit_counts.len()
    }
}

fn normalize_rad(angle_rad: f64) -> f64 {
    normalize_angle(angle_rad.to_degrees()).to_radians()
}

fn mat3_add(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] = a[row][col] + b[row][col];
        }
    }
    out
}

fn mat3_sub(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] = a[row][col] - b[row][col];
        }
    }
    out
}

fn mat3_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] =
                a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col];
        }
    }
    out
}

fn mat3_transpose(a: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]],
    ]
}

fn outer_product(a: [f64; 3], b: [f64; 3]) -> [[f64; 3]; 3] {
    [
        [a[0] * b[0], a[0] * b[1], a[0] * b[2]],
        [a[1] * b[0], a[1] * b[1], a[1] * b[2]],
        [a[2] * b[0], a[2] * b[1], a[2] * b[2]],
    ]
}
