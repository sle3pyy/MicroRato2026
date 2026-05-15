use super::config::{CELL_SIZE, COMPASS_NOISE_STD_DEG, MOTOR_NOISE_STD, ROBOT_DIAM};
use super::dir::Dir;

#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta_deg: f64,
}

pub struct MotorModel {
    pub out_l: f64,
    pub out_r: f64,
}

impl MotorModel {
    pub fn new() -> Self {
        Self { out_l: 0.0, out_r: 0.0 }
    }
    // Simulator filter: out_t = (out_{t-1} + in_t) / 2 (PDF §7).
    pub fn apply(&mut self, l_in: f64, r_in: f64) {
        self.out_l = 0.5 * (self.out_l + l_in);
        self.out_r = 0.5 * (self.out_r + r_in);
    }
}

pub fn wrap_pi(mut r: f64) -> f64 {
    while r > std::f64::consts::PI {
        r -= 2.0 * std::f64::consts::PI;
    }
    while r <= -std::f64::consts::PI {
        r += 2.0 * std::f64::consts::PI;
    }
    r
}

// EKF over (x, y, theta_rad). Predict from motor model, scalar compass update.
pub struct Ekf {
    pub x: f64,
    pub y: f64,
    pub theta_rad: f64,
    pub p: [[f64; 3]; 3],
}

impl Ekf {
    pub fn new() -> Self {
        Self { x: 0.0, y: 0.0, theta_rad: 0.0, p: [[0.0; 3]; 3] }
    }

    pub fn init(&mut self, theta0_rad: f64) {
        self.x = 0.0;
        self.y = 0.0;
        self.theta_rad = theta0_rad;
        self.p = [[0.0; 3]; 3];
        self.p[0][0] = 0.01;
        self.p[1][1] = 0.01;
        self.p[2][2] = (COMPASS_NOISE_STD_DEG.to_radians()).powi(2);
    }

    pub fn predict(&mut self, out_l: f64, out_r: f64) {
        let linr = (out_l + out_r) * 0.5;
        let rotr = (out_r - out_l) / ROBOT_DIAM;
        let c = self.theta_rad.cos();
        let s = self.theta_rad.sin();
        // Linear then rotational (PDF §7).
        self.x += linr * c;
        self.y += linr * s;
        self.theta_rad = wrap_pi(self.theta_rad + rotr);
        let f02 = -linr * s;
        let f12 = linr * c;
        let fp = [
            [
                self.p[0][0] + f02 * self.p[2][0],
                self.p[0][1] + f02 * self.p[2][1],
                self.p[0][2] + f02 * self.p[2][2],
            ],
            [
                self.p[1][0] + f12 * self.p[2][0],
                self.p[1][1] + f12 * self.p[2][1],
                self.p[1][2] + f12 * self.p[2][2],
            ],
            [self.p[2][0], self.p[2][1], self.p[2][2]],
        ];
        let mut np = [[0.0f64; 3]; 3];
        for i in 0..3 {
            np[i][0] = fp[i][0];
            np[i][1] = fp[i][1];
            np[i][2] = fp[i][0] * f02 + fp[i][1] * f12 + fp[i][2];
        }
        let q_lin = (MOTOR_NOISE_STD * linr.abs()).powi(2) + 1e-6;
        let q_rot = (MOTOR_NOISE_STD * rotr.abs()).powi(2) + 1e-7;
        np[0][0] += q_lin * c * c;
        np[1][1] += q_lin * s * s;
        np[0][1] += q_lin * c * s;
        np[1][0] += q_lin * c * s;
        np[2][2] += q_rot;
        self.p = np;
    }

    // Scalar heading update; decouple x,y to avoid Jacobian-driven drift.
    pub fn update_compass(&mut self, innov: f64) {
        let r = (COMPASS_NOISE_STD_DEG.to_radians()).powi(2);
        let s = self.p[2][2] + r;
        let k = self.p[2][2] / s;
        self.theta_rad = wrap_pi(self.theta_rad + k * innov);
        self.p[2][2] *= 1.0 - k;
        self.p[0][2] = 0.0;
        self.p[1][2] = 0.0;
        self.p[2][0] = 0.0;
        self.p[2][1] = 0.0;
    }

    pub fn pose(&self) -> Pose {
        Pose { x: self.x, y: self.y, theta_deg: self.theta_rad.to_degrees() }
    }
}

// Derive grid cell from continuous pose: (row, col) = (round(y/2), round(x/2)).
pub fn pose_to_cell(p: &Pose) -> (i32, i32) {
    ((p.y / CELL_SIZE).round() as i32, (p.x / CELL_SIZE).round() as i32)
}

// Nearest cardinal direction from theta (degrees).
pub fn pose_to_heading(theta_deg: f64) -> Dir {
    let mut t = theta_deg % 360.0;
    if t < 0.0 {
        t += 360.0;
    }
    // East=0, North=90, West=180, South=270 (compass convention used).
    if t < 45.0 || t >= 315.0 {
        Dir::East
    } else if t < 135.0 {
        Dir::North
    } else if t < 225.0 {
        Dir::West
    } else {
        Dir::South
    }
}

// Project pose onto travel-axis cell-center to remove longitudinal drift.
// Leaves lateral coordinate intact (corridor centering signal lives there).
pub fn anchor_pose_to_cell_along(ekf: &mut Ekf, cell: (i32, i32), dir: Dir) {
    let (cx, cy) = (cell.1 as f64 * CELL_SIZE, cell.0 as f64 * CELL_SIZE);
    match dir {
        Dir::East | Dir::West => ekf.x = cx,
        Dir::North | Dir::South => ekf.y = cy,
    }
}
