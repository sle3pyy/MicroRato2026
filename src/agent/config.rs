// Tuning constants. IR returns ~1/distance. Cell side=2, robot diam=1.

pub const FRONT_STOP: f64 = 3.5;
pub const DRIVE_POWER: f64 = 0.10;
pub const TURN_POWER: f64 = 0.07;
pub const BACKUP_POWER: f64 = -0.05;
pub const BACKUP_CYCLES: u32 = 6;
pub const DRIVE_CYCLES: u32 = 26;
pub const SETTLE_CYCLES: u32 = 4;
pub const LATERAL_KP: f64 = 0.05;
pub const HEADING_KP: f64 = 0.004;

pub const CELL_SIZE: f64 = 2.0;
pub const ROBOT_DIAM: f64 = 1.0;
pub const WALL_HIGH: f64 = 2.2;
pub const WALL_LOW: f64 = 1.5;
pub const WALL_CONFIRM_K: i32 = 2;
pub const TURN_TOL_DEG: f64 = 4.0;
pub const TURN_EXIT_K: u32 = 3;
pub const TURN_MAX_CYCLES: u32 = 36;
pub const DRIVE_DIST_TARGET: f64 = 2.0;
pub const DRIVE_DIST_MARGIN: f64 = 0.15;
pub const MOTOR_NOISE_STD: f64 = 0.015;
pub const COMPASS_NOISE_STD_DEG: f64 = 2.0;

