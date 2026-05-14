#[derive(Debug, Clone, Copy)]
pub struct SensorSnapshot {
    pub front_ir: f64,
    pub left_ir: f64,
    pub right_ir: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum WallFollowAction {
    Forward(MotorCommand),
    ArcLeft(MotorCommand),
    ArcRight(MotorCommand),
    TurnLeft,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorCommand {
    pub left: f64,
    pub right: f64,
}

pub struct RightWallFollower {
    front_blocked_reading: f64,
    left_avoid_reading: f64,
    right_wall_visible_reading: f64,
    right_too_close_reading: f64,
    base_speed: f64,
    right_escape_left_speed: f64,
    right_escape_right_speed: f64,
    left_avoid_left_speed: f64,
    left_avoid_right_speed: f64,
    edge_recover_left_speed: f64,
    edge_recover_right_speed: f64,
    recovering_right_wall: bool,
}

impl Default for RightWallFollower {
    fn default() -> Self {
        Self {
            front_blocked_reading: 3.8,
            left_avoid_reading: 2.8,
            right_wall_visible_reading: 0.55,
            right_too_close_reading: 4.0,
            base_speed: 0.08,
            right_escape_left_speed: 0.05,
            right_escape_right_speed: 0.12,
            left_avoid_left_speed: 0.14,
            left_avoid_right_speed: -0.08,
            edge_recover_left_speed: 0.15,
            edge_recover_right_speed: -0.09,
            recovering_right_wall: false,
        }
    }
}

impl RightWallFollower {
    pub fn next_action(&mut self, sensors: SensorSnapshot) -> WallFollowAction {
        let front_blocked = sensors.front_ir >= self.front_blocked_reading;
        let left_too_close = sensors.left_ir >= self.left_avoid_reading;
        let right_wall_visible = sensors.right_ir >= self.right_wall_visible_reading;
        let right_too_close = sensors.right_ir >= self.right_too_close_reading;

        if front_blocked && right_wall_visible {
            self.recovering_right_wall = false;
            return WallFollowAction::TurnLeft;
        }

        if right_too_close && !front_blocked {
            self.recovering_right_wall = false;
            return WallFollowAction::ArcLeft(MotorCommand {
                left: self.right_escape_left_speed,
                right: self.right_escape_right_speed,
            });
        }

        if left_too_close && !front_blocked {
            return WallFollowAction::ArcRight(MotorCommand {
                left: self.left_avoid_left_speed,
                right: self.left_avoid_right_speed,
            });
        }

        if !right_wall_visible {
            self.recovering_right_wall = true;
        }

        if self.recovering_right_wall {
            if right_wall_visible {
                self.recovering_right_wall = false;
            } else {
                return WallFollowAction::ArcRight(MotorCommand {
                    left: self.edge_recover_left_speed,
                    right: self.edge_recover_right_speed,
                });
            }
        }

        WallFollowAction::Forward(MotorCommand {
            left: self.base_speed,
            right: self.base_speed,
        })
    }
}
