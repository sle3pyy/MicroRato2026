mod config;
mod dir;
mod map;
mod motion;
mod planner;
mod pose;
mod sensors;
mod trace;

use crate::Config;
use crate::cif::{CiberIf, CiberMouse};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

use config::*;
use dir::{Dir, compass_to_dir, turn_left, turn_right};
use map::DynMap;
use motion::Motion;
use pose::{Ekf, MotorModel, anchor_pose_to_cell_along, pose_to_cell, pose_to_heading, wrap_pi};
use sensors::SensorCache;
use trace::Trace;

#[derive(Debug, PartialEq)]
enum AgentState {
    WaitStart,
    Explore,
    FoundTarget,
    ReturnToStart,
    Done,
}

struct TurnLog {
    cycle_started: u32,
    target_dir: Dir,
    start_compass: f64,
}

pub struct Agent {
    mouse: CiberMouse,
    config: Config,

    // Sensors
    sense: SensorCache,
    gps_origin: Option<(f64, f64)>,

    // Last motor command (trace + bookkeeping)
    last_l_pow: f64,
    last_r_pow: f64,

    // Map + grid pose (derived from EKF)
    map: DynMap,
    row: i32,
    col: i32,
    heading: Dir,
    target: Option<(i32, i32)>,

    // Motion state
    motion: Motion,
    pending_dir: Option<Dir>,

    // Pose
    ekf: Ekf,
    motor: MotorModel,

    // Wall-confirmation streaks
    wall_streak: HashMap<((i32, i32), Dir), i32>,
    drive_start_xy: Option<(f64, f64)>,
    turn_in_tol_streak: u32,

    // FSM
    state: AgentState,
    cycle_count: u32,
    last_cycle_time: f64,
    settle_cycles: u32,
    no_progress_cycles: u32,

    // Telemetry
    trace: Trace,
    cur_turn: Option<TurnLog>,
    last_decision: &'static str,
    last_reloc_delta: i32,
}

impl Agent {
    pub fn new(config: Config) -> Self {
        let trace = Trace::new(config.debug_gps);
        Self {
            mouse: CiberMouse::new().expect("Failed to initialize CiberMouse"),
            config,
            sense: SensorCache::new(),
            gps_origin: None,
            last_l_pow: 0.0,
            last_r_pow: 0.0,
            map: DynMap::new(),
            row: 0,
            col: 0,
            heading: Dir::East,
            target: None,
            motion: Motion::Idle,
            pending_dir: None,
            ekf: Ekf::new(),
            motor: MotorModel::new(),
            wall_streak: HashMap::new(),
            drive_start_xy: None,
            turn_in_tol_streak: 0,
            state: AgentState::WaitStart,
            cycle_count: 0,
            last_cycle_time: now(),
            settle_cycles: 0,
            no_progress_cycles: 0,
            trace,
            cur_turn: None,
            last_decision: "",
            last_reloc_delta: 0,
        }
    }

    pub fn connect(&mut self) {
        println!(
            "Connecting to {} as {} at pos {}...",
            self.config.host, self.config.name, self.config.pos
        );

        let angles = [0.0f64, 90.0, -90.0, 180.0];
        if !self.mouse.init_robot_2(
            &self.config.name,
            self.config.pos,
            &angles,
            &self.config.host,
        ) {
            eprintln!("Failed to initialize robot!");
            return;
        }

        println!("Connected. Cycle time: {}ms", self.mouse.get_cycle_time());

        loop {
            // 4-sensor-per-cycle budget (PDF §3.1, §8.2).
            // IR0/1/2 every cycle; 4th slot rotates Compass/Ground/Ground/IR3.
            match self.cycle_count % 4 {
                0 => self
                    .mouse
                    .request_sensors(&["IRSensor0", "IRSensor1", "IRSensor2", "Compass"]),
                3 => self.mouse.request_sensors(&[
                    "IRSensor0", "IRSensor1", "IRSensor2", "IRSensor3",
                ]),
                _ => self
                    .mouse
                    .request_sensors(&["IRSensor0", "IRSensor1", "IRSensor2", "Ground"]),
            }
            self.mouse.read_sensors();

            let t = now();
            if t - self.last_cycle_time > 5.0 {
                eprintln!("[FATAL] Simulator stopped sending cycles (timeout > 5s)");
                break;
            }
            self.last_cycle_time = t;

            self.tick();
            self.cycle_count += 1;
            if self.state == AgentState::Done {
                println!("Agent Done. Disconnecting.");
                break;
            }
        }
    }

    fn tick(&mut self) {
        self.update_sensors();

        if self.state != AgentState::WaitStart {
            self.ekf.predict(self.motor.out_l, self.motor.out_r);
            if self.sense.compass_fresh {
                let innov = wrap_pi(self.sense.compass.to_radians() - self.ekf.theta_rad);
                self.ekf.update_compass(innov);
                self.sense.compass_fresh = false;
            }
        } else {
            self.sense.compass_fresh = false;
        }

        match self.state {
            AgentState::WaitStart => self.tick_wait_start(),
            AgentState::Explore => self.tick_explore(),
            AgentState::FoundTarget => self.tick_found_target(),
            AgentState::ReturnToStart => self.tick_return(),
            AgentState::Done => self.cmd_motors(0.0, 0.0),
        }

        self.write_trace_row();
    }

    // ── Sensors ───────────────────────────────────────────────────────────────
    fn update_sensors(&mut self) {
        for i in 0..4 {
            if self.mouse.is_obstacle_ready(i) {
                let v = self.mouse.get_obstacle_sensor(i);
                self.sense.ir[i as usize] = v;
                self.sense.filters[i as usize].push(v);
            }
        }
        if self.mouse.is_compass_ready() {
            let new_c = self.mouse.get_compass_sensor();
            if new_c != self.sense.compass || !self.sense.compass_ready {
                self.sense.compass_fresh = true;
            }
            self.sense.compass = new_c;
            self.sense.compass_ready = true;
        }
        if self.mouse.is_ground_ready() {
            self.sense.ground = self.mouse.get_ground_sensor();
        }
        if self.mouse.is_bumper_ready() {
            self.sense.bumper = self.mouse.get_bumper_sensor();
        }
        if self.mouse.is_obstacle_ready(0)
            && self.mouse.is_obstacle_ready(1)
            && self.mouse.is_obstacle_ready(2)
        {
            self.sense.ir_ready = true;
        }
        if self.config.debug_gps && self.mouse.is_gps_ready() {
            self.sense.gps_x = self.mouse.get_x();
            self.sense.gps_y = self.mouse.get_y();
            self.sense.gps_ready = true;
        }
    }

    // Streak-confirmed wall sensing on front/left/right of current heading.
    fn sense_walls(&mut self) {
        let pos = (self.row, self.col);
        let dirs = [self.heading, turn_left(self.heading), turn_right(self.heading)];
        for (i, &dir) in dirs.iter().enumerate() {
            if !self.sense.filters[i].is_fresh() {
                continue;
            }
            let latched = self.sense.filters[i].latched;
            let key = (pos, dir);
            let entry = self.wall_streak.entry(key).or_insert(0);
            if latched {
                *entry = (*entry + 1).min(WALL_CONFIRM_K);
                if *entry >= WALL_CONFIRM_K {
                    let before = *self.map.walls.entry(pos).or_insert(0);
                    let bit = 1u8 << (dir as u8);
                    let was = before & bit != 0;
                    self.map.record_wall(pos, dir);
                    if !was {
                        self.trace.event(&format!(
                            "wall ({},{}) {:?} ir={:.2}",
                            pos.0, pos.1, dir,
                            self.sense.filters[i].median()
                        ));
                    }
                }
            } else {
                *entry = (*entry - 1).max(-WALL_CONFIRM_K);
                if *entry <= -WALL_CONFIRM_K {
                    let before = *self.map.walls.entry(pos).or_insert(0);
                    let bit = 1u8 << (dir as u8);
                    let was = before & bit != 0;
                    self.map.record_open(pos, dir);
                    if was {
                        self.trace.event(&format!("open ({},{}) {:?}", pos.0, pos.1, dir));
                    }
                }
            }
        }
        let wall_label = |latched: bool, ir: f64| {
            if latched { format!("WALL({:.1})", ir) } else { format!("open({:.1})", ir) }
        };
        println!(
            "[WALLS] ({},{}) heading={:?}  F={}  L={}  R={}  frontier={}  visited={}",
            self.col,
            self.row,
            self.heading,
            wall_label(self.sense.filters[0].latched, self.sense.ir[0]),
            wall_label(self.sense.filters[1].latched, self.sense.ir[1]),
            wall_label(self.sense.filters[2].latched, self.sense.ir[2]),
            self.map.frontier.len(),
            self.map.visited.len(),
        );
    }

    // ── Motors ────────────────────────────────────────────────────────────────
    fn cmd_motors(&mut self, l: f64, r: f64) {
        self.motor.apply(l, r);
        self.last_l_pow = l;
        self.last_r_pow = r;
        self.mouse.drive_motors(l, r);
    }

    fn heading_error(&self, target_deg: f64) -> f64 {
        let mut e = target_deg - self.sense.compass;
        while e > 180.0 {
            e -= 360.0;
        }
        while e <= -180.0 {
            e += 360.0;
        }
        e
    }

    // ── Motion FSM ────────────────────────────────────────────────────────────
    fn step_motion(&mut self) -> bool {
        match &self.motion {
            Motion::Idle => true,

            Motion::Turning { target_dir, cycles_left } => {
                let c = *cycles_left;
                let td = *target_dir;
                let tgt = td.compass_target();
                let err = self.heading_error(tgt);

                if self.sense.compass_ready && err.abs() < TURN_TOL_DEG {
                    self.turn_in_tol_streak += 1;
                } else {
                    self.turn_in_tol_streak = 0;
                }

                let streak_met = self.turn_in_tol_streak >= TURN_EXIT_K;
                if streak_met || c == 0 {
                    self.cmd_motors(0.0, 0.0);
                    self.motion = Motion::Idle;
                    let aborted = !streak_met;
                    if aborted {
                        println!("[TURN] ABORTED {:?}  err={:.1}deg  compass={:.1}", td, err, self.sense.compass);
                        self.pending_dir = None;
                    } else {
                        println!("[TURN] Locked {:?}  err={:.1}deg  compass={:.1}", td, err, self.sense.compass);
                    }
                    self.log_turn_end(self.sense.compass, tgt, err, aborted);
                    self.turn_in_tol_streak = 0;
                    return true;
                }
                if err > 0.0 {
                    self.cmd_motors(-TURN_POWER, TURN_POWER);
                } else {
                    self.cmd_motors(TURN_POWER, -TURN_POWER);
                }
                self.motion = Motion::Turning { target_dir: td, cycles_left: c - 1 };
                false
            }

            Motion::Driving { cycles_left } => {
                if self.sense.bumper || self.sense.ir[0] > FRONT_STOP {
                    println!(
                        "[BACKUP] ({},{}) heading={:?}  bumper={}  ir_front={:.2}",
                        self.col, self.row, self.heading, self.sense.bumper, self.sense.ir[0]
                    );
                    self.cmd_motors(0.0, 0.0);
                    let cur = (self.row, self.col);
                    let key = (cur, self.heading);
                    let e = self.wall_streak.entry(key).or_insert(0);
                    *e = (*e + 1).min(WALL_CONFIRM_K);
                    let in_return = self.state == AgentState::ReturnToStart;
                    if *e >= WALL_CONFIRM_K || in_return {
                        self.map.record_wall(cur, self.heading);
                        self.trace.event(&format!(
                            "wall-collision ({},{}) {:?}",
                            cur.0, cur.1, self.heading
                        ));
                    }
                    self.motion = Motion::Backup { cycles_left: BACKUP_CYCLES };
                    self.pending_dir = None;
                    return false;
                }

                let left = *cycles_left;
                let trav = self.trav_in_drive();
                let front_block_close = self.sense.filters[0].latched && trav >= CELL_SIZE * 0.5;
                if left == 0 || trav >= DRIVE_DIST_TARGET - DRIVE_DIST_MARGIN || front_block_close {
                    self.cmd_motors(0.0, 0.0);
                    self.motion = Motion::Settling { cycles_left: SETTLE_CYCLES };
                } else {
                    let power = DRIVE_POWER;
                    let err = self.heading_error(self.heading.compass_target());
                    let heading_kp = power * 0.015;
                    let heading_corr = (err * heading_kp).clamp(-power * 0.5, power * 0.5);

                    let sat = |v: f64| v.min(4.0).max(0.0);
                    let li = sat(self.sense.filters[1].median());
                    let ri = sat(self.sense.filters[2].median());
                    let raw_lat = if li > WALL_LOW && ri > WALL_LOW {
                        (li - ri) * LATERAL_KP
                    } else if li > WALL_HIGH {
                        (li - 1.0) * LATERAL_KP * 0.5
                    } else if ri > WALL_HIGH {
                        -(ri - 1.0) * LATERAL_KP * 0.5
                    } else {
                        0.0
                    };
                    let lateral_corr = raw_lat.clamp(-power * 0.3, power * 0.3);

                    let l = (power - heading_corr + lateral_corr).max(0.0);
                    let r = (power + heading_corr - lateral_corr).max(0.0);
                    self.cmd_motors(l, r);
                    self.motion = Motion::Driving { cycles_left: left - 1 };
                }
                false
            }

            Motion::Settling { cycles_left } => {
                let left = *cycles_left;
                self.cmd_motors(0.0, 0.0);
                if left == 0 {
                    self.motion = Motion::Idle;
                    true
                } else {
                    self.motion = Motion::Settling { cycles_left: left - 1 };
                    false
                }
            }

            Motion::Backup { cycles_left } => {
                let left = *cycles_left;
                if left == 0 || self.sense.ir[3] > FRONT_STOP {
                    self.cmd_motors(0.0, 0.0);
                    self.motion = Motion::Idle;
                    true
                } else {
                    self.cmd_motors(BACKUP_POWER, BACKUP_POWER);
                    self.motion = Motion::Backup { cycles_left: left - 1 };
                    false
                }
            }
        }
    }

    fn trav_in_drive(&self) -> f64 {
        match self.drive_start_xy {
            Some((sx, sy)) => (self.ekf.x - sx).hypot(self.ekf.y - sy),
            None => 0.0,
        }
    }

    fn start_move(&mut self, dir: Dir) {
        let err = self.heading_error(dir.compass_target());
        if err.abs() < TURN_TOL_DEG {
            println!("[DRIVE] → {:?}  compass={:.1}deg", dir, self.sense.compass);
            self.heading = dir;
            self.drive_start_xy = Some((self.ekf.x, self.ekf.y));
            self.motion = Motion::Driving { cycles_left: DRIVE_CYCLES };
        } else {
            let raw = ((err.abs() / 90.0) * 14.0).ceil() as u32;
            let cycles = raw.max(10).min(TURN_MAX_CYCLES);
            println!(
                "[TURN]  → {:?}  err={:.1}deg  compass={:.1}  cycles={}",
                dir, err, self.sense.compass, cycles
            );
            for f in self.sense.filters.iter_mut() {
                f.reset();
            }
            self.turn_in_tol_streak = 0;
            self.motion = Motion::Turning { target_dir: dir, cycles_left: cycles };
            self.cur_turn = Some(TurnLog {
                cycle_started: self.cycle_count,
                target_dir: dir,
                start_compass: self.sense.compass,
            });
        }
    }

    // Pose-as-truth arrival. No cardinal snap of theta; cell + heading derived
    // from EKF. Pose anchored along travel axis to remove longitudinal drift.
    fn finish_move(&mut self, dir: Dir) {
        let trav = self.trav_in_drive();
        let advanced = trav >= CELL_SIZE * 0.5;
        if !advanced {
            self.heading = dir;
            self.last_reloc_delta = 0;
            eprintln!(
                "[FINISH] short drive at ({},{})  trav={:.2}",
                self.col, self.row, trav
            );
            return;
        }

        let pose = self.ekf.pose();
        let new_pos = pose_to_cell(&pose);
        let old_pos = (self.row, self.col);

        let (dc, dr) = dir.delta();
        let supposed = (old_pos.0 + dr, old_pos.1 + dc);
        let delta_axis = match dir {
            Dir::East | Dir::West => new_pos.1 - supposed.1,
            Dir::North | Dir::South => new_pos.0 - supposed.0,
        };
        self.last_reloc_delta = delta_axis;

        self.row = new_pos.0;
        self.col = new_pos.1;
        self.heading = pose_to_heading(pose.theta_deg);

        anchor_pose_to_cell_along(&mut self.ekf, new_pos, dir);

        let key_fwd = (old_pos, dir);
        let e = self.wall_streak.entry(key_fwd).or_insert(0);
        *e = -WALL_CONFIRM_K;
        self.map.record_open(old_pos, dir);
        let key_back = (new_pos, dir.opposite());
        let e2 = self.wall_streak.entry(key_back).or_insert(0);
        *e2 = -WALL_CONFIRM_K;

        self.map.mark_visited(new_pos);
        println!(
            "[ARRIVED] col={} row={}  heading={:?}  pose=({:.2},{:.2},θ={:.1}deg)  trav={:.2}  drift={}",
            self.col, self.row, self.heading,
            self.ekf.x, self.ekf.y, self.ekf.pose().theta_deg,
            trav, delta_axis
        );
        self.sense_walls();
    }

    // ── Per-state ticks ───────────────────────────────────────────────────────
    fn tick_wait_start(&mut self) {
        if !self.mouse.get_start_button() {
            self.cmd_motors(0.0, 0.0);
            return;
        }
        if !(self.sense.compass_ready && self.sense.ir_ready) {
            self.cmd_motors(0.0, 0.0);
            return;
        }
        if self.settle_cycles == 0 {
            self.heading = compass_to_dir(self.sense.compass);
            self.ekf.init(self.heading.compass_target().to_radians());
            self.map.mark_visited((0, 0));
            if self.config.debug_gps && self.sense.gps_ready && self.gps_origin.is_none() {
                self.gps_origin = Some((self.sense.gps_x, self.sense.gps_y));
                eprintln!(
                    "[GPS] Origin: ({:.3}, {:.3})",
                    self.sense.gps_x, self.sense.gps_y
                );
            }
        }
        self.cmd_motors(0.0, 0.0);
        self.sense_walls();
        self.settle_cycles += 1;
        if self.settle_cycles >= 5 {
            println!(
                "Start! Heading: {:?}, settled after {} cycles.",
                self.heading, self.settle_cycles
            );
            self.state = AgentState::Explore;
        }
    }

    fn tick_explore(&mut self) {
        if self.sense.ground >= 0 && self.sense.ground != 0 && self.target.is_none() {
            self.target = Some((self.row, self.col));
            println!(
                "Target found at ({},{})  ground={}",
                self.col, self.row, self.sense.ground
            );
            self.state = AgentState::FoundTarget;
            return;
        }

        match self.motion {
            Motion::Idle => {
                if let Some(prev) = self.pending_dir.take() {
                    self.finish_move(prev);
                }
                // Second sense_walls call: streak needs WALL_CONFIRM_K consecutive
                // readings. finish_move gives one (+1/-1); this gives the second.
                self.sense_walls();
                let pos = (self.row, self.col);
                println!(
                    "[PLAN]  at ({},{}) heading={:?}  frontier={}  visited={}",
                    self.col, self.row, self.heading,
                    self.map.frontier.len(), self.map.visited.len()
                );
                if let Some(d) = planner::explore_next(&self.map, pos) {
                    self.last_decision = "frontier";
                    self.pending_dir = Some(d);
                    self.start_move(d);
                } else {
                    self.last_decision = "no-frontier";
                    println!(
                        "[EXPLORE] No frontier reachable from ({},{}). Visited={}. Returning.",
                        self.col,
                        self.row,
                        self.map.visited.len()
                    );
                    self.state = AgentState::ReturnToStart;
                }
            }
            Motion::Turning { .. } => {
                if self.step_motion() {
                    if let Some(d) = self.pending_dir {
                        self.heading = d;
                        self.drive_start_xy = Some((self.ekf.x, self.ekf.y));
                        self.motion = Motion::Driving { cycles_left: DRIVE_CYCLES };
                    }
                }
            }
            _ => {
                if self.step_motion() && self.pending_dir.is_some() {
                    let prev = self.pending_dir.take().unwrap();
                    self.finish_move(prev);
                    if self.sense.ground >= 0 && self.sense.ground != 0 && self.target.is_none() {
                        self.target = Some((self.row, self.col));
                        println!(
                            "Target found at ({},{})  ground={}",
                            self.col, self.row, self.sense.ground
                        );
                        self.state = AgentState::FoundTarget;
                    }
                }
            }
        }
    }

    fn tick_found_target(&mut self) {
        self.cmd_motors(0.0, 0.0);
        self.mouse.set_visiting_led(true);
        self.mouse.set_returning_led(true);
        self.motion = Motion::Idle;
        self.pending_dir = None;
        println!("At target. Returning to start.");
        self.state = AgentState::ReturnToStart;
    }

    fn tick_return(&mut self) {
        match self.motion {
            Motion::Idle => {
                if let Some(prev) = self.pending_dir.take() {
                    self.finish_move(prev);
                }
                if (self.row, self.col) == (0, 0) {
                    self.cmd_motors(0.0, 0.0);
                    self.mouse.finish();
                    println!("Returned to start! Done.");
                    self.state = AgentState::Done;
                    return;
                }
                println!(
                    "[RETURN] at ({},{}) heading={:?}  visited={}",
                    self.col, self.row, self.heading, self.map.visited.len()
                );
                if let Some(d) = planner::return_next(&self.map, (self.row, self.col)) {
                    self.no_progress_cycles = 0;
                    self.pending_dir = Some(d);
                    self.start_move(d);
                } else {
                    self.no_progress_cycles += 1;
                    if self.no_progress_cycles == 1 {
                        println!(
                            "Return: no path to (0,0) from ({},{}). Visited={} Frontier={}",
                            self.col,
                            self.row,
                            self.map.visited.len(),
                            self.map.frontier.len()
                        );
                    }
                    if self.no_progress_cycles > 50 {
                        eprintln!("[RETURN] Stuck >50 cycles. Giving up.");
                        self.cmd_motors(0.0, 0.0);
                        self.mouse.finish();
                        self.state = AgentState::Done;
                    }
                }
            }
            Motion::Turning { .. } => {
                if self.step_motion() {
                    if let Some(d) = self.pending_dir {
                        self.heading = d;
                        self.drive_start_xy = Some((self.ekf.x, self.ekf.y));
                        self.motion = Motion::Driving { cycles_left: DRIVE_CYCLES };
                    }
                }
            }
            _ => {
                if self.step_motion() {
                    if let Some(prev) = self.pending_dir.take() {
                        self.finish_move(prev);
                    }
                }
            }
        }
    }

    // ── Trace ─────────────────────────────────────────────────────────────────
    fn write_trace_row(&mut self) {
        if !self.config.debug_gps {
            return;
        }
        let pose = self.ekf.pose();
        let pending = self
            .pending_dir
            .map(|d| format!("{:?}", d))
            .unwrap_or_default();
        let trav = match (self.drive_start_xy, &self.motion) {
            (Some(_), Motion::Driving { .. }) => self.trav_in_drive(),
            _ => -1.0,
        };
        let ir_filt = [
            self.sense.filters[0].median(),
            self.sense.filters[1].median(),
            self.sense.filters[2].median(),
            self.sense.filters[3].median(),
        ];
        let row = format!(
            "{},{:?},{},{},{},{:?},{:.2},{},{:.3},{:.3},{},{:.2},{:.2},{:.2},{:.2},{},{},{:.4},{:.4},{},{:.3},{:.3},{:.2},{:.5},{:.5},{:.5},{:.2},{:.2},{:.2},{:.2},{:.3},{},{},{}",
            self.cycle_count,
            self.state,
            self.motion.kind(),
            self.row,
            self.col,
            self.heading,
            self.sense.compass,
            self.sense.compass_ready,
            self.sense.gps_x,
            self.sense.gps_y,
            self.sense.gps_ready,
            self.sense.ir[0],
            self.sense.ir[1],
            self.sense.ir[2],
            self.sense.ir[3],
            self.sense.ground,
            self.sense.bumper,
            self.last_l_pow,
            self.last_r_pow,
            pending,
            pose.x,
            pose.y,
            pose.theta_deg,
            self.ekf.p[0][0],
            self.ekf.p[1][1],
            self.ekf.p[2][2],
            ir_filt[0],
            ir_filt[1],
            ir_filt[2],
            ir_filt[3],
            trav,
            self.turn_in_tol_streak,
            self.last_reloc_delta,
            self.last_decision,
        );
        self.trace.writeln(&row);
        self.last_reloc_delta = 0;
    }

    fn log_turn_end(&mut self, end_compass: f64, target_compass: f64, err: f64, aborted: bool) {
        let Some(t) = self.cur_turn.take() else { return };
        let cycles_used = self.cycle_count.saturating_sub(t.cycle_started);
        self.trace.event(&format!(
            "turn started={} ended={} target={:?} tgt_compass={:.2} start={:.2} end={:.2} err={:.2} aborted={} used={}",
            t.cycle_started,
            self.cycle_count,
            t.target_dir,
            target_compass,
            t.start_compass,
            end_compass,
            err,
            aborted,
            cycles_used,
        ));
    }
}

fn now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}
