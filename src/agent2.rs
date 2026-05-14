use crate::Config;
use crate::agent_helpers::heading::{Heading, angle_error};
use crate::agent_helpers::position_tracker::PositionTracker;
use crate::agent_helpers::wall_follow::{RightWallFollower, SensorSnapshot, WallFollowAction};
use crate::cif::{CiberIf, CiberMouse};

const FRONT_IR: usize = 0;
const LEFT_IR: usize = 1;
const RIGHT_IR: usize = 2;
const COLLISION_RECOVERY_TICKS: u32 = 8;
const TURN_TOLERANCE_DEGREES: f64 = 7.0;
const TURN_SPEED: f64 = 0.12;
const MAX_MOTOR_POWER: f64 = 0.15;
const GROUND_REQUEST_PERIOD: u32 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MissionState {
    Initialize,
    Explore,
    SignalReturn,
    Return,
    Finished,
}

#[derive(Debug, Clone, Copy)]
struct SensorCache {
    front_ir: f64,
    left_ir: f64,
    right_ir: f64,
    compass: Option<f64>,
    heading: Heading,
    ground: i32,
    bumper: bool,
}

impl Default for SensorCache {
    fn default() -> Self {
        Self {
            front_ir: 0.0,
            left_ir: 0.0,
            right_ir: 0.0,
            compass: None,
            heading: Heading::East,
            ground: -1,
            bumper: false,
        }
    }
}

pub struct Agent2 {
    mouse: CiberMouse,
    state: MissionState,
    sensors: SensorCache,
    wall_follower: RightWallFollower,
    position_tracker: PositionTracker,
    turn_target: Option<Heading>,
    collision_recovery_ticks: u32,
    debug_tick: u32,
    processed_steps: u32,
    last_processed_time: Option<i32>,
    estimated_left_out: f64,
    estimated_right_out: f64,
    config: Config,
}

impl Agent2 {
    pub fn new(config: Config) -> Self {
        Self {
            mouse: CiberMouse::new().expect("Failed to initialize CiberMouse"),
            state: MissionState::Initialize,
            sensors: SensorCache::default(),
            wall_follower: RightWallFollower::default(),
            position_tracker: PositionTracker::default(),
            turn_target: None,
            collision_recovery_ticks: 0,
            debug_tick: 0,
            processed_steps: 0,
            last_processed_time: None,
            estimated_left_out: 0.0,
            estimated_right_out: 0.0,
            config,
        }
    }

    pub fn connect(&mut self) {
        println!(
            "Connecting to {} as {} at pos {}...",
            self.config.host, self.config.name, self.config.pos
        );

        if !self
            .mouse
            .init_robot(&self.config.name, self.config.pos, &self.config.host)
        {
            eprintln!("Failed to initialize robot with simulator!");
            return;
        }

        let cycle_ms = self.mouse.get_cycle_time().max(10) as u64;
        println!("Connected. Cycle time: {}ms", cycle_ms);
        self.request_navigation_sensors(true);
        self.mouse.send_actions();

        loop {
            if !self.mouse.read_sensors() {
                continue;
            }

            let current_time = self.mouse.get_time() as i32;
            if self
                .last_processed_time
                .is_some_and(|last_time| current_time <= last_time)
            {
                continue;
            }
            self.last_processed_time = Some(current_time);
            self.processed_steps = self.processed_steps.wrapping_add(1);

            self.tick();
            self.request_navigation_sensors(
                self.processed_steps.is_multiple_of(GROUND_REQUEST_PERIOD),
            );
            self.mouse.send_actions();
        }
    }

    fn tick(&mut self) {
        self.debug_tick = self.debug_tick.wrapping_add(1);
        self.update_sensor_cache();
        self.position_tracker.update(
            self.sensors.compass,
            self.estimated_left_out,
            self.estimated_right_out,
            self.sensors.bumper,
        );

        if self.mouse.get_finished() || self.state == MissionState::Finished {
            self.debug_log("finished");
            self.state = MissionState::Finished;
            self.drive_target_motors(0.0, 0.0);
            return;
        }

        if self.mouse.get_stop_button() {
            self.debug_log("stop-button");
            self.drive_target_motors(0.0, 0.0);
            return;
        }

        if self.sensors.bumper {
            self.collision_recovery_ticks = COLLISION_RECOVERY_TICKS;
        }

        if self.collision_recovery_ticks > 0 {
            self.collision_recovery_ticks -= 1;
            self.turn_target = None;
            self.debug_log("collision-recovery");
            self.drive_target_motors(-0.08, 0.1);
            return;
        }

        match self.state {
            MissionState::Initialize => self.initialize_start_pose(),
            MissionState::Explore => self.explore(),
            MissionState::SignalReturn => self.signal_return(),
            MissionState::Return => self.return_to_start(),
            MissionState::Finished => self.drive_target_motors(0.0, 0.0),
        }
    }

    fn initialize_start_pose(&mut self) {
        self.drive_target_motors(0.0, 0.0);

        if let Some(compass) = self.sensors.compass {
            self.sensors.heading = Heading::from_compass(compass);
            println!(
                "{} initialized start pose at logical cell (0, 0), heading {:?}",
                self.config.name, self.sensors.heading
            );
            self.debug_log("initialized");
            self.state = MissionState::Explore;
        }
    }

    fn update_sensor_cache(&mut self) {
        if self.mouse.is_obstacle_ready(FRONT_IR) {
            self.sensors.front_ir = self.mouse.get_obstacle_sensor(FRONT_IR);
        }
        if self.mouse.is_obstacle_ready(LEFT_IR) {
            self.sensors.left_ir = self.mouse.get_obstacle_sensor(LEFT_IR);
        }
        if self.mouse.is_obstacle_ready(RIGHT_IR) {
            self.sensors.right_ir = self.mouse.get_obstacle_sensor(RIGHT_IR);
        }
        if self.mouse.is_compass_ready() {
            let compass = self.mouse.get_compass_sensor();
            self.sensors.compass = Some(compass);
            self.sensors.heading = Heading::from_compass(compass);
        }
        if self.mouse.is_ground_ready() {
            self.sensors.ground = self.mouse.get_ground_sensor();
        }
        if self.mouse.is_bumper_ready() {
            self.sensors.bumper = self.mouse.get_bumper_sensor();
        }
    }

    fn explore(&mut self) {
        if self.sensors.ground == 0 {
            println!(
                "{} found target at time {}",
                self.config.name,
                self.mouse.get_time()
            );
            self.debug_log("target-detected");
            self.mouse.set_visiting_led(true);
            self.drive_target_motors(0.0, 0.0);
            self.state = MissionState::SignalReturn;
            return;
        }

        self.debug_log_periodic("explore");
        self.follow_right_wall();
    }

    fn signal_return(&mut self) {
        self.debug_log_periodic("signal-return");
        self.mouse.set_returning_led(true);
        self.mouse.set_visiting_led(false);
        self.drive_target_motors(0.0, 0.0);

        if self.mouse.get_returning_led() {
            println!(
                "{} returning from target at time {}",
                self.config.name,
                self.mouse.get_time()
            );
            self.debug_log("returning");
            self.state = MissionState::Return;
        }
    }

    fn return_to_start(&mut self) {
        self.mouse.set_visiting_led(false);
        self.mouse.set_returning_led(false);

        // This is still wall-follow return, not shortest-path return. The module split
        // leaves room for replacing this with a mapped path planner next.
        self.debug_log_periodic("return");
        self.follow_right_wall();
    }

    fn follow_right_wall(&mut self) {
        if self.run_turn() {
            return;
        }

        let action = self.wall_follower.next_action(SensorSnapshot {
            front_ir: self.sensors.front_ir,
            left_ir: self.sensors.left_ir,
            right_ir: self.sensors.right_ir,
        });

        match action {
            WallFollowAction::Forward(command) => {
                self.debug_log_periodic("forward");
                self.drive_target_motors(command.left, command.right);
            }
            WallFollowAction::ArcLeft(command) => {
                self.debug_log_periodic("arc-left");
                self.drive_target_motors(command.left, command.right);
            }
            WallFollowAction::ArcRight(command) => {
                self.debug_log_periodic("arc-right");
                self.drive_target_motors(command.left, command.right);
            }
            WallFollowAction::TurnLeft => {
                self.turn_target = Some(self.sensors.heading.left());
                self.debug_log("turn-left");
                self.run_turn();
            }
        }
    }

    fn run_turn(&mut self) -> bool {
        let Some(target) = self.turn_target else {
            return false;
        };

        let Some(compass) = self.sensors.compass else {
            self.drive_target_motors(0.0, 0.0);
            return true;
        };

        let error = angle_error(target.degrees(), compass);
        if error.abs() <= TURN_TOLERANCE_DEGREES {
            self.sensors.heading = target;
            self.turn_target = None;
            self.debug_log("turn-complete");
            self.drive_target_motors(0.0, 0.0);
            return true;
        }

        if error > 0.0 {
            self.drive_target_motors(-TURN_SPEED, TURN_SPEED);
        } else {
            self.drive_target_motors(TURN_SPEED, -TURN_SPEED);
        }

        true
    }

    fn drive_target_motors(&mut self, target_left: f64, target_right: f64) {
        // In the simulator, out_t = (out_{t-1} + in_t) / 2. Solve for in_t so the
        // effective wheel power tracks the target more closely despite motor inertia.
        let left_input =
            (2.0 * target_left - self.estimated_left_out).clamp(-MAX_MOTOR_POWER, MAX_MOTOR_POWER);
        let right_input = (2.0 * target_right - self.estimated_right_out)
            .clamp(-MAX_MOTOR_POWER, MAX_MOTOR_POWER);

        self.mouse.drive_motors(left_input, right_input);

        self.estimated_left_out = (self.estimated_left_out + left_input) / 2.0;
        self.estimated_right_out = (self.estimated_right_out + right_input) / 2.0;
    }

    fn request_navigation_sensors(&mut self, request_ground: bool) {
        if request_ground {
            self.mouse
                .request_sensors(&["IRSensor0", "IRSensor1", "IRSensor2", "Ground", "Compass"]);
        } else {
            self.mouse
                .request_sensors(&["IRSensor0", "IRSensor1", "IRSensor2", "Compass"]);
        }
    }

    fn debug_log_periodic(&self, label: &str) {
        if self.debug_tick % 10 == 0 {
            self.debug_log(label);
        }
    }

    fn debug_log(&self, label: &str) {
        let compass = self
            .sensors
            .compass
            .map(|value| format!("{value:>6.1}"))
            .unwrap_or_else(|| "  none".to_string());
        let turn_target = self
            .turn_target
            .map(|heading| format!("{heading:?}"))
            .unwrap_or_else(|| "-".to_string());
        let current_cell = self.position_tracker.current_cell();
        let visited_cells = self.position_tracker.visited_cells();

        println!(
            "[dbg t={:>5.0} {}] state={:?} heading={:?} cell=({}, {}) seen={} target={} front={:>4.2} left={:>4.2} right={:>4.2} compass={} ground={} bumper={}",
            self.mouse.get_time(),
            label,
            self.state,
            self.sensors.heading,
            current_cell.x,
            current_cell.y,
            visited_cells,
            turn_target,
            self.sensors.front_ir,
            self.sensors.left_ir,
            self.sensors.right_ir,
            compass,
            self.sensors.ground,
            self.sensors.bumper
        );
    }
}
