use crate::Config;
use crate::cif::{CiberIf, CiberMouse};
use std::thread;
use std::time::Duration;

pub enum State {
    Run,
    Wait,
    Return,
}
pub struct Agent {
    mouse: CiberMouse,
    state: State,
    ir_sensors: [f64; 3],
    beacon_to_follow: usize,
    ground: i32,
    collision_recover_ticks: u32,
    config: Config,
}

impl Agent {
    pub fn new(config: Config) -> Self {
        Self {
            mouse: CiberMouse::new().expect("Failed to initialize CiberMouse"),
            state: State::Run,
            ir_sensors: [0.0; 3],
            beacon_to_follow: 0,
            ground: -1,
            collision_recover_ticks: 0,
            config: config,
        }
    }

    pub fn connect(&mut self) {
        println!(
            "Connecting to {} as {} at pos {}...",
            self.config.host, self.config.name, self.config.pos
        );

        // Handshake
        if !self
            .mouse
            .init_robot(&self.config.name, self.config.pos, &self.config.host)
        {
            eprintln!("Failed to initialize robot with simulator!");
            return;
        }

        let cycle_ms = self.mouse.get_cycle_time() as u64;
        println!("Connected. Cycle time: {}ms", cycle_ms);
        let sleep_dur = Duration::from_millis(cycle_ms.max(10));

        loop {
            self.mouse.read_sensors();
            self.explore();
            thread::sleep(sleep_dur);
        }
    }

    fn explore(&mut self) {
        // 1. Update internal sensor cache
        if self.mouse.is_obstacle_ready(0) {
            self.ir_sensors[0] = self.mouse.get_obstacle_sensor(0);
        }
        if self.mouse.is_obstacle_ready(1) {
            self.ir_sensors[1] = self.mouse.get_obstacle_sensor(1);
        }
        if self.mouse.is_obstacle_ready(2) {
            self.ir_sensors[2] = self.mouse.get_obstacle_sensor(2);
        }

        if self.mouse.is_ground_ready() {
            self.ground = self.mouse.get_ground_sensor();
        }

        // 2. Collision recovery takes highest priority
        if self.mouse.get_bumper_sensor() {
            // Back up and turn for 8 ticks to escape the wall
            self.collision_recover_ticks = 8;
        }

        if self.collision_recover_ticks > 0 {
            self.mouse.drive_motors(-0.1, 0.1);
            self.collision_recover_ticks -= 1;
            return;
        }

        // 3. Objective State Machine
        match self.state {
            State::Run => {
                if self.ground == 0 {
                    self.mouse.set_visiting_led(true);
                    println!(
                        "{} visited target at time {}",
                        self.config.name,
                        self.mouse.get_time()
                    );
                }

                if self.mouse.get_visiting_led() {
                    self.state = State::Wait;
                } else {
                    self.next_move(false);
                }
            }
            State::Wait => {
                self.mouse.set_returning_led(true);
                if self.mouse.get_visiting_led() {
                    self.mouse.set_visiting_led(false);
                }

                if self.mouse.get_returning_led() {
                    self.state = State::Return;
                }

                self.mouse.drive_motors(0.0, 0.0);
            }
            State::Return => {
                self.mouse.set_visiting_led(false);
                self.mouse.set_returning_led(false);
                self.next_move(false);
            }
        }
    }

    fn next_move(&mut self, follow_beacon: bool) {
        if self.ir_sensors[0] > 4.0 || self.ir_sensors[1] > 4.0 || self.ir_sensors[2] > 4.0 {
            self.mouse.drive_motors(-0.1, 0.1);
        }
        // Priority 2: Left Side Obstacle
        else if self.ir_sensors[1] > 0.7 {
            self.mouse.drive_motors(0.15, 0.0);
        }
        // Priority 3: Right Side Obstacle
        else if self.ir_sensors[2] > 0.7 {
            self.mouse.drive_motors(0.0, 0.15);
        }
        // Priority 4: Steer toward Beacon
        else if follow_beacon
            && self.mouse.get_beacon_visible(self.beacon_to_follow)
            && self.mouse.get_beacon_dir(self.beacon_to_follow) > 20.0
        {
            self.mouse.drive_motors(0.0, 0.1);
        } else if follow_beacon
            && self.mouse.get_beacon_visible(self.beacon_to_follow)
            && self.mouse.get_beacon_dir(self.beacon_to_follow) < -20.0
        {
            self.mouse.drive_motors(0.1, 0.0);
        } else {
            self.mouse.drive_motors(0.15, 0.15);
        }
    }
}
