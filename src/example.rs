mod cif;

use cif::{CiberIf, CiberMouse};
use std::env;
use std::thread;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq)]
enum State {
    Run,
    Wait,
    Return,
}

struct Wanderer {
    mouse: CiberMouse,
    state: State,
    ir_sensors: [f64; 3],
    beacon_to_follow: usize,
    ground: i32,
    rob_name: String,
}

impl Wanderer {
    fn new(name: &str) -> Self {
        Self {
            mouse: CiberMouse::new().expect("Failed to initialize CiberMouse"),
            state: State::Run,
            ir_sensors: [0.0; 3],
            beacon_to_follow: 0,
            ground: -1,
            rob_name: name.to_string(),
        }
    }

    fn wander(&mut self, follow_beacon: bool) {
        // Priority 1: Emergency Turn (Too close to anything)
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
        }
        // Priority 5: Full Speed Ahead
        else {
            self.mouse.drive_motors(0.15, 0.15);
        }
    }

    fn decide(&mut self) {
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

        // 2. Objective State Machine
        match self.state {
            State::Run => {
                // If we are over the target (Ground == 0), signal "Visiting"
                if self.ground == 0 {
                    self.mouse.set_visiting_led(true);
                    println!(
                        "{} visited target at time {}",
                        self.rob_name,
                        self.mouse.get_time()
                    );
                }

                // If the simulator acknowledged the visit, move to WAIT
                if self.mouse.get_visiting_led() {
                    self.state = State::Wait;
                } else {
                    self.wander(false);
                }
            }
            State::Wait => {
                // Signal "Returning" and turn off "Visiting"
                self.mouse.set_returning_led(true);
                if self.mouse.get_visiting_led() {
                    self.mouse.set_visiting_led(false);
                }

                // If the simulator acknowledges "Returning", move to RETURN phase
                if self.mouse.get_returning_led() {
                    self.state = State::Return;
                }

                // Stop while waiting for state transition
                self.mouse.drive_motors(0.0, 0.0);
            }
            State::Return => {
                // Clean up state and head home
                self.mouse.set_visiting_led(false);
                self.mouse.set_returning_led(false);
                self.wander(false);
            }
        }
    }

    fn run(&mut self, host: &str, pos: i32) {
        println!(
            "Connecting to {} as {} at pos {}...",
            host, self.rob_name, pos
        );

        // Handshake
        if !self.mouse.init_robot(&self.rob_name, pos, host) {
            eprintln!("Failed to initialize robot with simulator!");
            return;
        }

        let cycle_ms = self.mouse.get_cycle_time() as u64;
        let sleep_dur = Duration::from_millis(if cycle_ms > 0 { cycle_ms } else { 100 });

        println!("Connected. Cycle time: {}ms", cycle_ms);

        loop {
            // Read incoming network packets
            self.mouse.read_sensors();

            // Execute logic
            self.decide();

            // Sync with simulator cycle
            thread::sleep(sleep_dur);
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let mut rob_name = String::from("RustWanderer");
    let mut host = String::from("127.0.0.1");
    let mut pos = 1;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--robname" | "-r" => {
                if i + 1 < args.len() {
                    rob_name = args[i + 1].clone();
                    i += 2;
                }
            }
            "--host" | "-h" => {
                if i + 1 < args.len() {
                    host = args[i + 1].clone();
                    i += 2;
                }
            }
            "--pos" | "-p" => {
                if i + 1 < args.len() {
                    pos = args[i + 1].parse().unwrap_or(1);
                    i += 2;
                }
            }
            _ => i += 1,
        }
    }

    let mut agent = Wanderer::new(&rob_name);
    agent.run(&host, pos);
}
