use crate::cif::protocol::MeasuresMsg;
use serde::{Deserialize, Serialize};

/* All sensor data received from the CiberRato simulator
in a single simulation cycle. This is the internal representation. */
#[derive(Debug, Default, Clone)]
pub struct Measurements {
    pub time: i32, // Current simulation cycle time

    pub compass: f32, // Direction of the robot in ground coordinates (-180.0, 180.0)
    pub compass_ready: bool, // True if a valid compass measure was received

    pub beacons: Vec<BeaconMeasure>, // State and direction of beacons
    pub beacons_ready: Vec<bool>,    // True if beacon measure is valid

    pub collision: bool,       // True if the robot is currently in a collision
    pub collision_ready: bool, // True if the collision status was updated

    pub ground: i32, // Ground sensor value (typically used for target detection)
    pub ground_ready: bool, // True if a ground measure was received

    pub ir_sensor: [f32; 4], // Array of obstacle sensor readings (IR) (0 to 3 in proto)
    pub ir_sensor_ready: [bool; 4], // Array indicating if each IR sensor measure is valid

    pub line_sensor: [bool; 7],  // Array of values from the line sensor
    pub line_sensor_ready: bool, // True if the line sensor data was updated

    pub gps_data: GpsMeasure, // High-precision position data
    pub gps_ready: bool,      // True if GPS data is available

    // State variables for each component
    pub start_button: bool,
    pub stop_button: bool,
    pub end_led: bool,
    pub returning_led: bool,
    pub visiting_led: bool,

    pub hear_message: Vec<String>, // Array of messages heard from other robots
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct GpsMeasure {
    #[serde(rename = "@X")]
    pub x: f32, // Absolute X coordinate
    #[serde(rename = "@Y")]
    pub y: f32, // Absolute Y coordinate
    #[serde(default)]
    pub dir: f32, // Absolute orientation in ground coordinates (if provided)
}

#[derive(Debug, Default, Clone)]
pub struct BeaconMeasure {
    pub dir: f32,      // Relative angle to the beacon in robot coordinates
    pub visible: bool, // True if the beacon is within line-of-sight
}

fn default_sim_time() -> i32 {
    10000
}
fn default_cycle_time() -> i32 {
    100
}

// Global simulation settings received during the initial handshake
#[derive(Debug, Deserialize, Clone, Default)]
pub struct Parameters {
    #[serde(rename = "@SimTime", default = "default_sim_time")]
    pub sim_time: i32, // Maximum simulation time allowed for the round
    #[serde(rename = "@CycleTime", default = "default_cycle_time")]
    pub cycle_time: i32, // Duration of each simulation cycle (in milliseconds)
    #[serde(rename = "@KeyTime", default)]
    pub key_time: i32, // Key time parameter for the simulation

    // Noise levels for each sensor
    #[serde(rename = "@BeaconNoise", default)]
    pub beacon_noise: f32,
    #[serde(rename = "@ObstacleNoise", default)]
    pub obstacle_noise: f32,
    #[serde(rename = "@MotorsNoise", default)]
    pub motors_noise: f32,
    #[serde(rename = "@CompassNoise", default)]
    pub compass_noise: f32,

    #[serde(rename = "@nBeacons", default)]
    pub beacons: i32, // Total number of beacons present in the simulation
}

impl Measurements {
    pub fn update_from(&mut self, msg: &MeasuresMsg) {
        self.time = msg.time;

        if let Some(c) = msg.sensors.compass {
            self.compass = c;
            self.compass_ready = true;
        }
        if let Some(c) = &msg.sensors.collision {
            self.collision = c == "Yes" || c == "Sim";
            self.collision_ready = true;
        }
        if let Some(g) = msg.sensors.ground {
            self.ground = g;
            self.ground_ready = true;
        }

        for ir in &msg.sensors.ir_sensors {
            if ir.id >= 0 && ir.id < 4 {
                self.ir_sensor[ir.id as usize] = ir.value;
                self.ir_sensor_ready[ir.id as usize] = true;
            }
        }

        for beacon in &msg.sensors.beacon_sensors {
            let id = beacon.id as usize;
            if id >= self.beacons.len() {
                continue;
            }

            self.beacons_ready[id] = true;
            if beacon.value == "NotVisible" {
                self.beacons[id].visible = false;
            } else if let Ok(dir) = beacon.value.parse::<f32>() {
                self.beacons[id].dir = dir;
                self.beacons[id].visible = true;
            }
        }

        if let Some(line) = &msg.sensors.line_sensor {
            self.line_sensor_ready = true;
            for (i, c) in line.value.chars().enumerate().take(7) {
                self.line_sensor[i] = c == '1';
            }
        }
        if let Some(gps) = &msg.sensors.gps {
            self.gps_data = gps.clone();
            self.gps_ready = true;
        }

        if let Some(e) = &msg.leds.end_led {
            self.end_led = e == "On";
        }
        if let Some(v) = &msg.leds.visiting_led {
            self.visiting_led = v == "On";
        }
        if let Some(r) = &msg.leds.returning_led {
            self.returning_led = r == "On";
        }

        if let Some(s) = &msg.buttons.start {
            self.start_button = s == "On";
        }
        if let Some(s) = &msg.buttons.stop {
            self.stop_button = s == "On";
        }
    }
}
