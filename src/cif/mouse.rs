use super::codec::Codec;
use super::network::NetworkHandler;
use super::protocol::{ActionsMsg, IrSensorConfig, RobotRegistration};
use super::state::{BeaconMeasure, Measurements, Parameters};
use crate::cif::CiberIf;

pub struct CiberMouse {
    network: NetworkHandler,
    port: u16,
    hostname: String,
    measurements: Measurements,
    parameters: Parameters,
    counter: i32,
}

impl CiberMouse {
    pub fn new() -> std::io::Result<Self> {
        Ok(Self {
            network: NetworkHandler::new()?,
            port: 6000,
            hostname: String::new(),
            measurements: Measurements::default(),
            parameters: Parameters::default(),
            counter: 0,
        })
    }

    fn send_init_and_parse_reply(&mut self, xml: &str) -> bool {
        self.network.send_str(xml, &self.hostname, self.port).ok();

        let mut buf = [0; 4096];
        let Ok((size, addr)) = self.network.receive(&mut buf) else {
            return false;
        };

        self.port = addr.port(); // Update to the port assigned by simulator

        let Ok(xml_str) = std::str::from_utf8(&buf[..size]) else {
            return false;
        };

        let trimmed = xml_str.trim_matches(char::from(0)).trim();
        let Ok(reply) = Codec::parse_reply(trimmed) else {
            return false;
        };

        if reply.status != "Ok" {
            return false;
        }

        if let Some(params) = reply.parameters {
            self.parameters = params.clone();
            if params.beacons > 0 {
                let n = params.beacons as usize;
                self.measurements
                    .beacons
                    .resize(n, BeaconMeasure::default());
                self.measurements.beacons_ready.resize(n, false);
            }
        }

        true
    }

    fn send_actions(&mut self, actions: &ActionsMsg) {
        let xml = Codec::serialize(actions);
        self.network.send_str(&xml, &self.hostname, self.port).ok();
        self.counter += 1
    }
}

impl CiberIf for CiberMouse {
    fn init_robot(&mut self, name: &str, _pos: i32, hostname: &str) -> bool {
        self.hostname = hostname.to_string();
        let robot = RobotRegistration {
            name: name.to_string(),
            ir_sensors: vec![],
        };
        let xml = Codec::serialize(&robot);
        self.send_init_and_parse_reply(&xml)
    }

    fn init_robot_2(
        &mut self,
        name: &str,
        _pos: i32,
        ir_sensor_angles: &[f64],
        hostname: &str,
    ) -> bool {
        self.hostname = hostname.to_string();
        let ir_sensors = ir_sensor_angles
            .iter()
            .enumerate()
            .map(|(i, &angle)| IrSensorConfig {
                id: i as i32,
                angle: angle as f32,
            })
            .collect();
        let robot = RobotRegistration {
            name: name.to_string(),
            ir_sensors,
        };
        let xml = Codec::serialize(&robot);
        self.send_init_and_parse_reply(&xml)
    }

    fn read_sensors(&mut self) {
        let mut buf = [0; 4096];
        let Ok((size, _addr)) = self.network.receive(&mut buf) else {
            return;
        };
        let Ok(xml_str) = std::str::from_utf8(&buf[..size]) else {
            return;
        };

        let trimmed = xml_str.trim_matches(char::from(0)).trim();
        let Ok(msg) = Codec::parse_measures(trimmed) else {
            return;
        };

        self.measurements.update_from(&msg);
    }

    fn get_time(&self) -> f64 {
        self.measurements.time as f64
    }
    fn is_obstacle_ready(&self, id: usize) -> bool {
        self.measurements.ir_sensor_ready[id]
    }
    fn get_obstacle_sensor(&self, id: usize) -> f64 {
        self.measurements.ir_sensor[id] as f64
    }
    fn is_beacon_ready(&self, id: usize) -> bool {
        self.measurements
            .beacons_ready
            .get(id)
            .copied()
            .unwrap_or(false)
    }
    fn get_beacon_visible(&self, id: usize) -> bool {
        self.measurements
            .beacons
            .get(id)
            .map_or(false, |b| b.visible)
    }
    fn get_beacon_dir(&self, id: usize) -> f64 {
        self.measurements
            .beacons
            .get(id)
            .map_or(0.0, |b| b.dir as f64)
    }
    fn is_compass_ready(&self) -> bool {
        self.measurements.compass_ready
    }
    fn get_compass_sensor(&self) -> f64 {
        self.measurements.compass as f64
    }
    fn get_line_sensor(&self) -> &[bool; 7] {
        &self.measurements.line_sensor
    }
    fn is_ground_ready(&self) -> bool {
        self.measurements.ground_ready
    }
    fn get_ground_sensor(&self) -> i32 {
        self.measurements.ground
    }
    fn is_bumper_ready(&self) -> bool {
        self.measurements.collision_ready
    }
    fn get_bumper_sensor(&self) -> bool {
        self.measurements.collision
    }
    fn new_message_from(&self, _id: usize) -> bool {
        false
    } // Not fully implemented in parser
    fn get_message_from(&self, _id: usize) -> Option<&String> {
        None
    } // Not fully implemented
    fn is_gps_ready(&self) -> bool {
        self.measurements.gps_ready
    }
    fn get_x(&self) -> f64 {
        self.measurements.gps_data.x as f64
    }
    fn get_y(&self) -> f64 {
        self.measurements.gps_data.y as f64
    }
    fn get_dir(&self) -> f64 {
        self.measurements.gps_data.dir as f64
    }
    fn get_start_button(&self) -> bool {
        self.measurements.start_button
    }
    fn get_stop_button(&self) -> bool {
        self.measurements.stop_button
    }
    fn get_visiting_led(&self) -> bool {
        self.measurements.visiting_led
    }
    fn get_returning_led(&self) -> bool {
        self.measurements.returning_led
    }
    fn get_finished(&self) -> bool {
        self.measurements.end_led
    }

    fn request_compass_sensor(&mut self) {
        let xml = Codec::build_sensor_request("Compass");
        self.network.send_str(&xml, &self.hostname, self.port).ok();
    }

    fn request_ground_sensor(&mut self) {
        let xml = Codec::build_sensor_request("Ground");
        self.network.send_str(&xml, &self.hostname, self.port).ok();
    }

    fn request_ir_sensor(&mut self, id: usize) {
        let xml = Codec::build_sensor_request(&format!("IRSensor{}", id));
        self.network.send_str(&xml, &self.hostname, self.port).ok();
    }

    fn request_beacon_sensor(&mut self, id: usize) {
        let xml = Codec::build_sensor_request(&format!("Beacon{}", id));
        self.network.send_str(&xml, &self.hostname, self.port).ok();
    }

    fn request_sensors(&mut self, sensor_ids: &[&str]) {
        let xml = Codec::build_sensor_requests(sensor_ids);
        self.network.send_str(&xml, &self.hostname, self.port).ok();
    }

    fn drive_motors(&mut self, l_pow: f64, r_pow: f64) {
        let actions = ActionsMsg {
            left_motor: l_pow as f32,
            right_motor: r_pow as f32,
            ..Default::default()
        };
        self.send_actions(&actions);
    }

    fn say(&mut self, msg: &str) {
        let actions = ActionsMsg {
            say: msg.to_string(),
            ..Default::default()
        };
        self.send_actions(&actions);
    }

    fn set_returning_led(&mut self, val: bool) {
        let actions = ActionsMsg {
            returning_led: if val {
                "On".to_string()
            } else {
                "Off".to_string()
            },
            ..Default::default()
        };
        self.send_actions(&actions);
    }

    fn set_visiting_led(&mut self, val: bool) {
        let actions = ActionsMsg {
            visiting_led: if val {
                "On".to_string()
            } else {
                "Off".to_string()
            },
            ..Default::default()
        };
        self.send_actions(&actions);
    }

    fn finish(&mut self) {
        let actions = ActionsMsg {
            end_led: "On".to_string(),
            ..Default::default()
        };
        self.send_actions(&actions);
    }

    fn get_cycle_time(&self) -> i32 {
        self.parameters.cycle_time
    }
    fn get_final_time(&self) -> i32 {
        self.parameters.sim_time
    }
    fn get_key_time(&self) -> i32 {
        self.parameters.key_time
    }
    fn get_noise_obstacle_sensor(&self) -> f32 {
        self.parameters.obstacle_noise
    }
    fn get_noise_beacon_sensor(&self) -> f32 {
        self.parameters.beacon_noise
    }
    fn get_noise_compass_sensor(&self) -> f32 {
        self.parameters.compass_noise
    }
    fn get_noise_motors(&self) -> f32 {
        self.parameters.motors_noise
    }
    fn get_number_of_beacons(&self) -> i32 {
        self.parameters.beacons
    }
}
