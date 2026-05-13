pub trait CiberIf {
    // Initialization
    fn init_robot(&mut self, name: &str, pos: i32, hostname: &str) -> bool;
    fn init_robot_2(
        &mut self,
        name: &str,
        pos: i32,
        ir_sensor_angles: &[f64],
        hostname: &str,
    ) -> bool;

    // Main Sync Method
    fn read_sensors(&mut self);

    // Getters for Measures
    fn get_time(&self) -> f64;

    fn is_obstacle_ready(&self, id: usize) -> bool;
    fn get_obstacle_sensor(&self, id: usize) -> f64;

    fn is_beacon_ready(&self, id: usize) -> bool;
    fn get_beacon_visible(&self, id: usize) -> bool;
    fn get_beacon_dir(&self, id: usize) -> f64;

    fn is_compass_ready(&self) -> bool;
    fn get_compass_sensor(&self) -> f64;

    fn get_line_sensor(&self) -> &[bool; 7];

    fn is_ground_ready(&self) -> bool;
    fn get_ground_sensor(&self) -> i32;

    fn is_bumper_ready(&self) -> bool;
    fn get_bumper_sensor(&self) -> bool;

    fn new_message_from(&self, id: usize) -> bool;
    fn get_message_from(&self, id: usize) -> Option<&String>;

    fn is_gps_ready(&self) -> bool;
    fn get_x(&self) -> f64;
    fn get_y(&self) -> f64;
    fn get_dir(&self) -> f64;

    fn get_start_button(&self) -> bool;
    fn get_stop_button(&self) -> bool;
    fn get_visiting_led(&self) -> bool;
    fn get_returning_led(&self) -> bool;
    fn get_finished(&self) -> bool;

    // Requests
    fn request_compass_sensor(&mut self);
    fn request_ground_sensor(&mut self);
    fn request_ir_sensor(&mut self, id: usize);
    fn request_beacon_sensor(&mut self, id: usize);
    fn request_sensors(&mut self, sensor_ids: &[&str]);

    // Actions
    fn drive_motors(&mut self, l_pow: f64, r_pow: f64);
    fn say(&mut self, msg: &str);
    fn set_returning_led(&mut self, val: bool);
    fn set_visiting_led(&mut self, val: bool);
    fn finish(&mut self);

    // Getters for Parameters
    fn get_cycle_time(&self) -> i32;
    fn get_final_time(&self) -> i32;
    fn get_key_time(&self) -> i32;
    fn get_noise_obstacle_sensor(&self) -> f32;
    fn get_noise_beacon_sensor(&self) -> f32;
    fn get_noise_compass_sensor(&self) -> f32;
    fn get_noise_motors(&self) -> f32;
    fn get_number_of_beacons(&self) -> i32;
}
