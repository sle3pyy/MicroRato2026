use crate::cif::state::{GpsMeasure, Parameters};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Default)]
#[serde(rename = "Robot")]
pub struct RobotRegistration {
    #[serde(rename = "@Name")]
    pub name: String,

    #[serde(rename = "IRSensor", default)]
    pub ir_sensors: Vec<IrSensorConfig>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct IrSensorConfig {
    #[serde(rename = "@Id")]
    pub id: i32,
    #[serde(rename = "@Angle")]
    pub angle: f32,
}

#[derive(Debug, Deserialize)]
#[serde(rename = "Reply")]
pub struct Reply {
    #[serde(rename = "@Status")]
    pub status: String,

    #[serde(rename = "Parameters")]
    pub parameters: Option<Parameters>,
}

#[derive(Debug, Deserialize)]
#[serde(rename = "Measures")]
pub struct MeasuresMsg {
    #[serde(rename = "@Time")]
    pub time: i32,

    #[serde(rename = "Sensors", default)]
    pub sensors: SensorsMsg,

    #[serde(rename = "Leds", default)]
    pub leds: LedsMsg,

    #[serde(rename = "Buttons", default)]
    pub buttons: ButtonsMsg,
}

#[derive(Debug, Deserialize, Default)]
pub struct SensorsMsg {
    #[serde(rename = "@Compass")]
    pub compass: Option<f32>,
    #[serde(rename = "@Collision")]
    pub collision: Option<String>, // "Sim" / "Não"
    #[serde(rename = "@Ground")]
    pub ground: Option<i32>,

    #[serde(rename = "IRSensor", default)]
    pub ir_sensors: Vec<IrSensorValue>,

    #[serde(rename = "BeaconSensor", default)]
    pub beacon_sensors: Vec<BeaconSensorValue>,

    #[serde(rename = "LineSensor")]
    pub line_sensor: Option<LineSensorValue>,

    #[serde(rename = "GPS")]
    pub gps: Option<GpsMeasure>,
}

#[derive(Debug, Deserialize)]
pub struct BeaconSensorValue {
    #[serde(rename = "@Id")]
    pub id: i32,
    #[serde(rename = "@Value")]
    pub value: String,
}

#[derive(Debug, Deserialize)]
pub struct LineSensorValue {
    #[serde(rename = "@Value")]
    pub value: String,
}

#[derive(Debug, Deserialize)]
pub struct IrSensorValue {
    #[serde(rename = "@Id")]
    pub id: i32,
    #[serde(rename = "@Value")]
    pub value: f32,
}

#[derive(Debug, Deserialize, Default)]
pub struct LedsMsg {
    #[serde(rename = "@EndLed")]
    pub end_led: Option<String>, // "On" / "Off"
    #[serde(rename = "@VisitingLed")]
    pub visiting_led: Option<String>,
    #[serde(rename = "@ReturningLed")]
    pub returning_led: Option<String>,
}

#[derive(Debug, Deserialize, Default)]
pub struct ButtonsMsg {
    #[serde(rename = "@Start")]
    pub start: Option<String>, // "On" / "Off"
    #[serde(rename = "@Stop")]
    pub stop: Option<String>,
}

#[derive(Debug, Serialize, Deserialize, Default)]
#[serde(rename = "Actions")]
pub struct ActionsMsg {
    #[serde(rename = "@LeftMotor", default, skip_serializing_if = "Option::is_none")]
    pub left_motor: Option<f32>,
    #[serde(rename = "@RightMotor", default, skip_serializing_if = "Option::is_none")]
    pub right_motor: Option<f32>,

    #[serde(rename = "@VisitingLed", default, skip_serializing_if = "String::is_empty")]
    pub visiting_led: String,
    #[serde(rename = "@ReturningLed", default, skip_serializing_if = "String::is_empty")]
    pub returning_led: String,
    #[serde(rename = "@EndLed", default, skip_serializing_if = "String::is_empty")]
    pub end_led: String,

    #[serde(rename = "Say", default, skip_serializing_if = "String::is_empty")]
    pub say: String,

    #[serde(rename = "SensorRequests", default, skip_serializing_if = "Option::is_none")]
    pub sensor_requests: Option<SensorRequests>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct SensorRequests {
    #[serde(rename = "@IRSensor0", default, skip_serializing_if = "String::is_empty")]
    pub ir0: String,
    #[serde(rename = "@IRSensor1", default, skip_serializing_if = "String::is_empty")]
    pub ir1: String,
    #[serde(rename = "@IRSensor2", default, skip_serializing_if = "String::is_empty")]
    pub ir2: String,
    #[serde(rename = "@IRSensor3", default, skip_serializing_if = "String::is_empty")]
    pub ir3: String,
    #[serde(rename = "@Compass", default, skip_serializing_if = "String::is_empty")]
    pub compass: String,
    #[serde(rename = "@Ground", default, skip_serializing_if = "String::is_empty")]
    pub ground: String,
}

impl SensorRequests {
    pub fn set(&mut self, name: &str) -> bool {
        let slot = match name {
            "IRSensor0" => &mut self.ir0,
            "IRSensor1" => &mut self.ir1,
            "IRSensor2" => &mut self.ir2,
            "IRSensor3" => &mut self.ir3,
            "Compass"   => &mut self.compass,
            "Ground"    => &mut self.ground,
            _ => return false,
        };
        *slot = "Yes".to_string();
        true
    }

    pub fn count(&self) -> usize {
        [&self.ir0, &self.ir1, &self.ir2, &self.ir3, &self.compass, &self.ground]
            .iter().filter(|s| !s.is_empty()).count()
    }
}
