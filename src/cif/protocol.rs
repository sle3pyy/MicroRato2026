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

#[derive(Debug, Serialize, Default)]
#[serde(rename = "Actions")]
pub struct ActionsMsg {
    #[serde(rename = "@LeftMotor")]
    pub left_motor: f32,
    #[serde(rename = "@RightMotor")]
    pub right_motor: f32,

    #[serde(rename = "@VisitingLed", skip_serializing_if = "String::is_empty")]
    pub visiting_led: String,
    #[serde(rename = "@ReturningLed", skip_serializing_if = "String::is_empty")]
    pub returning_led: String,
    #[serde(rename = "@EndLed", skip_serializing_if = "String::is_empty")]
    pub end_led: String,

    #[serde(rename = "Say", skip_serializing_if = "String::is_empty")]
    pub say: String,
}
