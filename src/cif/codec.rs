use super::protocol::{MeasuresMsg, Reply};
use quick_xml::de::from_str;
use quick_xml::se::to_string;
use serde::Serialize;

pub struct Codec;

impl Codec {
    pub fn parse_reply(xml: &str) -> Result<Reply, quick_xml::DeError> {
        from_str(xml)
    }

    pub fn parse_measures(xml: &str) -> Result<MeasuresMsg, quick_xml::DeError> {
        from_str(xml)
    }

    pub fn serialize<T: Serialize>(obj: &T) -> String {
        to_string(obj).unwrap_or_default()
    }

    pub fn build_sensor_request(id: &str) -> String {
        format!("<Actions> <SensorRequests {}=\"Yes\"/> </Actions>", id)
    }

    pub fn build_sensor_requests(ids: &[&str]) -> String {
        let mut req_str = String::from("<Actions> <SensorRequests ");
        for id in ids {
            req_str.push_str(&format!("{}=\"Yes\" ", id));
        }
        req_str.push_str("/> </Actions>");
        req_str
    }
}
