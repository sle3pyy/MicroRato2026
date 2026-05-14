mod agent;
mod cif;

use crate::agent::Agent;
use serde::Deserialize;
use std::error::Error;
use std::fs;

#[derive(Debug, Deserialize)]
pub struct Config {
    host: String,
    port: u16,
    pos: i32,
    name: String,
}

fn main() {
    let config: Config = match read_config() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Application error: {}", e);
            std::process::exit(1)
        }
    };
    println!(
        "Sucessufully loaded robot {} config for host: {} ",
        config.pos, config.host
    );

    let mut agent = Agent::new(config);

    agent.connect();
}

pub fn read_config() -> Result<Config, Box<dyn Error>> {
    let yaml_content = fs::read_to_string("config.yaml")?;

    let config: Config = serde_yaml::from_str(&yaml_content)?;

    println!("Connecting to {} on port {}", config.host, config.port);

    Ok(config)
}
