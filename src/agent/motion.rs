use super::dir::Dir;

#[derive(Debug, PartialEq)]
pub enum Motion {
    Idle,
    Turning { target_dir: Dir, cycles_left: u32 },
    Driving { cycles_left: u32 },
    Settling { cycles_left: u32 },
    Backup { cycles_left: u32 },
}

impl Motion {
    pub fn kind(&self) -> &'static str {
        match self {
            Motion::Idle => "Idle",
            Motion::Turning { .. } => "Turning",
            Motion::Driving { .. } => "Driving",
            Motion::Settling { .. } => "Settling",
            Motion::Backup { .. } => "Backup",
        }
    }
}
