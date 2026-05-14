#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Heading {
    East,
    North,
    West,
    South,
}

impl Heading {
    pub fn from_compass(degrees: f64) -> Self {
        let degrees = normalize_angle(degrees);
        let candidates = [
            (Heading::East, 0.0),
            (Heading::North, 90.0),
            (Heading::West, 180.0),
            (Heading::South, -90.0),
        ];

        candidates
            .iter()
            .min_by(|(_, a), (_, b)| {
                angle_error(*a, degrees)
                    .abs()
                    .partial_cmp(&angle_error(*b, degrees).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(heading, _)| *heading)
            .unwrap_or(Heading::East)
    }

    pub fn degrees(self) -> f64 {
        match self {
            Heading::East => 0.0,
            Heading::North => 90.0,
            Heading::West => 180.0,
            Heading::South => -90.0,
        }
    }

    pub fn left(self) -> Self {
        match self {
            Heading::East => Heading::North,
            Heading::North => Heading::West,
            Heading::West => Heading::South,
            Heading::South => Heading::East,
        }
    }

}

pub fn normalize_angle(mut degrees: f64) -> f64 {
    while degrees <= -180.0 {
        degrees += 360.0;
    }
    while degrees > 180.0 {
        degrees -= 360.0;
    }
    degrees
}

pub fn angle_error(target_degrees: f64, current_degrees: f64) -> f64 {
    normalize_angle(target_degrees - current_degrees)
}
