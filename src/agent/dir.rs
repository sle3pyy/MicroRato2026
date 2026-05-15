#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Dir {
    North = 0,
    East = 1,
    South = 2,
    West = 3,
}

impl Dir {
    pub fn compass_target(self) -> f64 {
        match self {
            Dir::North => 90.0,
            Dir::East => 0.0,
            Dir::South => -90.0,
            Dir::West => 180.0,
        }
    }

    pub fn delta(self) -> (i32, i32) {
        match self {
            Dir::North => (0, 1),
            Dir::East => (1, 0),
            Dir::South => (0, -1),
            Dir::West => (-1, 0),
        }
    }

    pub fn opposite(self) -> Dir {
        match self {
            Dir::North => Dir::South,
            Dir::East => Dir::West,
            Dir::South => Dir::North,
            Dir::West => Dir::East,
        }
    }

    pub fn from_index(i: usize) -> Dir {
        match i {
            0 => Dir::North,
            1 => Dir::East,
            2 => Dir::South,
            _ => Dir::West,
        }
    }
}

pub fn neighbor(pos: (i32, i32), dir: Dir) -> (i32, i32) {
    let (dc, dr) = dir.delta();
    (pos.0 + dr, pos.1 + dc)
}

pub fn turn_left(d: Dir) -> Dir {
    match d {
        Dir::North => Dir::West,
        Dir::West => Dir::South,
        Dir::South => Dir::East,
        Dir::East => Dir::North,
    }
}

pub fn turn_right(d: Dir) -> Dir {
    match d {
        Dir::North => Dir::East,
        Dir::East => Dir::South,
        Dir::South => Dir::West,
        Dir::West => Dir::North,
    }
}

pub fn compass_to_dir(compass: f64) -> Dir {
    let n = if compass < 0.0 { compass + 360.0 } else { compass };
    match n {
        x if x < 45.0 || x >= 315.0 => Dir::East,
        x if x < 135.0 => Dir::North,
        x if x < 225.0 => Dir::West,
        _ => Dir::South,
    }
}
