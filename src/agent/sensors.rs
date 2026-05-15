use super::config::{WALL_HIGH, WALL_LOW};

pub struct IrFilter {
    buf: [f64; 5],
    idx: usize,
    count: usize,
    pub latched: bool,
}

impl IrFilter {
    pub fn new() -> Self {
        Self { buf: [0.0; 5], idx: 0, count: 0, latched: false }
    }
    // Drop history on turn entry: sensor about to face a new cardinal direction.
    pub fn reset(&mut self) {
        self.buf = [0.0; 5];
        self.idx = 0;
        self.count = 0;
        self.latched = false;
    }
    pub fn push(&mut self, v: f64) {
        self.buf[self.idx] = v;
        self.idx = (self.idx + 1) % 5;
        if self.count < 5 {
            self.count += 1;
        }
        let m = self.median();
        if m >= WALL_HIGH {
            self.latched = true;
        } else if m <= WALL_LOW {
            self.latched = false;
        }
    }
    pub fn median(&self) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let mut v: Vec<f64> = self.buf[..self.count].to_vec();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v[v.len() / 2]
    }
    pub fn is_fresh(&self) -> bool {
        self.count >= 5
    }
}

// Raw sensor cache. Owned by Agent; updated each cycle from CiberMouse.
pub struct SensorCache {
    pub ir: [f64; 4],
    pub compass: f64,
    pub ground: i32,
    pub bumper: bool,
    pub compass_ready: bool,
    pub ir_ready: bool,
    pub compass_fresh: bool,
    pub gps_x: f64,
    pub gps_y: f64,
    pub gps_ready: bool,
    pub filters: [IrFilter; 4],
}

impl SensorCache {
    pub fn new() -> Self {
        Self {
            ir: [0.0; 4],
            compass: 0.0,
            ground: -1,
            bumper: false,
            compass_ready: false,
            ir_ready: false,
            compass_fresh: false,
            gps_x: 0.0,
            gps_y: 0.0,
            gps_ready: false,
            filters: [IrFilter::new(), IrFilter::new(), IrFilter::new(), IrFilter::new()],
        }
    }
}
