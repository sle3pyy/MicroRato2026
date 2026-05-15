use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};

pub const HEADER: &str = "cycle,state,motion,row,col,heading,compass,compass_ready,gps_x,gps_y,gps_ready,ir0,ir1,ir2,ir3,ground,bumper,l_pow,r_pow,pending_dir,pose_x,pose_y,pose_theta,pose_cov_xx,pose_cov_yy,pose_cov_tt,ir0_filt,ir1_filt,ir2_filt,ir3_filt,trav_in_drive,turn_streak,reloc_delta,event\n";

pub struct Trace {
    csv: Option<BufWriter<File>>,
}

impl Trace {
    pub fn new(enabled: bool) -> Self {
        let csv = if enabled {
            OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open("cif_trace.csv")
                .ok()
                .map(|f| {
                    let mut w = BufWriter::new(f);
                    let _ = w.write_all(HEADER.as_bytes());
                    w
                })
        } else {
            None
        };
        Self { csv }
    }

    pub fn writeln(&mut self, row: &str) {
        if let Some(w) = self.csv.as_mut() {
            let _ = writeln!(w, "{}", row);
            let _ = w.flush();
        }
    }

    pub fn event(&mut self, msg: &str) {
        if let Some(w) = self.csv.as_mut() {
            let _ = writeln!(w, "# {}", msg);
            let _ = w.flush();
        }
    }
}
