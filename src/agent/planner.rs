use super::dir::Dir;
use super::map::{DynMap, bfs_first_step};

// Next move toward the closest frontier cell, or None if none reachable.
pub fn explore_next(map: &DynMap, pos: (i32, i32)) -> Option<Dir> {
    if map.frontier.is_empty() {
        return None;
    }
    bfs_first_step(map, pos, |m, p| m.frontier.contains(&p))
}

// Next move toward (0,0) via any known-open edge.
pub fn return_next(map: &DynMap, pos: (i32, i32)) -> Option<Dir> {
    bfs_first_step(map, pos, |_m, p| p == (0, 0))
}
