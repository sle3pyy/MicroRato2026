use super::dir::Dir;
use super::map::{DynMap, bfs_first_step, dijkstra_path};

// Next move toward the closest frontier cell; prefers straight (heading) over turns.
pub fn explore_next(map: &DynMap, pos: (i32, i32), heading: Dir) -> Option<Dir> {
    if map.frontier.is_empty() {
        return None;
    }
    bfs_first_step(map, pos, |m, p| m.frontier.contains(&p), Some(heading))
}

// Next move toward (0,0) via any known-open edge.
pub fn return_next(map: &DynMap, pos: (i32, i32)) -> Option<Dir> {
    bfs_first_step(map, pos, |_m, p| p == (0, 0), None)
}

// Full turn-minimising path from start to goal over the known map.
// Empty if start==goal or no path exists.
pub fn plan_speedrun(map: &DynMap, start: (i32, i32), heading: Dir, goal: (i32, i32)) -> Vec<Dir> {
    dijkstra_path(map, start, heading, goal)
}
