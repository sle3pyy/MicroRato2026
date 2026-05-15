use super::dir::{Dir, neighbor};
use std::collections::{HashMap, HashSet, VecDeque};

// Sparse map in relative coords; origin (0,0) is start cell.
pub struct DynMap {
    pub walls: HashMap<(i32, i32), u8>,
    pub visited: HashSet<(i32, i32)>,
    pub frontier: HashSet<(i32, i32)>,
}

impl DynMap {
    pub fn new() -> Self {
        Self {
            walls: HashMap::new(),
            visited: HashSet::new(),
            frontier: HashSet::new(),
        }
    }

    pub fn record_wall(&mut self, pos: (i32, i32), dir: Dir) {
        *self.walls.entry(pos).or_insert(0) |= 1 << (dir as u8);
        let nb = neighbor(pos, dir);
        *self.walls.entry(nb).or_insert(0) |= 1 << (dir.opposite() as u8);
    }

    pub fn record_open(&mut self, pos: (i32, i32), dir: Dir) {
        let nb = neighbor(pos, dir);
        if !self.visited.contains(&nb) {
            self.frontier.insert(nb);
        }
        *self.walls.entry(pos).or_insert(0) &= !(1 << (dir as u8));
        *self.walls.entry(nb).or_insert(0) &= !(1 << (dir.opposite() as u8));
    }

    pub fn mark_visited(&mut self, pos: (i32, i32)) {
        self.frontier.remove(&pos);
        self.visited.insert(pos);
        self.walls.entry(pos).or_insert(0);
    }

    pub fn has_wall(&self, pos: (i32, i32), dir: Dir) -> bool {
        self.walls
            .get(&pos)
            .map_or(false, |w| w & (1 << (dir as u8)) != 0)
    }

}

// BFS over any cell connected by known-open edges. Returns the first
// direction step toward the nearest cell satisfying `goal`, or None.
// Traverses through frontier cells too (their walls are unknown but the edge
// reaching them was confirmed open), so this handles both:
//   - return-to-start (goal: pos == (0,0)), traversing visited cells, AND
//   - frontier exploration (goal: map.frontier.contains(pos)).
pub fn bfs_first_step(
    map: &DynMap,
    start: (i32, i32),
    goal: impl Fn(&DynMap, (i32, i32)) -> bool,
) -> Option<Dir> {
    if goal(map, start) {
        return None;
    }
    let mut came_first: HashMap<(i32, i32), Dir> = HashMap::new();
    let mut seen: HashSet<(i32, i32)> = HashSet::new();
    let mut queue: VecDeque<(i32, i32)> = VecDeque::new();
    queue.push_back(start);
    seen.insert(start);

    while let Some(pos) = queue.pop_front() {
        for i in 0..4 {
            let dir = Dir::from_index(i);
            if map.has_wall(pos, dir) {
                continue;
            }
            let nb = neighbor(pos, dir);
            if seen.contains(&nb) {
                continue;
            }
            let first = if pos == start { dir } else { came_first[&pos] };
            if goal(map, nb) {
                return Some(first);
            }
            came_first.insert(nb, first);
            seen.insert(nb);
            queue.push_back(nb);
        }
    }
    None
}
