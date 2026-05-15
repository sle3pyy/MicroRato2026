use super::dir::{Dir, neighbor};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

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
// prefer: when Some(d), try d first at each node so BFS naturally picks
// straight paths over turns when distances are equal.
pub fn bfs_first_step(
    map: &DynMap,
    start: (i32, i32),
    goal: impl Fn(&DynMap, (i32, i32)) -> bool,
    prefer: Option<Dir>,
) -> Option<Dir> {
    if goal(map, start) {
        return None;
    }
    let mut came_first: HashMap<(i32, i32), Dir> = HashMap::new();
    let mut seen: HashSet<(i32, i32)> = HashSet::new();
    let mut queue: VecDeque<(i32, i32)> = VecDeque::new();
    queue.push_back(start);
    seen.insert(start);

    // Direction order: preferred first, then the rest in natural order.
    let dir_order: [usize; 4] = match prefer {
        Some(p) => {
            let pi = p as usize;
            let mut arr = [0usize; 4];
            arr[0] = pi;
            let mut j = 1;
            for i in 0..4 {
                if i != pi { arr[j] = i; j += 1; }
            }
            arr
        }
        None => [0, 1, 2, 3],
    };

    while let Some(pos) = queue.pop_front() {
        for &i in &dir_order {
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

// Dijkstra on (pos, heading) state space.  Minimises actual cycle cost:
//   straight move ≈ DRIVE_CYCLES+SETTLE_CYCLES=31, 90° turn ≈ TURN_MAX_CYCLES=36.
// Traverses only visited cells (plus the goal), so call after exploration done.
// Returns the full sequence of movement directions (one per cell hop).
pub fn dijkstra_path(
    map: &DynMap,
    start: (i32, i32),
    start_heading: Dir,
    goal: (i32, i32),
) -> Vec<Dir> {
    if start == goal {
        return vec![];
    }

    const MOVE: u32 = 31;
    const TURN90: u32 = 36;

    let turn_extra = |from: u8, to: u8| -> u32 {
        match (to + 4 - from) % 4 {
            0 => 0,
            2 => 2 * TURN90,
            _ => TURN90,
        }
    };

    let sh = start_heading as u8;
    let mut dist: HashMap<((i32, i32), u8), u32> = HashMap::new();
    let mut prev: HashMap<((i32, i32), u8), ((i32, i32), u8, Dir)> = HashMap::new();
    let mut heap: BinaryHeap<Reverse<(u32, i32, i32, u8)>> = BinaryHeap::new();

    dist.insert((start, sh), 0);
    heap.push(Reverse((0, start.0, start.1, sh)));

    while let Some(Reverse((cost, r, c, h))) = heap.pop() {
        let pos = (r, c);
        if dist.get(&(pos, h)).map_or(true, |&d| cost > d) {
            continue;
        }
        for i in 0u8..4 {
            let dir = Dir::from_index(i as usize);
            if map.has_wall(pos, dir) {
                continue;
            }
            let nb = neighbor(pos, dir);
            if !map.visited.contains(&nb) && nb != goal {
                continue;
            }
            let new_cost = cost + MOVE + turn_extra(h, i);
            let state = (nb, i);
            if new_cost < *dist.get(&state).unwrap_or(&u32::MAX) {
                dist.insert(state, new_cost);
                prev.insert(state, (pos, h, dir));
                heap.push(Reverse((new_cost, nb.0, nb.1, i)));
            }
        }
    }

    let Some((_, end_h)) = (0u8..4)
        .filter_map(|h| dist.get(&(goal, h)).map(|&c| (c, h)))
        .min_by_key(|&(c, _)| c)
    else {
        return vec![];
    };

    let mut path: Vec<Dir> = Vec::new();
    let mut cur = (goal, end_h);
    while cur.0 != start {
        let &(par_pos, par_h, dir) = prev.get(&cur).unwrap();
        path.push(dir);
        cur = (par_pos, par_h);
    }
    path.reverse();
    path
}
