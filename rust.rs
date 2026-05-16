use std::collections::{HashMap, VecDeque};
use std::time::Instant;

// ==================== 常量定义 ====================
const GRIDSIZE: usize = 8;
const OBSTACLE: i32 = 2;
const GRID_BLACK: i32 = 1;
const GRID_WHITE: i32 = -1;
const MAX_DEPTH: usize = 20;
const TIME_LIMIT: f64 = 0.80;
const TT_SIZE: usize = 1 << 20;

// 方向偏移
const DX: [i32; 8] = [-1, -1, -1, 0, 0, 1, 1, 1];
const DY: [i32; 8] = [-1, 0, 1, -1, 1, -1, 0, 1];

// Q-Learning参数
const Q_ALPHA: f64 = 0.25;
const Q_GAMMA: f64 = 0.90;
const Q_LAMBDA: f64 = 0.12;
const Q_EPSILON: f64 = 0.05;

// ==================== 简单的伪随机数生成器 ====================
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        SimpleRng { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }
}

// ==================== 数据结构 ====================
#[derive(Copy, Clone, Debug, PartialEq)]
struct Point {
    x: i32,
    y: i32,
}

impl Point {
    fn new(x: i32, y: i32) -> Self {
        Point { x, y }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct Move {
    initgrid: Point,
    newgrid: Point,
    arows: Point,
}

impl Move {
    fn new(initgrid: Point, newgrid: Point, arows: Point) -> Self {
        Move {
            initgrid,
            newgrid,
            arows,
        }
    }
}

#[derive(Copy, Clone)]
struct TTEntry {
    key: u64,
    depth: usize,
    value: f64,
    flag: i32,
    best_move: Move,
}

impl TTEntry {
    fn new() -> Self {
        TTEntry {
            key: 0,
            depth: 0,
            value: 0.0,
            flag: 0,
            best_move: Move::new(Point::new(0, 0), Point::new(0, 0), Point::new(0, 0)),
        }
    }
}

struct GameState {
    stop_searching: bool,
    curr_bot_color: i32,
    grid_info: [[i32; GRIDSIZE]; GRIDSIZE],
    zobrist_table: [[[u64; 4]; GRIDSIZE]; GRIDSIZE],
    current_hash: u64,
    history_from: [[[i32; GRIDSIZE]; GRIDSIZE]; 2],
    history_to: [[[i32; GRIDSIZE]; GRIDSIZE]; 2],
    tt: Vec<TTEntry>,
    q_table: HashMap<String, f64>,
    start_time: Option<Instant>,
}

impl GameState {
    fn new() -> Self {
        GameState {
            stop_searching: false,
            curr_bot_color: GRID_WHITE,
            grid_info: [[0; GRIDSIZE]; GRIDSIZE],
            zobrist_table: [[[0; 4]; GRIDSIZE]; GRIDSIZE],
            current_hash: 0,
            history_from: [[[0; GRIDSIZE]; GRIDSIZE]; 2],
            history_to: [[[0; GRIDSIZE]; GRIDSIZE]; 2],
            tt: vec![TTEntry::new(); TT_SIZE],
            q_table: HashMap::new(),
            start_time: None,
        }
    }

    fn start_timer(&mut self) {
        self.start_time = Some(Instant::now());
    }

    fn time_is_up(&self) -> bool {
        if let Some(start) = self.start_time {
            start.elapsed().as_secs_f64() > TIME_LIMIT
        } else {
            false
        }
    }

    fn init_zobrist(&mut self) {
        let mut rng = SimpleRng::new(12345);
        for i in 0..GRIDSIZE {
            for j in 0..GRIDSIZE {
                for k in 0..4 {
                    self.zobrist_table[i][j][k] = rng.next_u64();
                }
            }
        }
    }

    fn in_map(&self, x: i32, y: i32) -> bool {
        x >= 0 && x < GRIDSIZE as i32 && y >= 0 && y < GRIDSIZE as i32
    }

    fn legal_step(&self, x: i32, y: i32) -> bool {
        if !self.in_map(x, y) {
            return false;
        }
        let val = self.grid_info[x as usize][y as usize];
        val != OBSTACLE && val != GRID_BLACK && val != GRID_WHITE
    }

    fn proc_step(
        &mut self,
        x0: i32,
        y0: i32,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        color: i32,
        check_only: bool,
    ) -> bool {
        if !self.in_map(x0, y0) || !self.in_map(x1, y1) || !self.in_map(x2, y2) {
            return false;
        }

        let x0u = x0 as usize;
        let y0u = y0 as usize;
        let x1u = x1 as usize;
        let y1u = y1 as usize;
        let x2u = x2 as usize;
        let y2u = y2 as usize;

        if self.grid_info[x0u][y0u] != color || self.grid_info[x1u][y1u] != 0 {
            return false;
        }

        if self.grid_info[x2u][y2u] != 0 && !(x2 == x0 && y2 == y0) {
            return false;
        }

        if !check_only {
            self.grid_info[x0u][y0u] = 0;
            self.grid_info[x1u][y1u] = color;
            self.grid_info[x2u][y2u] = OBSTACLE;
        }

        true
    }

    fn get_move_pos(&self, cu_point: Point) -> Vec<Point> {
        let mut vecpo = Vec::new();
        for d in 0..8 {
            let mut px = cu_point.x;
            let mut py = cu_point.y;
            loop {
                px += DX[d];
                py += DY[d];
                if self.legal_step(px, py) {
                    vecpo.push(Point::new(px, py));
                } else {
                    break;
                }
            }
        }
        vecpo
    }

    fn get_arrow_pos(&self, grid: &[[i32; GRIDSIZE]; GRIDSIZE], newpoint: Point) -> Vec<Point> {
        let mut vecpo = Vec::new();
        for d in 0..8 {
            let mut px = newpoint.x;
            let mut py = newpoint.y;
            loop {
                px += DX[d];
                py += DY[d];
                if self.in_map(px, py) {
                    let val = grid[px as usize][py as usize];
                    if val != OBSTACLE && val != GRID_BLACK && val != GRID_WHITE {
                        vecpo.push(Point::new(px, py));
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
        }
        vecpo
    }

    fn get_valid_moves(&self, color: i32, tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> Vec<Move> {
        let mut all_valid_moves = Vec::new();
        let mut starts = Vec::new();

        for x in 0..GRIDSIZE as i32 {
            for y in 0..GRIDSIZE as i32 {
                if tpgrid[x as usize][y as usize] == color {
                    starts.push(Point::new(x, y));
                }
            }
        }

        for p in starts {
            let move_positions = self.get_move_pos(p);

            for new_p in move_positions {
                let mut temp_grid = *tpgrid;
                temp_grid[p.x as usize][p.y as usize] = 0;
                temp_grid[new_p.x as usize][new_p.y as usize] = color;

                let arrows_position = self.get_arrow_pos(&temp_grid, new_p);

                for arrow in arrows_position {
                    if arrow.x != new_p.x || arrow.y != new_p.y {
                        all_valid_moves.push(Move::new(p, new_p, arrow));
                    }
                }
            }
        }

        all_valid_moves
    }

    fn get_piece_index(&self, val: i32) -> usize {
        match val {
            0 => 0,
            v if v == GRID_BLACK => 1,
            v if v == GRID_WHITE => 2,
            v if v == OBSTACLE => 3,
            _ => 0,
        }
    }

    fn compute_initial_hash(&mut self) {
        let mut h = 0u64;
        for i in 0..GRIDSIZE {
            for j in 0..GRIDSIZE {
                let piece_idx = self.get_piece_index(self.grid_info[i][j]);
                if piece_idx != 0 {
                    h ^= self.zobrist_table[i][j][piece_idx];
                }
            }
        }
        self.current_hash = h;
    }

    fn update_hash(&self, old_hash: u64, x0: i32, y0: i32, x1: i32, y1: i32, x2: i32, y2: i32, color: i32) -> u64 {
        let mut new_hash = old_hash;
        let x0u = x0 as usize;
        let y0u = y0 as usize;
        let x1u = x1 as usize;
        let y1u = y1 as usize;
        let x2u = x2 as usize;
        let y2u = y2 as usize;

        new_hash ^= self.zobrist_table[x0u][y0u][self.get_piece_index(color)];
        new_hash ^= self.zobrist_table[x1u][y1u][self.get_piece_index(color)];
        new_hash ^= self.zobrist_table[x2u][y2u][self.get_piece_index(OBSTACLE)];
        new_hash
    }

    fn encode_move(&self, m: &Move) -> String {
        format!(
            "{},{}->{},{}|{},{}",
            m.initgrid.x, m.initgrid.y, m.newgrid.x, m.newgrid.y, m.arows.x, m.arows.y
        )
    }

    fn state_action_key(&self, state_hash: u64, m: &Move) -> String {
        format!("{}|{}", state_hash, self.encode_move(m))
    }

    fn get_q_value(&self, state_hash: u64, m: &Move) -> f64 {
        let key = self.state_action_key(state_hash, m);
        self.q_table.get(&key).copied().unwrap_or(0.0)
    }

    fn update_q_value(&mut self, state_hash: u64, m: &Move, reward: f64, next_best_q: f64) {
        let key = self.state_action_key(state_hash, m);
        let old_q = self.q_table.get(&key).copied().unwrap_or(0.0);
        let target = reward + Q_GAMMA * next_best_q;
        let new_q = old_q + Q_ALPHA * (target - old_q);
        self.q_table.insert(key, new_q);
    }

    fn center_score(&self, p: &Point) -> i32 {
        7 - ((p.x - 3).abs() + (p.y - 3).abs())
    }

    fn opening_formation_bonus(
        &self,
        m: &Move,
        color: i32,
        turn_id: usize,
        tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE],
    ) -> i32 {
        if turn_id > 12 {
            return 0;
        }

        let mut bonus = 0;
        let targets = [
            Point::new(1, 1),
            Point::new(1, 6),
            Point::new(6, 1),
            Point::new(6, 6),
        ];

        let mut best_before = 100;
        let mut best_after = 100;
        for target in &targets {
            let db = (m.initgrid.x - target.x).abs() + (m.initgrid.y - target.y).abs();
            let da = (m.newgrid.x - target.x).abs() + (m.newgrid.y - target.y).abs();
            best_before = best_before.min(db);
            best_after = best_after.min(da);
        }
        bonus += (best_before - best_after) * 16;

        let center_dist = (m.newgrid.x - 3).abs() + (m.newgrid.y - 3).abs();
        if center_dist <= 3 {
            bonus += 6;
        }

        let forward = if color == GRID_WHITE { -1 } else { 1 };
        if (m.newgrid.y - m.initgrid.y) * forward > 0 {
            bonus += 6;
        }

        let arrow_dist_center = (m.arows.x - 3).abs() + (m.arows.y - 3).abs();
        if arrow_dist_center <= 3 {
            bonus += 4;
        }

        for d in 0..8 {
            let nx = m.arows.x + DX[d];
            let ny = m.arows.y + DY[d];
            if self.in_map(nx, ny) && tpgrid[nx as usize][ny as usize] == color {
                bonus += 2;
            }
        }
        bonus
    }

    fn quick_move_score(
        &self,
        m: &Move,
        color: i32,
        _turn_id: usize,
        tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE],
    ) -> i32 {
        let mut score = 0;
        score += self.center_score(&m.newgrid) * 6;
        score += self.center_score(&m.arows) * 2;

        let opp = -color;
        for d in 0..8 {
            let nx = m.arows.x + DX[d];
            let ny = m.arows.y + DY[d];
            if self.in_map(nx, ny) && tpgrid[nx as usize][ny as usize] == opp {
                score += 8;
            }
        }
        score
    }

    fn reachable_count_from_queen(&self, q: &Point, tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> i32 {
        let mut count = 0;
        for d in 0..8 {
            let mut x = q.x + DX[d];
            let mut y = q.y + DY[d];
            while self.in_map(x, y) && tpgrid[x as usize][y as usize] == 0 {
                count += 1;
                x += DX[d];
                y += DY[d];
            }
        }
        count
    }

    fn compute_distances(
        &self,
        color: i32,
        tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE],
        dist_map: &mut [[i32; GRIDSIZE]; GRIDSIZE],
        mode: usize,
    ) {
        for i in 0..GRIDSIZE {
            for j in 0..GRIDSIZE {
                dist_map[i][j] = 100;
            }
        }

        let mut q = VecDeque::new();
        for i in 0..GRIDSIZE {
            for j in 0..GRIDSIZE {
                if tpgrid[i][j] == color {
                    dist_map[i][j] = 0;
                    q.push_back((i as i32, j as i32));
                }
            }
        }

        while let Some((curr_x, curr_y)) = q.pop_front() {
            let curr_xu = curr_x as usize;
            let curr_yu = curr_y as usize;
            let next_dist = dist_map[curr_xu][curr_yu] + 1;

            for d in 0..8 {
                let mut nx = curr_x + DX[d];
                let mut ny = curr_y + DY[d];

                if mode == 1 {
                    while self.in_map(nx, ny) && tpgrid[nx as usize][ny as usize] == 0 {
                        let nxu = nx as usize;
                        let nyu = ny as usize;
                        if dist_map[nxu][nyu] > next_dist {
                            dist_map[nxu][nyu] = next_dist;
                            q.push_back((nx, ny));
                        } else if dist_map[nxu][nyu] < next_dist {
                            break;
                        }
                        nx += DX[d];
                        ny += DY[d];
                    }
                } else {
                    if self.in_map(nx, ny) && tpgrid[nx as usize][ny as usize] == 0 {
                        let nxu = nx as usize;
                        let nyu = ny as usize;
                        if dist_map[nxu][nyu] > dist_map[curr_xu][curr_yu] + 1 {
                            dist_map[nxu][nyu] = dist_map[curr_xu][curr_yu] + 1;
                            q.push_back((nx, ny));
                        }
                    }
                }
            }
        }
    }

    fn get_mobility_score(&self, color: i32, tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> f64 {
        let mut total_reach = 0;
        let mut min_reach = 9999;
        let mut queen_cnt = 0;

        for x in 0..GRIDSIZE {
            for y in 0..GRIDSIZE {
                if tpgrid[x][y] != color {
                    continue;
                }
                queen_cnt += 1;
                let reach = self.reachable_count_from_queen(&Point::new(x as i32, y as i32), tpgrid);
                total_reach += reach;
                min_reach = min_reach.min(reach);
            }
        }

        if queen_cnt == 0 || min_reach == 9999 {
            min_reach = 0;
        }
        total_reach as f64 + min_reach as f64
    }

    fn enclosure_territory_score(
        &self,
        tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE],
        white_dist_q: &[[i32; GRIDSIZE]; GRIDSIZE],
        black_dist_q: &[[i32; GRIDSIZE]; GRIDSIZE],
    ) -> i32 {
        let mut vis = [[false; GRIDSIZE]; GRIDSIZE];
        let mut score = 0;

        for sx in 0..GRIDSIZE {
            for sy in 0..GRIDSIZE {
                if tpgrid[sx][sy] != 0 || vis[sx][sy] {
                    continue;
                }

                let mut q = VecDeque::new();
                q.push_back(Point::new(sx as i32, sy as i32));
                vis[sx][sy] = true;

                let mut area = 0;
                let mut white_reach = false;
                let mut black_reach = false;

                while let Some(u) = q.pop_front() {
                    area += 1;
                    let ux = u.x as usize;
                    let uy = u.y as usize;

                    if white_dist_q[ux][uy] < 100 {
                        white_reach = true;
                    }
                    if black_dist_q[ux][uy] < 100 {
                        black_reach = true;
                    }

                    for d in 0..8 {
                        let nx = u.x + DX[d];
                        let ny = u.y + DY[d];
                        if !self.in_map(nx, ny) {
                            continue;
                        }
                        let nxu = nx as usize;
                        let nyu = ny as usize;
                        if tpgrid[nxu][nyu] != 0 || vis[nxu][nyu] {
                            continue;
                        }
                        vis[nxu][nyu] = true;
                        q.push_back(Point::new(nx, ny));
                    }
                }

                if white_reach && !black_reach {
                    score += area;
                } else if black_reach && !white_reach {
                    score -= area;
                }
            }
        }
        score
    }

    fn evaluate(&self, tpgrid: &[[i32; GRIDSIZE]; GRIDSIZE], turn_id: usize) -> f64 {
        let mut white_dist_q = [[100; GRIDSIZE]; GRIDSIZE];
        let mut black_dist_q = [[100; GRIDSIZE]; GRIDSIZE];

        self.compute_distances(GRID_WHITE, tpgrid, &mut white_dist_q, 1);
        self.compute_distances(GRID_BLACK, tpgrid, &mut black_dist_q, 1);

        let mut t1 = 0.0;
        let mut p1 = 0.0;

        for i in 0..GRIDSIZE {
            for j in 0..GRIDSIZE {
                if tpgrid[i][j] != 0 {
                    continue;
                }

                if white_dist_q[i][j] < black_dist_q[i][j] {
                    t1 += 1.0;
                } else if white_dist_q[i][j] > black_dist_q[i][j] {
                    t1 -= 1.0;
                }

                p1 += (2f64).powf(-white_dist_q[i][j] as f64) - (2f64).powf(-black_dist_q[i][j] as f64);
            }
        }

        let m = self.get_mobility_score(GRID_WHITE, tpgrid) - self.get_mobility_score(GRID_BLACK, tpgrid);

        let (a, c, e) = if turn_id <= 20 {
            (0.30, 0.40, 0.30)
        } else if turn_id <= 49 {
            (0.40, 0.35, 0.25)
        } else {
            (0.70, 0.20, 0.10)
        };

        let score = a * t1 + c * p1 + e * m;

        if self.curr_bot_color == GRID_WHITE {
            score
        } else {
            -score
        }
    }

    fn get_newgrid(&self, tmpgrid: &mut [[i32; GRIDSIZE]; GRIDSIZE], m: &Move, color: i32) {
        let start_x = m.initgrid.x as usize;
        let start_y = m.initgrid.y as usize;
        let new_x = m.newgrid.x as usize;
        let new_y = m.newgrid.y as usize;
        let arrow_x = m.arows.x as usize;
        let arrow_y = m.arows.y as usize;

        tmpgrid[start_x][start_y] = 0;
        tmpgrid[new_x][new_y] = color;
        tmpgrid[arrow_x][arrow_y] = OBSTACLE;
    }

    fn is_same_move(&self, m1: &Move, m2: &Move) -> bool {
        m1.initgrid == m2.initgrid && m1.newgrid == m2.newgrid && m1.arows == m2.arows
    }

    fn terminal_score(&self, is_max_player: bool) -> f64 {
        if is_max_player {
            -1e8
        } else {
            1e8
        }
    }

    fn minmax(
        &mut self,
        grid: &[[i32; GRIDSIZE]; GRIDSIZE],
        depth: usize,
        is_max: bool,
        turn_id: usize,
        mut alpha: f64,
        mut beta: f64,
        h: u64,
    ) -> f64 {
        if self.stop_searching || self.time_is_up() {
            self.stop_searching = true;
            return self.evaluate(grid, turn_id);
        }

        let index = (h & ((TT_SIZE - 1) as u64)) as usize;
        let mut best_move_from_tt = Move::new(Point::new(0, 0), Point::new(0, 0), Point::new(0, 0));
        let mut has_best_move_from_tt = false;
        let alpha_orig = alpha;
        let beta_orig = beta;

        if self.tt[index].key == h {
            best_move_from_tt = self.tt[index].best_move;
            has_best_move_from_tt = true;

            if self.tt[index].depth >= depth {
                if self.tt[index].flag == 0 {
                    return self.tt[index].value;
                }
                if self.tt[index].flag == 1 && self.tt[index].value >= beta {
                    return self.tt[index].value;
                }
                if self.tt[index].flag == 2 && self.tt[index].value <= alpha {
                    return self.tt[index].value;
                }
            }
        }

        if depth == 0 {
            return self.evaluate(grid, turn_id);
        }

        let mut best_move_in_this_node = Move::new(Point::new(0, 0), Point::new(0, 0), Point::new(0, 0));
        let mut found_best = false;

        if is_max {
            let mut moves = self.get_valid_moves(self.curr_bot_color, grid);
            if moves.is_empty() {
                return self.terminal_score(true);
            }

            let mut current_max = -1e9;

            moves.sort_by(|a, b| {
                let cidx = if self.curr_bot_color == GRID_BLACK { 0 } else { 1 };
                let ha = self.history_from[cidx][a.initgrid.x as usize][a.initgrid.y as usize]
                    + self.history_to[cidx][a.newgrid.x as usize][a.newgrid.y as usize];
                let hb = self.history_from[cidx][b.initgrid.x as usize][b.initgrid.y as usize]
                    + self.history_to[cidx][b.newgrid.x as usize][b.newgrid.y as usize];

                hb.cmp(&ha)
            });

            if has_best_move_from_tt {
                for i in 0..moves.len() {
                    if self.is_same_move(&moves[i], &best_move_from_tt) {
                        moves.swap(0, i);
                        break;
                    }
                }
            }

            let mut first_move = true;
            for m in &moves {
                let mut tmpgrid = *grid;
                self.get_newgrid(&mut tmpgrid, m, self.curr_bot_color);

                let next_h = self.update_hash(h, m.initgrid.x, m.initgrid.y, m.newgrid.x, m.newgrid.y, m.arows.x, m.arows.y, self.curr_bot_color);

                let tpscore = if first_move {
                    self.minmax(&tmpgrid, depth - 1, false, turn_id, alpha, beta, next_h)
                } else {
                    let mut score = self.minmax(&tmpgrid, depth - 1, false, turn_id, alpha, alpha + 0.01, next_h);
                    if score > alpha && score < beta {
                        score = self.minmax(&tmpgrid, depth - 1, false, turn_id, alpha, beta, next_h);
                    }
                    score
                };
                first_move = false;

                if tpscore > current_max {
                    current_max = tpscore;
                    best_move_in_this_node = *m;
                    found_best = true;
                }

                if current_max > beta {
                    let cidx = if self.curr_bot_color == GRID_BLACK { 0 } else { 1 };
                    self.history_from[cidx][m.initgrid.x as usize][m.initgrid.y as usize] += (depth * depth) as i32;
                    self.history_to[cidx][m.newgrid.x as usize][m.newgrid.y as usize] += (depth * depth) as i32;
                    break;
                }
                if current_max > alpha {
                    alpha = current_max;
                }
            }

            if self.tt[index].key != h || depth >= self.tt[index].depth {
                self.tt[index].key = h;
                self.tt[index].depth = depth;
                self.tt[index].value = current_max;
                if found_best {
                    self.tt[index].best_move = best_move_in_this_node;
                }
                if current_max <= alpha_orig {
                    self.tt[index].flag = 2;
                } else if current_max >= beta_orig {
                    self.tt[index].flag = 1;
                } else {
                    self.tt[index].flag = 0;
                }
            }
            current_max
        } else {
            let mut moves = self.get_valid_moves(-self.curr_bot_color, grid);
            if moves.is_empty() {
                return self.terminal_score(false);
            }

            let mut current_min = 1e9;

            moves.sort_by(|a, b| {
                let opp_color = -self.curr_bot_color;
                let cidx = if opp_color == GRID_BLACK { 0 } else { 1 };
                let ha = self.history_from[cidx][a.initgrid.x as usize][a.initgrid.y as usize]
                    + self.history_to[cidx][a.newgrid.x as usize][a.newgrid.y as usize];
                let hb = self.history_from[cidx][b.initgrid.x as usize][b.initgrid.y as usize]
                    + self.history_to[cidx][b.newgrid.x as usize][b.newgrid.y as usize];

                hb.cmp(&ha)
            });

            if has_best_move_from_tt {
                for i in 0..moves.len() {
                    if self.is_same_move(&moves[i], &best_move_from_tt) {
                        moves.swap(0, i);
                        break;
                    }
                }
            }

            let mut first_move = true;
            for m in &moves {
                let mut tmpgrid = *grid;
                self.get_newgrid(&mut tmpgrid, m, -self.curr_bot_color);

                let next_h = self.update_hash(h, m.initgrid.x, m.initgrid.y, m.newgrid.x, m.newgrid.y, m.arows.x, m.arows.y, -self.curr_bot_color);

                let tpscore = if first_move {
                    self.minmax(&tmpgrid, depth - 1, true, turn_id, alpha, beta, next_h)
                } else {
                    let mut score = self.minmax(&tmpgrid, depth - 1, true, turn_id, beta - 0.01, beta, next_h);
                    if score < beta && score > alpha {
                        score = self.minmax(&tmpgrid, depth - 1, true, turn_id, alpha, beta, next_h);
                    }
                    score
                };
                first_move = false;

                if tpscore < current_min {
                    current_min = tpscore;
                    best_move_in_this_node = *m;
                    found_best = true;
                }

                if current_min < alpha {
                    let opp_color = -self.curr_bot_color;
                    let cidx = if opp_color == GRID_BLACK { 0 } else { 1 };
                    self.history_from[cidx][m.initgrid.x as usize][m.initgrid.y as usize] += (depth * depth) as i32;
                    self.history_to[cidx][m.newgrid.x as usize][m.newgrid.y as usize] += (depth * depth) as i32;
                    break;
                }
                if current_min < beta {
                    beta = current_min;
                }
            }

            if self.tt[index].key != h || depth >= self.tt[index].depth {
                self.tt[index].key = h;
                self.tt[index].depth = depth;
                self.tt[index].value = current_min;
                if found_best {
                    self.tt[index].best_move = best_move_in_this_node;
                }
                if current_min <= alpha_orig {
                    self.tt[index].flag = 2;
                } else if current_min >= beta_orig {
                    self.tt[index].flag = 1;
                } else {
                    self.tt[index].flag = 0;
                }
            }
            current_min
        }
    }

    fn mtdf(&mut self, h: u64, f: f64, depth: usize, turn_id: usize) -> f64 {
        let mut g = f;
        let mut upper_bound = 1e9;
        let mut lower_bound = -1e9;

        while lower_bound < upper_bound - 0.001 {
            let beta = if g == lower_bound { g + 0.01 } else { g };
            g = self.minmax(&self.grid_info.clone(), depth, true, turn_id, beta - 0.01, beta, h);
            if self.stop_searching {
                break;
            }
            if g < beta {
                upper_bound = g;
            } else {
                lower_bound = g;
            }
        }
        g
    }
}

fn main() {
    use std::io::{self, BufRead};

    let mut game = GameState::new();

    game.grid_info[0][2] = GRID_BLACK;
    game.grid_info[2][0] = GRID_BLACK;
    game.grid_info[5][0] = GRID_BLACK;
    game.grid_info[7][2] = GRID_BLACK;

    game.grid_info[0][5] = GRID_WHITE;
    game.grid_info[2][7] = GRID_WHITE;
    game.grid_info[5][7] = GRID_WHITE;
    game.grid_info[7][5] = GRID_WHITE;

    game.init_zobrist();
    game.compute_initial_hash();

    let stdin = io::stdin();
    let mut reader = stdin.lock();
    let mut line = String::new();
    reader.read_line(&mut line).unwrap();
    let turn_id: usize = line.trim().parse().unwrap_or(0);

    game.curr_bot_color = GRID_WHITE;

    for i in 0..turn_id {
        line.clear();
        reader.read_line(&mut line).unwrap();
        let coords: Vec<i32> = line
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        if coords.len() >= 6 {
            let (x0, y0, x1, y1, x2, y2) = (coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]);

            if x0 == -1 {
                game.curr_bot_color = GRID_BLACK;
            } else {
                game.proc_step(x0, y0, x1, y1, x2, y2, -game.curr_bot_color, false);
                game.current_hash = game.update_hash(game.current_hash, x0, y0, x1, y1, x2, y2, -game.curr_bot_color);
            }
        }

        if i < turn_id - 1 {
            line.clear();
            reader.read_line(&mut line).unwrap();
            let coords: Vec<i32> = line
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if coords.len() >= 6 {
                let (x0, y0, x1, y1, x2, y2) = (coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]);

                if x0 >= 0 {
                    game.proc_step(x0, y0, x1, y1, x2, y2, game.curr_bot_color, false);
                    game.current_hash = game.update_hash(game.current_hash, x0, y0, x1, y1, x2, y2, game.curr_bot_color);
                }
            }
        }
    }

    game.start_timer();
    game.stop_searching = false;
    let root_moves = game.get_valid_moves(game.curr_bot_color, &game.grid_info);
    let mut overall_best_move = if root_moves.is_empty() {
        Move::new(Point::new(0, 0), Point::new(0, 0), Point::new(0, 0))
    } else {
        root_moves[0]
    };
    let mut last_score = 0.0;

    for d in 1..=MAX_DEPTH {
        if d > 1 {
            for c in 0..2 {
                for i in 0..GRIDSIZE {
                    for j in 0..GRIDSIZE {
                        game.history_from[c][i][j] = game.history_from[c][i][j] * 8 / 10;
                        game.history_to[c][i][j] = game.history_to[c][i][j] * 8 / 10;
                    }
                }
            }
        }

        let current_score = game.mtdf(game.current_hash, last_score, d, turn_id);

        if game.stop_searching {
            break;
        }
        last_score = current_score;

        let index = (game.current_hash & ((TT_SIZE - 1) as u64)) as usize;
        if game.tt[index].key == game.current_hash {
            overall_best_move = game.tt[index].best_move;
        }
    }

    let mut legal_best = false;
    for m in &root_moves {
        if game.is_same_move(m, &overall_best_move) {
            legal_best = true;
            break;
        }
    }
    if !legal_best && !root_moves.is_empty() {
        overall_best_move = root_moves[0];
    }

    if !root_moves.is_empty() {
        let mut next_grid = game.grid_info;
        game.get_newgrid(&mut next_grid, &overall_best_move, game.curr_bot_color);
        let mut next_best_q: f64 = 0.0;
        let next_moves = game.get_valid_moves(-game.curr_bot_color, &next_grid);
        for nm in &next_moves {
            let next_hash = game.update_hash(
                game.current_hash,
                overall_best_move.initgrid.x,
                overall_best_move.initgrid.y,
                overall_best_move.newgrid.x,
                overall_best_move.newgrid.y,
                overall_best_move.arows.x,
                overall_best_move.arows.y,
                game.curr_bot_color,
            );
            next_best_q = next_best_q.max(game.get_q_value(next_hash, nm));
        }
        game.update_q_value(game.current_hash, &overall_best_move, last_score, next_best_q);

        let mut rng = SimpleRng::new(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_nanos() as u64);
        let rand_val = rng.next_u64() as f64 / (u64::MAX as f64);
        if rand_val < Q_EPSILON {
            let idx = (rng.next_u64() as usize) % root_moves.len();
            overall_best_move = root_moves[idx];
        } else {
            let mut best_blend = -1e18;
            let mut blended_best = overall_best_move;
            for rm in &root_moves {
                let mut tmpgrid = game.grid_info;
                game.get_newgrid(&mut tmpgrid, rm, game.curr_bot_color);
                let eval_score = game.evaluate(&tmpgrid, turn_id);
                let blend = eval_score + Q_LAMBDA * game.get_q_value(game.current_hash, rm);
                if blend > best_blend {
                    best_blend = blend;
                    blended_best = *rm;
                }
            }
            overall_best_move = blended_best;
        }
    }

    println!(
        "{} {} {} {} {} {}",
        overall_best_move.initgrid.x,
        overall_best_move.initgrid.y,
        overall_best_move.newgrid.x,
        overall_best_move.newgrid.y,
        overall_best_move.arows.x,
        overall_best_move.arows.y
    );
}
