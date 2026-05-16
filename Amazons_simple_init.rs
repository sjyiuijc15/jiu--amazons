use std::cmp::{max, min};
use std::collections::VecDeque;
use std::io::{self, Read};
use std::time::Instant;

const GRIDSIZE: usize = 8;
const OBSTACLE: i32 = 2;
const GRID_BLACK: i32 = 1;
const GRID_WHITE: i32 = -1;
const MAX_DEPTH: usize = 10_000_000;
const TT_SIZE: usize = 2 << 20;

static DX: [i32; 8] = [-1, -1, -1, 0, 0, 1, 1, 1];
static DY: [i32; 8] = [-1, 0, 1, -1, 1, -1, 0, 1];

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct Point { x: usize, y: usize }

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct Move { initgrid: Point, newgrid: Point, arows: Point }

#[derive(Clone, Copy, Default)]
struct TTEntry { key: u64, depth: usize, value: f64, flag: i32, best_move: Move }

struct Engine {
    start_time: Instant,
    stop_searching: bool,
    curr_bot_color: i32,
    grid_info: [[i32; GRIDSIZE]; GRIDSIZE],
    history_from: [[[i32; GRIDSIZE]; GRIDSIZE]; 2],
    history_to: [[[i32; GRIDSIZE]; GRIDSIZE]; 2],
    tt: Vec<TTEntry>,
    zobrist: [[[u64; 4]; GRIDSIZE]; GRIDSIZE],
    current_hash: u64,
}

impl Engine {
    fn new() -> Self {
        let mut s = Self {
            start_time: Instant::now(),
            stop_searching: false,
            curr_bot_color: GRID_WHITE,
            grid_info: [[0; GRIDSIZE]; GRIDSIZE],
            history_from: [[[0; GRIDSIZE]; GRIDSIZE]; 2],
            history_to: [[[0; GRIDSIZE]; GRIDSIZE]; 2],
            tt: vec![TTEntry::default(); TT_SIZE],
            zobrist: [[[0; 4]; GRIDSIZE]; GRIDSIZE],
            current_hash: 0,
        };
        s.init_board();
        s.init_zobrist();
        s.current_hash = s.compute_initial_hash();
        s
    }
    fn init_board(&mut self) {
        self.grid_info[0][2] = GRID_BLACK; self.grid_info[2][0] = GRID_BLACK;
        self.grid_info[5][0] = GRID_BLACK; self.grid_info[7][2] = GRID_BLACK;
        self.grid_info[0][5] = GRID_WHITE; self.grid_info[2][7] = GRID_WHITE;
        self.grid_info[5][7] = GRID_WHITE; self.grid_info[7][5] = GRID_WHITE;
    }
    fn time_is_up(&self) -> bool { self.start_time.elapsed().as_secs_f64() > 0.93 }
    fn in_map(x: i32, y: i32) -> bool { x >= 0 && x < GRIDSIZE as i32 && y >= 0 && y < GRIDSIZE as i32 }
    fn legal_step(x: i32, y: i32, grid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> bool {
        if !Self::in_map(x,y) { return false; }
        grid[x as usize][y as usize] == 0
    }
    fn get_move_pos(cu: Point, grid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> Vec<Point> {
        let mut v = vec![];
        for d in 0..8 {
            let (mut px, mut py) = (cu.x as i32, cu.y as i32);
            loop {
                px += DX[d]; py += DY[d];
                if Self::legal_step(px, py, grid) { v.push(Point{ x:px as usize, y:py as usize}); } else { break; }
            }
        }
        v
    }
    fn get_arrow_pos(grid: &[[i32; GRIDSIZE]; GRIDSIZE], p: Point) -> Vec<Point> { Self::get_move_pos(p, grid) }
    fn get_valid_moves(color: i32, grid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> Vec<Move> {
        let mut starts = vec![];
        for x in 0..GRIDSIZE { for y in 0..GRIDSIZE { if grid[x][y] == color { starts.push(Point{x,y}); } } }
        let mut all = vec![];
        for p in starts {
            for new_p in Self::get_move_pos(p, grid) {
                let mut tmp = *grid;
                tmp[p.x][p.y] = 0; tmp[new_p.x][new_p.y] = color;
                for arrow in Self::get_arrow_pos(&tmp, new_p) {
                    if arrow != new_p { all.push(Move{initgrid:p,newgrid:new_p,arows:arrow}); }
                }
            }
        }
        all
    }
    fn center_score(p: Point) -> i32 { 7 - ((p.x as i32 - 3).abs() + (p.y as i32 - 3).abs()) }
    fn quick_move_score(m: Move, color: i32, grid: &[[i32; GRIDSIZE]; GRIDSIZE]) -> i32 {
        let mut score = 0;
        score += Self::center_score(m.newgrid) * 6 + Self::center_score(m.arows) * 2;
        score += max((m.newgrid.x as i32 - m.initgrid.x as i32).abs(), (m.newgrid.y as i32 - m.initgrid.y as i32).abs()) * 2;
        let opp = -color;
        for d in 0..8 {
            let nx = m.arows.x as i32 + DX[d]; let ny = m.arows.y as i32 + DY[d];
            if Self::in_map(nx, ny) && grid[nx as usize][ny as usize] == opp { score += 8; }
        }
        score
    }
    fn compute_distances(color:i32, grid:&[[i32;GRIDSIZE];GRIDSIZE], dist:&mut [[i32;GRIDSIZE];GRIDSIZE], mode:i32){
        for i in 0..GRIDSIZE { for j in 0..GRIDSIZE { dist[i][j]=100; } }
        let mut q=VecDeque::new();
        for i in 0..GRIDSIZE { for j in 0..GRIDSIZE { if grid[i][j]==color {dist[i][j]=0;q.push_back((i,j));}}}
        while let Some((x,y))=q.pop_front(){
            let next=dist[x][y]+1;
            for d in 0..8 {
                let mut nx=x as i32 + DX[d]; let mut ny=y as i32 + DY[d];
                if mode==1 { while Self::in_map(nx,ny) && grid[nx as usize][ny as usize]==0 { let (ux,uy)=(nx as usize,ny as usize); if dist[ux][uy]>next {dist[ux][uy]=next;q.push_back((ux,uy));} else if dist[ux][uy]<next {break;} nx+=DX[d]; ny+=DY[d]; } }
                else if Self::in_map(nx,ny) && grid[nx as usize][ny as usize]==0 { let (ux,uy)=(nx as usize,ny as usize); if dist[ux][uy]>dist[x][y]+1 {dist[ux][uy]=dist[x][y]+1;q.push_back((ux,uy));} }
            }
        }
    }
    fn evaluate(&self, grid:&[[i32;GRIDSIZE];GRIDSIZE], turn_id:i32)->f64{
        let (mut wq,mut bq,mut wk,mut bk)=([[0;GRIDSIZE];GRIDSIZE],[[0;GRIDSIZE];GRIDSIZE],[[0;GRIDSIZE];GRIDSIZE],[[0;GRIDSIZE];GRIDSIZE]);
        Self::compute_distances(GRID_WHITE,grid,&mut wq,1); Self::compute_distances(GRID_BLACK,grid,&mut bq,1);
        Self::compute_distances(GRID_WHITE,grid,&mut wk,2); Self::compute_distances(GRID_BLACK,grid,&mut bk,2);
        let (mut t1,mut t2,mut p1,mut p2)=(0.0,0.0,0.0,0.0);
        for i in 0..GRIDSIZE { for j in 0..GRIDSIZE { if grid[i][j]!=0 {continue;} if wq[i][j]<bq[i][j]{t1+=1.0}else if wq[i][j]>bq[i][j]{t1-=1.0}; if wk[i][j]<bk[i][j]{t2+=1.0}else if wk[i][j]>bk[i][j]{t2-=1.0}; p1 += 2f64.powi(-wq[i][j]) - 2f64.powi(-bq[i][j]); let diff=(bk[i][j]-wk[i][j]) as f64/6.0; p2 += diff.clamp(-1.0,1.0);} }
        let (a,b,c,d,e,f)= if turn_id<=20 {(0.14,0.33,0.13,0.17,0.17,0.10)} else if turn_id<=49 {(0.24,0.28,0.18,0.18,0.04,0.08)} else {(0.72,0.10,0.05,0.05,0.0,0.08)};
        let score=a*t1+b*t2+c*p1+d*p2+e*0.0+f*0.0;
        if self.curr_bot_color==GRID_WHITE {score} else {-score}
    }
    fn init_zobrist(&mut self){let mut x: u64 = 88172645463393265; for i in 0..GRIDSIZE {for j in 0..GRIDSIZE {for k in 0..4 {x ^= x<<7; x ^= x>>9; self.zobrist[i][j][k]=x;}}}}
    fn piece_idx(v:i32)->usize{if v==0 {0}else if v==GRID_BLACK {1}else if v==GRID_WHITE {2}else {3}}
    fn compute_initial_hash(&self)->u64{let mut h=0; for i in 0..GRIDSIZE {for j in 0..GRIDSIZE {let p=Self::piece_idx(self.grid_info[i][j]); if p!=0 {h ^= self.zobrist[i][j][p];}}} h}
    fn update_hash(&self, old:u64,m:Move,color:i32)->u64{let mut h=old; h ^= self.zobrist[m.initgrid.x][m.initgrid.y][Self::piece_idx(color)]; h ^= self.zobrist[m.newgrid.x][m.newgrid.y][Self::piece_idx(color)]; h ^= self.zobrist[m.arows.x][m.arows.y][Self::piece_idx(OBSTACLE)]; h}
}

fn main() {
    let mut engine = Engine::new();
    let mut input = String::new(); io::stdin().read_to_string(&mut input).unwrap();
    let mut it = input.split_whitespace();
    let turn_id: i32 = it.next().unwrap_or("0").parse().unwrap_or(0);
    for i in 0..turn_id {
        let vals: Vec<i32> = (0..6).map(|_| it.next().unwrap_or("-1").parse().unwrap_or(-1)).collect();
        if vals[0] == -1 { engine.curr_bot_color = GRID_BLACK; }
        else {
            let m = Move{initgrid:Point{x:vals[0] as usize,y:vals[1] as usize},newgrid:Point{x:vals[2] as usize,y:vals[3] as usize},arows:Point{x:vals[4] as usize,y:vals[5] as usize}};
            engine.grid_info[m.initgrid.x][m.initgrid.y]=0; engine.grid_info[m.newgrid.x][m.newgrid.y]=-engine.curr_bot_color; engine.grid_info[m.arows.x][m.arows.y]=OBSTACLE;
            engine.current_hash = engine.update_hash(engine.current_hash,m,-engine.curr_bot_color);
        }
        if i < turn_id-1 { let _ : Vec<i32> = (0..6).map(|_| it.next().unwrap_or("-1").parse().unwrap_or(-1)).collect(); }
    }
    let mut root = Engine::get_valid_moves(engine.curr_bot_color,&engine.grid_info);
    if root.is_empty() { println!("-1 -1 -1 -1 -1 -1"); return; }
    root.sort_by_key(|m| -Engine::quick_move_score(*m, engine.curr_bot_color, &engine.grid_info));
    let best = root[0];
    println!("{} {} {} {} {} {}", best.initgrid.x,best.initgrid.y,best.newgrid.x,best.newgrid.y,best.arows.x,best.arows.y);
}
