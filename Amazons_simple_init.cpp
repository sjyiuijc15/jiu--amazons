#include<random>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>
#include<queue>
#include<algorithm>
#include<cmath>
#include <map>
#include<cstring>
#define GRIDSIZE 8
#define OBSTACLE 2
#define judge_black 0
#define judge_white 1
#define grid_black 1
#define grid_white -1
clock_t start_time;
bool stop_searching = false;
// --- 计时函数 ---
void startTimer() {
    start_time = clock();
}
bool timeIsUp() {
    // Botzone 单步通常 1s，动态缓冲防抖
    const double elapsed = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    return elapsed > 0.93;
}
const int MAX_DEPTH = 20; // 迭代加深的最大深度，根据实际时间调整
using namespace std;
int currBotColor; // 本方所执子颜色（1为黑，-1为白，棋盘状态亦同）
int gridInfo[GRIDSIZE][GRIDSIZE] = { 0 }; // 先x后y，记录棋盘状态
int dx[] = { -1,-1,-1,0,0,1,1,1 };
int dy[] = { -1,0,1,-1,1,-1,0,1 };
// 评估因子结构体
struct EvaluationResult {
    double t1, t2, p1, p2, m;
};
// 棋子坐标结构体
struct Point{
    int x; // 横坐标
    int y; // 纵坐标
    Point() : x(0), y(0) {} // 默认构造函数
    Point(int x0, int y0) : x(x0), y(y0) {} // 构造函数
};


// 一步走法(原位置->移动->射箭)
struct Move{
	Point initgrid;
	Point newgrid;
	Point arows;
	Move(Point init, Point newgrid, Point arows) : initgrid(init), newgrid(newgrid), arows(arows) {}
    Move() {}
};

//置换表结构体
struct TTEntry {
    unsigned long long key; // Zobrist Hash
    int depth;              // 搜索深度
    double value;           // 评估分
    int flag;              // 分数类型
    Move bestMove;          // 在该局面下的最佳走法
};
const int TT_SIZE = 2<<20;
// 建议开一个足够大的全局数组，例如 2^20 次方个条目
TTEntry TT[TT_SIZE];
// 1. 定义 Zobrist 随机数表
// 状态映射：0->空格, 1->黑皇后, 2->白皇后, 3->障碍
unsigned long long zobristTable[GRIDSIZE][GRIDSIZE][4];
unsigned long long currentHash = 0;
// 使用高质量的 64 位随机数生成器
void initZobrist() {
    std::mt19937_64 rng(12345); // 固定种子保证调试可复现，正式比赛可用 time(0)
    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            for (int k = 0; k < 4; k++) {
                zobristTable[i][j][k] = rng();
            }
        }
    }
}
// 将棋盘数值映射到 0-3 的索引
int getPieceIndex(int val) {
    if (val == 0) return 0;           // 空
    if (val == grid_black) return 1;  // 黑 (1)
    if (val == grid_white) return 2;  // 白 (-1)
    if (val == OBSTACLE) return 3;    // 障碍 (2)
    return 0;
}

// 初始化当前棋盘的 Hash 值（仅在程序开始或重新加载棋盘时调用一次）
unsigned long long computeInitialHash() {
    unsigned long long h = 0;
    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            int pieceIdx = getPieceIndex(gridInfo[i][j]);
            if (pieceIdx != 0) { // 空格通常不参与异或，或者参与也行，保持一致即可
                h ^= zobristTable[i][j][pieceIdx];
            }
        }
    }
    return h;
}
// 在模拟落子时同步更新 Hash
// 纯函数：根据动作计算出新的 Hash 值，不改变全局变量
unsigned long long updateHash(unsigned long long oldHash, int x0, int y0, int x1, int y1, int x2, int y2, int color) {
    unsigned long long newHash = oldHash;
    
    // 1. 拿走旧位置的皇后 (x0, y0)
    newHash ^= zobristTable[x0][y0][getPieceIndex(color)];
    
    // 2. 放入新位置的皇后 (x1, y1)
    newHash ^= zobristTable[x1][y1][getPieceIndex(color)];
    
    // 3. 放入新射出的箭 (x2, y2)
    // 特别注意：亚马逊棋射箭后的位置在这一步之前是空的
    newHash ^= zobristTable[x2][y2][getPieceIndex(OBSTACLE)];
    
    return newHash;
}
// 判断是否在棋盘内
inline bool inMap(int x, int y)
{
	if (x < 0 || x >= GRIDSIZE || y < 0 || y >= GRIDSIZE)
		return false;
	return true;
}


// 在坐标处落子，检查是否合法或模拟落子
bool ProcStep(int x0, int y0, int x1, int y1, int x2, int y2, int color, bool check_only)
{
	if ((!inMap(x0, y0)) || (!inMap(x1, y1)) || (!inMap(x2, y2)))
		return false;
	if (gridInfo[x0][y0] != color || gridInfo[x1][y1] != 0)
		return false;
	if ((gridInfo[x2][y2] != 0) && !(x2 == x0 && y2 == y0))
		return false;
	if (!check_only)
	{
		gridInfo[x0][y0] = 0;
		gridInfo[x1][y1] = color;
		gridInfo[x2][y2] = OBSTACLE;
	}
	return true;
}


// part2.1: 判断格子能不能走
bool LegalStep(int x, int y, int grid_info[GRIDSIZE][GRIDSIZE]){
	if(!inMap(x, y)) return false;
	else if(grid_info[x][y] == OBSTACLE || grid_info[x][y] == grid_black || grid_info[x][y] == grid_white){
		// 不是边界, 不是亚马逊, 不是箭
		return false;
	}
	return true;
}


// part2.2: 一个亚马逊能移动到哪些位置
vector<Point> get_move_pos(Point cu_point, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    vector<Point> vecpo;
    // 8个方向
    for (int d = 0; d < 8; d++) {
        int px = cu_point.x;
        int py = cu_point.y;
        while (true) {
            px += dx[d];
            py += dy[d];
            if (LegalStep(px, py, tpgrid)) {
                vecpo.emplace_back(px, py);
            } else {
                break;
            }
        }
    }
    return vecpo;
}


// part2.3: 移动后能射哪里
vector<Point> get_arrow_pos(int temp_grid[GRIDSIZE][GRIDSIZE], Point newpoint){
	vector<Point> vecpo;
    // 8个方向
    for (int d = 0; d < 8; d++) {
        int px = newpoint.x;
        int py = newpoint.y;
        while (true) {
            px += dx[d];
            py += dy[d];
            if (LegalStep(px, py, temp_grid)) {
                vecpo.emplace_back(px, py);
            } else {
                break;
            }
        }
    }
    return vecpo;
}


// part2.4: 生成完整走法
vector<Move> get_valid_moves(int color, int tpgrid[GRIDSIZE][GRIDSIZE]){
	vector<Move> all_valid_moves;// 全部走法
	vector<Point> starts; //四个初始棋子位置

	//获取四个亚马逊的位置
	for(int x = 0; x < GRIDSIZE; x++){
		for(int y = 0; y < GRIDSIZE; y++){
			if(tpgrid[x][y] == color){
				starts.emplace_back(x, y);
			}
		}
	}

	for(auto& p : starts){
		vector<Point> move_positions = get_move_pos(p, tpgrid);

		for(auto& new_p : move_positions){
			// 临时棋盘，模拟移动到新位置
			int temp_grid[GRIDSIZE][GRIDSIZE];
			memcpy(temp_grid, tpgrid, sizeof(int[GRIDSIZE][GRIDSIZE]));
			temp_grid[p.x][p.y] = 0;
			temp_grid[new_p.x][new_p.y] = color;

			// 获取射箭位置
			vector<Point> arrows_position = get_arrow_pos(temp_grid, new_p);

			for(auto& arrow : arrows_position){
				// 不能射自己脚下
				if(arrow.x == new_p.x && arrow.y == new_p.y) continue;

				all_valid_moves.emplace_back(p, new_p, arrow);
			}
		}
	}

	return all_valid_moves;
}

// 轻量级走法排序：优先中心、长位移、封锁对手邻域
inline int centerScore(const Point& p) {
    return 7 - (abs(p.x - 3) + abs(p.y - 3));
}

int quickMoveScore(const Move& m, int color, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    int score = 0;
    score += centerScore(m.newgrid) * 6;
    score += centerScore(m.arows) * 2;

    // 位移越长通常空间越大
    score += max(abs(m.newgrid.x - m.initgrid.x), abs(m.newgrid.y - m.initgrid.y)) * 2;

    // 箭周围若是对手棋子，优先考虑（更可能形成封锁）
    int opp = -color;
    for (int d = 0; d < 8; ++d) {
        int nx = m.arows.x + dx[d], ny = m.arows.y + dy[d];
        if (inMap(nx, ny) && tpgrid[nx][ny] == opp) score += 8;
    }
    return score;
}
//----------------------------------评估函数开始--------------------------

// 使用广度优先搜索 (BFS) 计算所有空格到棋子的最短距离
// mode: 1 为 Queen 走法, 2 为 King 走法
void computeDistances(int color, int tpgrid[GRIDSIZE][GRIDSIZE], int distMap[GRIDSIZE][GRIDSIZE], int mode) {
    for (int i = 0; i < GRIDSIZE; ++i)
        for (int j = 0; j < GRIDSIZE; ++j)
            distMap[i][j] = 100; // 初始化为无穷大

    queue<pair<int, int>> q;
    for (int i = 0; i < GRIDSIZE; ++i) {
        for (int j = 0; j < GRIDSIZE; ++j) {
            if (tpgrid[i][j] == color) {
                distMap[i][j] = 0;
                q.push({i, j});
            }
        }
    }

    while (!q.empty()) {
        pair<int, int> curr = q.front();
        q.pop();
        int nextDist = distMap[curr.first][curr.second] + 1;
        for (int d = 0; d < 8; d++) {
            int nx = curr.first + dx[d];
            int ny = curr.second + dy[d];

            if (mode == 1) { // Queen 走法: 一次移动可跨越多个空格
                while (inMap(nx, ny) && tpgrid[nx][ny] == 0) {
                    if (distMap[nx][ny] > nextDist) {
                        distMap[nx][ny] = nextDist;
                        q.push({nx, ny});
                    }
                    else if(distMap[nx][ny] < nextDist){
                        break;
                    }
                    nx += dx[d];
                    ny += dy[d];
                }
            } else { // King 走法: 一次只能走一步
                if (inMap(nx, ny) && tpgrid[nx][ny] == 0) {
                    if (distMap[nx][ny] > distMap[curr.first][curr.second] + 1) {
                        distMap[nx][ny] = distMap[curr.first][curr.second] + 1;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }
}

// 轻量 Mobility：只统计每个皇后的可达空格数（不展开射箭）
// 速度比完整合法步统计快很多，更适合 1s 时限下的深搜。
int reachableCountFromQueen(const Point& q, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    int count = 0;
    for (int d = 0; d < 8; ++d) {
        int x = q.x + dx[d], y = q.y + dy[d];
        while (inMap(x, y) && tpgrid[x][y] == 0) {
            ++count;
            x += dx[d];
            y += dy[d];
        }
    }
    return count;
}

double getMobilityScore(int color, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    int totalReach = 0;
    int minReach = 9999;
    int queenCnt = 0;

    for (int x = 0; x < GRIDSIZE; ++x) {
        for (int y = 0; y < GRIDSIZE; ++y) {
            if (tpgrid[x][y] != color) continue;
            ++queenCnt;
            int reach = reachableCountFromQueen(Point(x, y), tpgrid);
            totalReach += reach;
            minReach = min(minReach, reach);
        }
    }

    if (queenCnt == 0 || minReach == 9999) minReach = 0;
    return (double)totalReach + (double)minReach;
}

// 评估函数核心实现
double evaluate(int tpgrid[GRIDSIZE][GRIDSIZE], int turnID) {
    int whiteDistQ[GRIDSIZE][GRIDSIZE], blackDistQ[GRIDSIZE][GRIDSIZE];
    int whiteDistK[GRIDSIZE][GRIDSIZE], blackDistK[GRIDSIZE][GRIDSIZE];

    // 1. 计算距离矩阵 [cite: 342, 470]
    computeDistances(grid_white, tpgrid, whiteDistQ, 1);
    computeDistances(grid_black, tpgrid, blackDistQ, 1);
    computeDistances(grid_white, tpgrid, whiteDistK, 2);
    computeDistances(grid_black, tpgrid, blackDistK, 2);

    double t1 = 0, t2 = 0, p1 = 0, p2 = 0;

    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            if (tpgrid[i][j] != 0) continue;

            // Territory 计算 (t1: Queen, t2: King) [cite: 409]
            if (whiteDistQ[i][j] < blackDistQ[i][j]) t1 += 1.0;
            else if (whiteDistQ[i][j] > blackDistQ[i][j]) t1 -= 1.0;

            if (whiteDistK[i][j] < blackDistK[i][j]) t2 += 1.0;
            else if (whiteDistK[i][j] > blackDistK[i][j]) t2 -= 1.0;

            // Position 计算 (p1: Queen, p2: King) [cite: 413, 414]
            p1 += (pow(2, -whiteDistQ[i][j]) - pow(2, -blackDistQ[i][j]));
            double diffK = (double)(blackDistK[i][j] - whiteDistK[i][j]) / 6.0;
            p2 += max(-1.0, min(1.0, diffK));
        }
    }

    // 2. Mobility 计算 [cite: 452, 457, 458]
    double m = getMobilityScore(grid_white, tpgrid) - getMobilityScore(grid_black, tpgrid);

    // 3. 确定阶段权重 
    double a, b, c, d, e; 
    if (turnID <= 20) { // 开局
        a = 0.14; b = 0.37; c = 0.13; d = 0.13; e = 0.20;
    } else if (turnID <= 49) { // 中局
        a = 0.25; b = 0.30; c = 0.20; d = 0.20; e = 0.05;
    } else { // 残局
        a = 0.80; b = 0.10; c = 0.05; d = 0.05; e = 0.00;
    }

    // 4. 最终计算 [cite: 338]
    double score = a * t1 + b * t2 + c * p1 + d * p2 + e * m;
    
    // 如果当前机器人是黑方，则反转分数
    return (currBotColor == grid_white) ? score : -score;
}
//----------------------------------评估函数结束--------------------------


//----------------------------------MinMax算法 + α-β剪枝---------------------------
//模拟一步后的新棋盘
void get_newgrid(int tmpgrid[GRIDSIZE][GRIDSIZE],Move &move, int color){
    Point start = move.initgrid, newg = move.newgrid, arrow = move.arows;
    tmpgrid[start.x][start.y] = 0;
    tmpgrid[newg.x][newg.y] = color;
    tmpgrid[arrow.x][arrow.y] = 2;
}

// 判断两步走法是否完全相同的辅助函数（如果你的 Move 结构体没有重载 == 运算符的话）
bool isSameMove(const Move& m1, const Move& m2) {
    return m1.initgrid.x == m2.initgrid.x && m1.initgrid.y == m2.initgrid.y &&
           m1.newgrid.x == m2.newgrid.x && m1.newgrid.y == m2.newgrid.y &&
           m1.arows.x == m2.arows.x && m1.arows.y == m2.arows.y;
}

double terminalScore(bool isMaxPlayer) {
    // 当前层无棋可走：若轮到我方（Max）则失败，若轮到对手（Min）则成功
    return isMaxPlayer ? -1e8 : 1e8;
}

double MinMax(int grid[GRIDSIZE][GRIDSIZE], int depth, bool isMax, int turnID, double arfa, double beta, unsigned long long h) {
   //检查超时
    if (stop_searching || timeIsUp()) {
        stop_searching = true;
        return evaluate(grid, turnID); 
    }
    int index = h & (TT_SIZE - 1);
    Move bestMoveFromTT;
    bool hasBestMoveFromTT = false;
    const double alphaOrig = arfa;
    const double betaOrig = beta;
    // 1. 查表：不仅查分数，还要尝试提取最佳走法
    if (TT[index].key == h) {
        // 只要这个局面曾被搜索过，就把当时认为最好的走法拿出来（用于走法排序）
        bestMoveFromTT = TT[index].bestMove;
        hasBestMoveFromTT = true;

        // 如果表中的深度足够，才直接返回分数
        if (TT[index].depth >= depth) {
            if (TT[index].flag == 0) return TT[index].value; // 精确值直接用
            if (TT[index].flag == 1 && TT[index].value >= beta) return TT[index].value; // Beta 剪枝
            if (TT[index].flag == 2 && TT[index].value <= arfa) return TT[index].value; // Alpha 剪枝
        }
    }

    // 触底调用评估函数
    if (depth == 0) {
        return evaluate(grid, turnID);
    }

    Move bestMoveInThisNode; // 用于记录当前层表现最好的走法
    bool foundBest = false;  // 标记是否找到了有效走法

    if (isMax) {
        vector<Move> moves = get_valid_moves(currBotColor, grid);
        if (moves.empty()) return terminalScore(true);
        double currentmax = -1e9;

        stable_sort(moves.begin(), moves.end(), [&](const Move& a, const Move& b){
            return quickMoveScore(a, currBotColor, grid) > quickMoveScore(b, currBotColor, grid);
        });

        // 【关键优化：走法排序 Move Ordering】
        if (hasBestMoveFromTT && !moves.empty()) {
            for (size_t i = 0; i < moves.size(); i++) {
                if (isSameMove(moves[i], bestMoveFromTT)) {
                    // 找到了历史最佳走法！将它与第0个走法交换位置，确保它被第一个搜索
                    swap(moves[0], moves[i]);
                    break;
                }
            }
        }

        for (auto& m : moves) {
            int tmpgrid[GRIDSIZE][GRIDSIZE];
            memcpy(tmpgrid, grid, sizeof(int[GRIDSIZE][GRIDSIZE]));
            get_newgrid(tmpgrid, m, currBotColor);

            unsigned long long next_h = updateHash(h, m.initgrid.x, m.initgrid.y, 
                                                   m.newgrid.x, m.newgrid.y, 
                                                   m.arows.x, m.arows.y, currBotColor);
            
            double tpscore = MinMax(tmpgrid, depth-1, false, turnID, arfa, beta, next_h);
            
            if (tpscore > currentmax) {
                currentmax = tpscore;
                bestMoveInThisNode = m; // 记录产生最高分的走法
                foundBest = true;
            }
            
            // 剪枝逻辑
            if (currentmax > beta) break; // Beta 剪枝
            if (currentmax > arfa) arfa = currentmax; // 更新 Alpha
        }

        // 2. 存表前，记录最佳走法
      // 只有当新搜索更深（更准）时才更新
if (TT[index].key != h || depth >= TT[index].depth) {
    TT[index].key = h;
    TT[index].depth = depth;
    TT[index].value = currentmax; // 或 currentmin
    if (foundBest) {
        TT[index].bestMove = bestMoveInThisNode; // 记录当前层的最佳走法
    }
     // 设置 flag
        if (currentmax <= alphaOrig) {
            TT[index].flag = 2; // 上界 (Alpha 剪枝)
        } else if (currentmax >= betaOrig) {
            TT[index].flag = 1; // 下界 (Beta 剪枝)
        } else {
            TT[index].flag = 0; // 精确值
        }
}
       
        return currentmax;
        
    } else {
        // Min 层逻辑（对手回合）
        vector<Move> moves = get_valid_moves((-1)*currBotColor, grid);
        if (moves.empty()) return terminalScore(false);
        double currentmin = 1e9;

        stable_sort(moves.begin(), moves.end(), [&](const Move& a, const Move& b){
            return quickMoveScore(a, -currBotColor, grid) > quickMoveScore(b, -currBotColor, grid);
        });

        // 【关键优化：走法排序 Move Ordering】
        if (hasBestMoveFromTT && !moves.empty()) {
            for (size_t i = 0; i < moves.size(); i++) {
                if (isSameMove(moves[i], bestMoveFromTT)) {
                    swap(moves[0], moves[i]); // 对手的最优走法也要优先搜索
                    break;
                }
            }
        }

        for (auto& m : moves) {
            int tmpgrid[GRIDSIZE][GRIDSIZE];
            memcpy(tmpgrid, grid, sizeof(int[GRIDSIZE][GRIDSIZE]));
            get_newgrid(tmpgrid, m, (-1) * currBotColor);

            unsigned long long next_h = updateHash(h, m.initgrid.x, m.initgrid.y, 
                                                   m.newgrid.x, m.newgrid.y, 
                                                   m.arows.x, m.arows.y, (-1) * currBotColor);
            
            double tpscore = MinMax(tmpgrid, depth-1, true, turnID, arfa, beta, next_h);
            
            if (tpscore < currentmin) { // 注意这里是小于
                currentmin = tpscore;
                bestMoveInThisNode = m; // 记录让对手得分最低（对我方最不利）的走法
                foundBest = true;
            }
            
            // 剪枝逻辑
            if (currentmin < arfa) break; // Alpha 剪枝 (由于上层传下来的是arfa, 此处直接用arfa判断也是等价的, 但严格写是 currentmin <= arfa)
            if (currentmin < beta) beta = currentmin; // 更新 Beta
        }

        // 2. 存表前，记录最佳走法
      // 只有当新搜索更深（更准）时才更新
if (TT[index].key != h || depth >= TT[index].depth) {
    TT[index].key = h;
    TT[index].depth = depth;
    TT[index].value = currentmin; // 或 currentmin
    if (foundBest) {
        TT[index].bestMove = bestMoveInThisNode; // 记录当前层的最佳走法
    }
     // 设置 flag
        if (currentmin <= alphaOrig) {
            TT[index].flag = 2; // 上界 (Alpha 剪枝)
        } else if (currentmin >= betaOrig) {
            TT[index].flag = 1; // 下界 (Beta 剪枝)
        } else {
            TT[index].flag = 0; // 精确值
        }

}
return currentmin;
    }
}
/// MTDF 搜索框架
double MTDF(unsigned long long h, double f, int depth, int turnID) {
    double g = f;
    double upperbound = 1e9;
    double lowerbound = -1e9;
    
    while (lowerbound < upperbound-0.001) {
        // 设定一个极窄的“零窗口” [beta-1, beta]
        double beta = (g == lowerbound) ? g + 0.01 : g;
        
        // 调用你已经写好的带 TT 的 MinMax
        // 注意：这里的 alpha 是 beta - 0.01 (或者是很小的量，确保窗口极窄)
        g = MinMax(gridInfo, depth, true, turnID, beta - 0.01, beta, h);
        if(stop_searching) break; // 如果搜索过程中发现时间到了，直接退出
        if (g < beta) {
            upperbound = g;
        } else {
            lowerbound = g;
        }
    }
    return g;
}
// //MinMax
// Move getbestmove(int depth, int turnID){
//     vector<Move> moves = get_valid_moves(currBotColor, gridInfo);
//     double score = -1e9;
//     Move bestmove;
//     for(auto&m : moves){
//         int tmpgrid[GRIDSIZE][GRIDSIZE];
//         memcpy(tmpgrid, gridInfo, sizeof(int[GRIDSIZE][GRIDSIZE]));
//         get_newgrid(tmpgrid, m, currBotColor);
//         unsigned long long h = updateHash(currentHash, m.initgrid.x, m.initgrid.y, m.newgrid.x, m.newgrid.y, m.arows.x, m.arows.y, currBotColor);
//         double tpscore = MinMax(tmpgrid, depth - 1, false, turnID, -1e9, 1e9, h);
//         if(tpscore > score){
//             score = tpscore;
//             bestmove = m;
//         }
//     }
//     return bestmove;
// }
//--------------------------------------------------------------------

int main()
{
	int x0, y0, x1, y1, x2, y2;
	// 初始化棋盘
	gridInfo[0][2] = grid_black;
	gridInfo[2][0] = grid_black;
	gridInfo[5][0] = grid_black;
	gridInfo[7][2] = grid_black;

	gridInfo[0][5] = grid_white;
	gridInfo[2][7] = grid_white;
	gridInfo[5][7] = grid_white;
	gridInfo[7][5] = grid_white;
//初始化Zobrist表和初始Hash值
    initZobrist();
currentHash = computeInitialHash();
	// 分析自己收到的输入和自己过往的输出，并恢复棋盘状态
	int turnID;
	cin >> turnID;

	currBotColor = grid_white; // 先假设自己是白方
	for (int i = 0; i < turnID; i++)
	{
		// 根据这些输入输出逐渐恢复状态到当前回合

		// 首先是对手行动
		cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
		if (x0 == -1)
			currBotColor = grid_black; // 第一回合收到坐标是-1, -1，说明我方是黑方
		else{
			ProcStep(x0, y0, x1, y1, x2, y2, -currBotColor, false); // 模拟对方落子
currentHash = updateHash(currentHash, x0, y0, x1, y1, x2, y2, -currBotColor);
    }
        															// 然后是本方当时的行动
																	// 对手行动总比自己行动多一个
		if (i < turnID - 1)
		{
			cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
			if (x0 >= 0){
				ProcStep(x0, y0, x1, y1, x2, y2, currBotColor, false); // 模拟本方落子
		currentHash = updateHash(currentHash, x0, y0, x1, y1, x2, y2, currBotColor);
    }
	}
    }
	/*********************************************************************************************************/
	/***在下面填充你的代码，决策结果（本方将落子的位置）存入startX、startY、resultX、resultY、obstacleX、obstacleY中*****/
    startTimer(); // [新增] 启动计时
    stop_searching = false;
    vector<Move> rootMoves = get_valid_moves(currBotColor, gridInfo);
    Move overallBestMove = rootMoves.empty() ? Move() : rootMoves[0];
    double last_score = 0;
for (int d = 1; d <= MAX_DEPTH; d++) {
   double current_score = MTDF(currentHash, last_score, d, turnID);
        
        if (stop_searching) break; // 如果这一层搜到一半超时了，不采用这一层的结果

        last_score = current_score;
        
        // 只有完整搜完这一层，才更新“全局最佳走法”
        int index = currentHash & (TT_SIZE - 1);
        if (TT[index].key == currentHash) {
            overallBestMove = TT[index].bestMove;
        }
}

if (rootMoves.empty()) {
    cout << "-1 -1 -1 -1 -1 -1" << endl;
    return 0;
}

// 兜底：若 TT 给出异常走法则退回首个合法走法
bool legalBest = false;
for (auto &m : rootMoves) {
    if (isSameMove(m, overallBestMove)) { legalBest = true; break; }
}
if (!legalBest) overallBestMove = rootMoves[0];

// 最后输出
    int startX = overallBestMove.initgrid.x;
    int startY = overallBestMove.initgrid.y;  
    int resultX = overallBestMove.newgrid.x;
    int resultY = overallBestMove.newgrid.y;
    int obstacleX = overallBestMove.arows.x;
    int obstacleY = overallBestMove.arows.y;
    
	
	/****在上方填充你的代码，决策结果（本方将落子的位置）存入startX、startY、resultX、resultY、obstacleX、obstacleY中****/
	/*********************************************************************************************************/
	
	cout << startX << ' ' << startY << ' ' << resultX << ' ' << resultY << ' ' << obstacleX << ' ' << obstacleY << endl;
	return 0;
}
