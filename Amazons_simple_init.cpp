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

// 棋盘尺寸定义 8x8
#define GRIDSIZE 8
// 障碍/箭标记值
#define OBSTACLE 2
// 裁判方：黑方标识
#define judge_black 0
// 裁判方：白方标识
#define judge_white 1
// 棋盘黑皇后值
#define grid_black 1
// 棋盘白皇后值
#define grid_white -1

// 计时起始时间
clock_t start_time;
// 全局超时标记，控制搜索提前终止
bool stop_searching = false;

// --- 计时函数 ---
// 启动计时器，记录搜索开始时间
void startTimer() {
    start_time = clock();
}

// 判断是否超出时间限制（0.93秒）
bool timeIsUp() {
    const double elapsed = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    return elapsed > 0.93;
}

// 历史启发表：记录优质走法，用于走法排序优化 [玩家][x][y]
int history[2][GRIDSIZE][GRIDSIZE] = {0};

// 迭代加深最大搜索深度
const int MAX_DEPTH = 1000;

using namespace std;

// 当前AI执子颜色（1黑，-1白）
int currBotColor;
// 棋盘状态数组：x横坐标，y纵坐标
int gridInfo[GRIDSIZE][GRIDSIZE] = { 0 };
// 8个方向偏移量x
int dx[] = { -1,-1,-1,0,0,1,1,1 };
// 8个方向偏移量y
int dy[] = { -1,0,1,-1,1,-1,0,1 };

// 棋子坐标结构体
struct Point {
    int x;  // 横坐标
    int y;  // 纵坐标
    Point() : x(0), y(0) {}  // 默认构造
    Point(int x0, int y0) : x(x0), y(y0) {}  // 带参构造
};

// 亚马逊棋一步完整走法：原位置 -> 移动位置 -> 射箭位置
struct Move {
    Point initgrid;  // 皇后初始位置
    Point newgrid;   // 皇后移动后位置
    Point arows;     // 射箭落点
    Move(Point init, Point newgrid, Point arows) : initgrid(init), newgrid(newgrid), arows(arows) {}
    Move() {}
};

// 置换表条目结构体：存储搜索过的局面信息
struct TTEntry {
    unsigned long long key;  // Zobrist哈希键
    int depth;               // 搜索深度
    double value;            // 局面评估值
    int flag;                // 分数类型：0精确值/1下界/2上界
    Move bestMove;           // 该局面最优走法
};

// 置换表大小 2^21
const int TT_SIZE = 2 << 20;
// 全局置换表数组
TTEntry TT[TT_SIZE];

// Zobrist哈希随机数表：[x][y][棋子类型]
unsigned long long zobristTable[GRIDSIZE][GRIDSIZE][4];
// 当前棋盘全局哈希值
unsigned long long currentHash = 0;

// 初始化Zobrist随机数表，固定种子保证可复现
void initZobrist() {
    std::mt19937_64 rng(12345);
    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            for (int k = 0; k < 4; k++) {
                zobristTable[i][j][k] = rng();
            }
        }
    }
}

// 将棋盘值映射为Zobrist表索引：0空/1黑/2白/3障碍
int getPieceIndex(int val) {
    if (val == 0) return 0;
    if (val == grid_black) return 1;
    if (val == grid_white) return 2;
    if (val == OBSTACLE) return 3;
    return 0;
}

// 计算初始棋盘的Zobrist哈希值
unsigned long long computeInitialHash() {
    unsigned long long h = 0;
    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            int pieceIdx = getPieceIndex(gridInfo[i][j]);
            if (pieceIdx != 0) {
                h ^= zobristTable[i][j][pieceIdx];
            }
        }
    }
    return h;
}

// 根据一步走法更新哈希值，返回新哈希
unsigned long long updateHash(unsigned long long oldHash, int x0, int y0, int x1, int y1, int x2, int y2, int color) {
    unsigned long long newHash = oldHash;
    // 移除原位置皇后
    newHash ^= zobristTable[x0][y0][getPieceIndex(color)];
    // 添加新位置皇后
    newHash ^= zobristTable[x1][y1][getPieceIndex(color)];
    // 添加箭（障碍）
    newHash ^= zobristTable[x2][y2][getPieceIndex(OBSTACLE)];
    return newHash;
}

// 判断坐标是否在棋盘内
inline bool inMap(int x, int y) {
    if (x < 0 || x >= GRIDSIZE || y < 0 || y >= GRIDSIZE)
        return false;
    return true;
}

// 执行一步落子：check_only=true仅校验合法性，false执行落子
bool ProcStep(int x0, int y0, int x1, int y1, int x2, int y2, int color, bool check_only) {
    // 越界判断
    if ((!inMap(x0, y0)) || (!inMap(x1, y1)) || (!inMap(x2, y2)))
        return false;
    // 原位置必须是己方棋子，目标位置必须为空
    if (gridInfo[x0][y0] != color || gridInfo[x1][y1] != 0)
        return false;
    // 射箭位置必须为空（除了原位置）
    if ((gridInfo[x2][y2] != 0) && !(x2 == x0 && y2 == y0))
        return false;
    // 非校验模式，执行棋盘修改
    if (!check_only) {
        gridInfo[x0][y0] = 0;
        gridInfo[x1][y1] = color;
        gridInfo[x2][y2] = OBSTACLE;
    }
    return true;
}

// 判断格子是否可以移动/射箭（空且在棋盘内）
bool LegalStep(int x, int y, int grid_info[GRIDSIZE][GRIDSIZE]) {
    if (!inMap(x, y)) return false;
    // 不能是障碍、黑棋、白棋
    else if (grid_info[x][y] == OBSTACLE || grid_info[x][y] == grid_black || grid_info[x][y] == grid_white) {
        return false;
    }
    return true;
}

// 获取单个皇后所有合法移动位置（皇后走法）
vector<Point> get_move_pos(Point cu_point, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    vector<Point> vecpo;
    // 遍历8个方向
    for (int d = 0; d < 8; d++) {
        int px = cu_point.x;
        int py = cu_point.y;
        while (true) {
            px += dx[d];
            py += dy[d];
            if (LegalStep(px, py, tpgrid)) {
                vecpo.emplace_back(px, py);
            }
            else {
                break;
            }
        }
    }
    return vecpo;
}

// 获取皇后移动后所有合法射箭位置
vector<Point> get_arrow_pos(int temp_grid[GRIDSIZE][GRIDSIZE], Point newpoint) {
    vector<Point> vecpo;
    // 遍历8个方向
    for (int d = 0; d < 8; d++) {
        int px = newpoint.x;
        int py = newpoint.y;
        while (true) {
            px += dx[d];
            py += dy[d];
            if (LegalStep(px, py, temp_grid)) {
                vecpo.emplace_back(px, py);
            }
            else {
                break;
            }
        }
    }
    return vecpo;
}

// 生成指定颜色所有合法走法
vector<Move> get_valid_moves(int color, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    vector<Move> all_valid_moves;  // 全部合法走法
    vector<Point> starts;           // 己方所有皇后位置

    // 遍历棋盘，收集己方皇后坐标
    for (int x = 0; x < GRIDSIZE; x++) {
        for (int y = 0; y < GRIDSIZE; y++) {
            if (tpgrid[x][y] == color) {
                starts.emplace_back(x, y);
            }
        }
    }

    // 遍历每个皇后，生成移动+射箭组合
    for (auto& p : starts) {
        vector<Point> move_positions = get_move_pos(p, tpgrid);

        for (auto& new_p : move_positions) {
            // 复制临时棋盘，模拟皇后移动
            int temp_grid[GRIDSIZE][GRIDSIZE];
            memcpy(temp_grid, tpgrid, sizeof(int[GRIDSIZE][GRIDSIZE]));
            temp_grid[p.x][p.y] = 0;
            temp_grid[new_p.x][new_p.y] = color;

            // 获取移动后可射箭位置
            vector<Point> arrows_position = get_arrow_pos(temp_grid, new_p);

            for (auto& arrow : arrows_position) {
                // 不能射自己脚下
                if (arrow.x == new_p.x && arrow.y == new_p.y) continue;
                all_valid_moves.emplace_back(p, new_p, arrow);
            }
        }
    }

    return all_valid_moves;
}

// 中心位置评分：越靠近中心分数越高
inline int centerScore(const Point& p) {
    return 7 - (abs(p.x - 3) + abs(p.y - 3));
}

// 快速走法评分：用于走法排序，中心、位移、封锁对手权重更高
int quickMoveScore(const Move& m, int color, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    int score = 0;
    score += centerScore(m.newgrid) * 6;
    score += centerScore(m.arows) * 2;
    // 移动距离越长分数越高
    score += max(abs(m.newgrid.x - m.initgrid.x), abs(m.newgrid.y - m.initgrid.y)) * 2;

    // 箭靠近对方皇后加分
    int opp = -color;
    for (int d = 0; d < 8; ++d) {
        int nx = m.arows.x + dx[d], ny = m.arows.y + dy[d];
        if (inMap(nx, ny) && tpgrid[nx][ny] == opp) score += 8;
    }
    return score;
}

// BFS计算空格到棋子的最短距离：mode1皇后走法，mode2国王走法
void computeDistances(int color, int tpgrid[GRIDSIZE][GRIDSIZE], int distMap[GRIDSIZE][GRIDSIZE], int mode) {
    // 初始化距离为极大值
    for (int i = 0; i < GRIDSIZE; ++i)
        for (int j = 0; j < GRIDSIZE; ++j)
            distMap[i][j] = 100;

    queue<pair<int, int>> q;
    // 初始化己方棋子距离为0
    for (int i = 0; i < GRIDSIZE; ++i) {
        for (int j = 0; j < GRIDSIZE; ++j) {
            if (tpgrid[i][j] == color) {
                distMap[i][j] = 0;
                q.push({ i, j });
            }
        }
    }

    // BFS扩散计算
    while (!q.empty()) {
        pair<int, int> curr = q.front();
        q.pop();
        int nextDist = distMap[curr.first][curr.second] + 1;
        for (int d = 0; d < 8; d++) {
            int nx = curr.first + dx[d];
            int ny = curr.second + dy[d];

            if (mode == 1) {  // 皇后：连续移动
                while (inMap(nx, ny) && tpgrid[nx][ny] == 0) {
                    if (distMap[nx][ny] > nextDist) {
                        distMap[nx][ny] = nextDist;
                        q.push({ nx, ny });
                    }
                    else if (distMap[nx][ny] < nextDist) {
                        break;
                    }
                    nx += dx[d];
                    ny += dy[d];
                }
            }
            else {  // 国王：单步移动
                if (inMap(nx, ny) && tpgrid[nx][ny] == 0) {
                    if (distMap[nx][ny] > distMap[curr.first][curr.second] + 1) {
                        distMap[nx][ny] = distMap[curr.first][curr.second] + 1;
                        q.push({ nx, ny });
                    }
                }
            }
        }
    }
}

// 计算单个皇后可达空格数量（轻量机动性评估）
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

// 计算玩家机动性得分：总可达格子+最小可达格子
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

// 局面评估函数：领地、位置、机动性加权求和
double evaluate(int tpgrid[GRIDSIZE][GRIDSIZE], int turnID) {
    int whiteDistQ[GRIDSIZE][GRIDSIZE], blackDistQ[GRIDSIZE][GRIDSIZE];
    int whiteDistK[GRIDSIZE][GRIDSIZE], blackDistK[GRIDSIZE][GRIDSIZE];

    // 计算双方距离矩阵
    computeDistances(grid_white, tpgrid, whiteDistQ, 1);
    computeDistances(grid_black, tpgrid, blackDistQ, 1);
    computeDistances(grid_white, tpgrid, whiteDistK, 2);
    computeDistances(grid_black, tpgrid, blackDistK, 2);

    double t1 = 0, t2 = 0, p1 = 0, p2 = 0;

    // 遍历空格计算领地与位置优势
    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            if (tpgrid[i][j] != 0) continue;

            // 皇后领地优势
            if (whiteDistQ[i][j] < blackDistQ[i][j]) t1 += 1.0;
            else if (whiteDistQ[i][j] > blackDistQ[i][j]) t1 -= 1.0;

            // 国王领地优势
            if (whiteDistK[i][j] < blackDistK[i][j]) t2 += 1.0;
            else if (whiteDistK[i][j] > blackDistK[i][j]) t2 -= 1.0;

            // 皇后位置价值
            p1 += (pow(2, -whiteDistQ[i][j]) - pow(2, -blackDistQ[i][j]));
            // 国王位置价值
            double diffK = (double)(blackDistK[i][j] - whiteDistK[i][j]) / 6.0;
            p2 += max(-1.0, min(1.0, diffK));
        }
    }

    // 机动性差值
    double m = getMobilityScore(grid_white, tpgrid) - getMobilityScore(grid_black, tpgrid);

    // 按回合分配权重：开局/中局/残局
    double a, b, c, d, e;
    if (turnID <= 20) {
        a = 0.14; b = 0.37; c = 0.13; d = 0.13; e = 0.20;
    }
    else if (turnID <= 49) {
        a = 0.25; b = 0.30; c = 0.20; d = 0.20; e = 0.05;
    }
    else {
        a = 0.80; b = 0.10; c = 0.05; d = 0.05; e = 0.00;
    }

    // 加权总分，黑方反转分数
    double score = a * t1 + b * t2 + c * p1 + d * p2 + e * m;
    return (currBotColor == grid_white) ? score : -score;
}

// 模拟一步走法，生成新棋盘
void get_newgrid(int tmpgrid[GRIDSIZE][GRIDSIZE], Move& move, int color) {
    Point start = move.initgrid, newg = move.newgrid, arrow = move.arows;
    tmpgrid[start.x][start.y] = 0;
    tmpgrid[newg.x][newg.y] = color;
    tmpgrid[arrow.x][arrow.y] = 2;
}

// 判断两步走法是否完全相同
bool isSameMove(const Move& m1, const Move& m2) {
    return m1.initgrid.x == m2.initgrid.x && m1.initgrid.y == m2.initgrid.y &&
        m1.newgrid.x == m2.newgrid.x && m1.newgrid.y == m2.newgrid.y &&
        m1.arows.x == m2.arows.x && m1.arows.y == m2.arows.y;
}

// 终局得分：无子可走判负
double terminalScore(bool isMaxPlayer) {
    return isMaxPlayer ? -1e8 : 1e8;
}

// ========================== 最终 MinMax（带历史启发 + TT + αβ）==========================
// 极大极小值搜索 + αβ剪枝 + 置换表 + 历史启发
double MinMax(int grid[GRIDSIZE][GRIDSIZE], int depth, bool isMax, int turnID, double arfa, double beta, unsigned long long h) {
    // 超时终止搜索，返回当前评估值
    if (stop_searching || timeIsUp()) {
        stop_searching = true;
        return evaluate(grid, turnID);
    }

    // 计算置换表索引
    int index = h & (TT_SIZE - 1);
    Move bestMoveFromTT;
    bool hasBestMoveFromTT = false;
    const double alphaOrig = arfa;
    const double betaOrig = beta;

    // 置换表查询：读取历史最优走法与分数
    if (TT[index].key == h) {
        bestMoveFromTT = TT[index].bestMove;
        hasBestMoveFromTT = true;

        // 深度足够直接返回缓存值
        if (TT[index].depth >= depth) {
            if (TT[index].flag == 0) return TT[index].value;
            if (TT[index].flag == 1 && TT[index].value >= beta) return TT[index].value;
            if (TT[index].flag == 2 && TT[index].value <= arfa) return TT[index].value;
        }
    }

    // 叶子节点，调用评估函数
    if (depth == 0) {
        return evaluate(grid, turnID);
    }

    Move bestMoveInThisNode;  // 当前节点最优走法
    bool foundBest = false;   // 是否找到有效走法

    // 极大层：我方回合
    if (isMax) {
        vector<Move> moves = get_valid_moves(currBotColor, grid);
        if (moves.empty()) return terminalScore(true);
        double currentmax = -1e9;

        // 走法排序：历史启发优先 + 快速评分
        stable_sort(moves.begin(), moves.end(), [&](const Move& a, const Move& b) {
            int cidx = (currBotColor == grid_black) ? 0 : 1;
            int ha = history[cidx][a.initgrid.x][a.initgrid.y];
            int hb = history[cidx][b.initgrid.x][b.initgrid.y];
            if (ha != hb) return ha > hb;
            return quickMoveScore(a, currBotColor, grid) > quickMoveScore(b, currBotColor, grid);
        });

        // 置换表最优走法置顶
        if (hasBestMoveFromTT) {
            for (size_t i = 0; i < moves.size(); i++) {
                if (isSameMove(moves[i], bestMoveFromTT)) {
                    swap(moves[0], moves[i]);
                    break;
                }
            }
        }

        // 遍历所有走法
        for (auto& m : moves) {
            int tmpgrid[GRIDSIZE][GRIDSIZE];
            memcpy(tmpgrid, grid, sizeof(int[GRIDSIZE][GRIDSIZE]));
            get_newgrid(tmpgrid, m, currBotColor);

            // 计算新哈希
            unsigned long long next_h = updateHash(h, m.initgrid.x, m.initgrid.y,
                m.newgrid.x, m.newgrid.y,
                m.arows.x, m.arows.y, currBotColor);

            // 递归搜索
            double tpscore = MinMax(tmpgrid, depth - 1, false, turnID, arfa, beta, next_h);

            // 更新最大值
            if (tpscore > currentmax) {
                currentmax = tpscore;
                bestMoveInThisNode = m;
                foundBest = true;
            }

            // β剪枝，更新历史启发
            if (currentmax > beta) {
                int cidx = (currBotColor == grid_black) ? 0 : 1;
                history[cidx][m.initgrid.x][m.initgrid.y] += 120;
                break;
            }
            if (currentmax > arfa) arfa = currentmax;
        }

        // 写入置换表
        if (TT[index].key != h || depth >= TT[index].depth) {
            TT[index].key = h;
            TT[index].depth = depth;
            TT[index].value = currentmax;
            if (foundBest) TT[index].bestMove = bestMoveInThisNode;
            if (currentmax <= alphaOrig)      TT[index].flag = 2;
            else if (currentmax >= betaOrig)  TT[index].flag = 1;
            else                              TT[index].flag = 0;
        }
        return currentmax;
    }
    // 极小层：对手回合
    else {
        vector<Move> moves = get_valid_moves((-1) * currBotColor, grid);
        if (moves.empty()) return terminalScore(false);
        double currentmin = 1e9;

        // 走法排序
        stable_sort(moves.begin(), moves.end(), [&](const Move& a, const Move& b) {
            int oppColor = -currBotColor;
            int cidx = (oppColor == grid_black) ? 0 : 1;
            int ha = history[cidx][a.initgrid.x][a.initgrid.y];
            int hb = history[cidx][b.initgrid.x][b.initgrid.y];
            if (ha != hb) return ha > hb;
            return quickMoveScore(a, oppColor, grid) > quickMoveScore(b, oppColor, grid);
        });

        // 置换表最优走法置顶
        if (hasBestMoveFromTT) {
            for (size_t i = 0; i < moves.size(); i++) {
                if (isSameMove(moves[i], bestMoveFromTT)) {
                    swap(moves[0], moves[i]);
                    break;
                }
            }
        }

        // 遍历所有走法
        for (auto& m : moves) {
            int tmpgrid[GRIDSIZE][GRIDSIZE];
            memcpy(tmpgrid, grid, sizeof(int[GRIDSIZE][GRIDSIZE]));
            get_newgrid(tmpgrid, m, (-1) * currBotColor);

            unsigned long long next_h = updateHash(h, m.initgrid.x, m.initgrid.y,
                m.newgrid.x, m.newgrid.y,
                m.arows.x, m.arows.y, (-1) * currBotColor);

            double tpscore = MinMax(tmpgrid, depth - 1, true, turnID, arfa, beta, next_h);

            // 更新最小值
            if (tpscore < currentmin) {
                currentmin = tpscore;
                bestMoveInThisNode = m;
                foundBest = true;
            }

            // α剪枝，更新历史启发
            if (currentmin < arfa) {
                int oppColor = -currBotColor;
                int cidx = (oppColor == grid_black) ? 0 : 1;
                history[cidx][m.initgrid.x][m.initgrid.y] += 120;
                break;
            }
            if (currentmin < beta) beta = currentmin;
        }

        // 写入置换表
        if (TT[index].key != h || depth >= TT[index].depth) {
            TT[index].key = h;
            TT[index].depth = depth;
            TT[index].value = currentmin;
            if (foundBest) TT[index].bestMove = bestMoveInThisNode;
            if (currentmin <= alphaOrig)      TT[index].flag = 2;
            else if (currentmin >= betaOrig)  TT[index].flag = 1;
            else                              TT[index].flag = 0;
        }
        return currentmin;
    }
}

// MTD(f)搜索框架：基于零窗口搜索的高效寻优算法
double MTDF(unsigned long long h, double f, int depth, int turnID) {
    double g = f;
    double upperbound = 1e9;
    double lowerbound = -1e9;

    // 迭代缩小区间
    while (lowerbound < upperbound - 0.001) {
        double beta = (g == lowerbound) ? g + 0.01 : g;
        // 零窗口MinMax搜索
        g = MinMax(gridInfo, depth, true, turnID, beta - 0.01, beta, h);
        if (stop_searching) break;
        if (g < beta) upperbound = g;
        else lowerbound = g;
    }
    return g;
}

int main() {
    int x0, y0, x1, y1, x2, y2;

    // 初始化棋盘：黑方初始位置
    gridInfo[0][2] = grid_black;
    gridInfo[2][0] = grid_black;
    gridInfo[5][0] = grid_black;
    gridInfo[7][2] = grid_black;

    // 白方初始位置
    gridInfo[0][5] = grid_white;
    gridInfo[2][7] = grid_white;
    gridInfo[5][7] = grid_white;
    gridInfo[7][5] = grid_white;

    // 初始化Zobrist哈希与初始哈希值
    initZobrist();
    currentHash = computeInitialHash();

    // 读取回合数
    int turnID;
    cin >> turnID;

    // 默认白方先手
    currBotColor = grid_white;
    // 恢复棋盘历史状态
    for (int i = 0; i < turnID; i++) {
        cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
        // 首回合输入-1，说明本方是黑方
        if (x0 == -1)
            currBotColor = grid_black;
        else {
            // 模拟对手落子，更新棋盘与哈希
            ProcStep(x0, y0, x1, y1, x2, y2, -currBotColor, false);
            currentHash = updateHash(currentHash, x0, y0, x1, y1, x2, y2, -currBotColor);
        }

        // 读取并恢复自己上一步的走法
        if (i < turnID - 1) {
            cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
            if (x0 >= 0) {
                ProcStep(x0, y0, x1, y1, x2, y2, currBotColor, false);
                currentHash = updateHash(currentHash, x0, y0, x1, y1, x2, y2, currBotColor);
            }
        }
    }

    // 启动计时器，初始化搜索
    startTimer();
    stop_searching = false;
    vector<Move> rootMoves = get_valid_moves(currBotColor, gridInfo);
    Move overallBestMove = rootMoves.empty() ? Move() : rootMoves[0];
    double last_score = 0;

    // 迭代加深搜索：1层到最大层
    for (int d = 1; d <= MAX_DEPTH; d++) {
        memset(history, 0, sizeof(history));  // 每层清空历史表
        double current_score = MTDF(currentHash, last_score, d, turnID);

        if (stop_searching) break;
        last_score = current_score;

        // 从置换表读取最优走法
        int index = currentHash & (TT_SIZE - 1);
        if (TT[index].key == currentHash) {
            overallBestMove = TT[index].bestMove;
        }
    }

    // 校验最优走法合法性，兜底处理
    bool legalBest = false;
    for (auto& m : rootMoves) {
        if (isSameMove(m, overallBestMove)) {
            legalBest = true;
            break;
        }
    }
    if (!legalBest) overallBestMove = rootMoves[0];

    // 解析最终走法坐标
    int startX = overallBestMove.initgrid.x;
    int startY = overallBestMove.initgrid.y;
    int resultX = overallBestMove.newgrid.x;
    int resultY = overallBestMove.newgrid.y;
    int obstacleX = overallBestMove.arows.x;
    int obstacleY = overallBestMove.arows.y;

    // 输出走法
    cout << startX << ' ' << startY << ' ' << resultX << ' ' << resultY << ' ' << obstacleX << ' ' << obstacleY << endl;
    return 0;
}