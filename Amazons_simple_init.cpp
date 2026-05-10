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
    const double elapsed = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    return elapsed > 0.93;
}

// 历史启发表
int history[2][GRIDSIZE][GRIDSIZE] = {0};

const int MAX_DEPTH = 1000;

using namespace std;

int currBotColor;
int gridInfo[GRIDSIZE][GRIDSIZE] = { 0 };
int dx[] = { -1,-1,-1,0,0,1,1,1 };
int dy[] = { -1,0,1,-1,1,-1,0,1 };

struct Point {
    int x;
    int y;
    Point() : x(0), y(0) {}
    Point(int x0, int y0) : x(x0), y(y0) {}
};

struct Move {
    Point initgrid;
    Point newgrid;
    Point arows;
    Move(Point init, Point newgrid, Point arows) : initgrid(init), newgrid(newgrid), arows(arows) {}
    Move() {}
};

// 置换表
struct TTEntry {
    unsigned long long key;
    int depth;
    double value;
    int flag;
    Move bestMove;
};

const int TT_SIZE = 2 << 20;
TTEntry TT[TT_SIZE];

unsigned long long zobristTable[GRIDSIZE][GRIDSIZE][4];
unsigned long long currentHash = 0;

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

int getPieceIndex(int val) {
    if (val == 0) return 0;
    if (val == grid_black) return 1;
    if (val == grid_white) return 2;
    if (val == OBSTACLE) return 3;
    return 0;
}

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

unsigned long long updateHash(unsigned long long oldHash, int x0, int y0, int x1, int y1, int x2, int y2, int color) {
    unsigned long long newHash = oldHash;
    newHash ^= zobristTable[x0][y0][getPieceIndex(color)];
    newHash ^= zobristTable[x1][y1][getPieceIndex(color)];
    newHash ^= zobristTable[x2][y2][getPieceIndex(OBSTACLE)];
    return newHash;
}

inline bool inMap(int x, int y) {
    if (x < 0 || x >= GRIDSIZE || y < 0 || y >= GRIDSIZE)
        return false;
    return true;
}

bool ProcStep(int x0, int y0, int x1, int y1, int x2, int y2, int color, bool check_only) {
    if ((!inMap(x0, y0)) || (!inMap(x1, y1)) || (!inMap(x2, y2)))
        return false;
    if (gridInfo[x0][y0] != color || gridInfo[x1][y1] != 0)
        return false;
    if ((gridInfo[x2][y2] != 0) && !(x2 == x0 && y2 == y0))
        return false;
    if (!check_only) {
        gridInfo[x0][y0] = 0;
        gridInfo[x1][y1] = color;
        gridInfo[x2][y2] = OBSTACLE;
    }
    return true;
}

bool LegalStep(int x, int y, int grid_info[GRIDSIZE][GRIDSIZE]) {
    if (!inMap(x, y)) return false;
    else if (grid_info[x][y] == OBSTACLE || grid_info[x][y] == grid_black || grid_info[x][y] == grid_white) {
        return false;
    }
    return true;
}

vector<Point> get_move_pos(Point cu_point, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    vector<Point> vecpo;
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

vector<Point> get_arrow_pos(int temp_grid[GRIDSIZE][GRIDSIZE], Point newpoint) {
    vector<Point> vecpo;
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

vector<Move> get_valid_moves(int color, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    vector<Move> all_valid_moves;
    vector<Point> starts;

    for (int x = 0; x < GRIDSIZE; x++) {
        for (int y = 0; y < GRIDSIZE; y++) {
            if (tpgrid[x][y] == color) {
                starts.emplace_back(x, y);
            }
        }
    }

    for (auto& p : starts) {
        vector<Point> move_positions = get_move_pos(p, tpgrid);

        for (auto& new_p : move_positions) {
            int temp_grid[GRIDSIZE][GRIDSIZE];
            memcpy(temp_grid, tpgrid, sizeof(int[GRIDSIZE][GRIDSIZE]));
            temp_grid[p.x][p.y] = 0;
            temp_grid[new_p.x][new_p.y] = color;

            vector<Point> arrows_position = get_arrow_pos(temp_grid, new_p);

            for (auto& arrow : arrows_position) {
                if (arrow.x == new_p.x && arrow.y == new_p.y) continue;
                all_valid_moves.emplace_back(p, new_p, arrow);
            }
        }
    }

    return all_valid_moves;
}

inline int centerScore(const Point& p) {
    return 7 - (abs(p.x - 3) + abs(p.y - 3));
}

int quickMoveScore(const Move& m, int color, int tpgrid[GRIDSIZE][GRIDSIZE]) {
    int score = 0;
    score += centerScore(m.newgrid) * 6;
    score += centerScore(m.arows) * 2;
    score += max(abs(m.newgrid.x - m.initgrid.x), abs(m.newgrid.y - m.initgrid.y)) * 2;

    int opp = -color;
    for (int d = 0; d < 8; ++d) {
        int nx = m.arows.x + dx[d], ny = m.arows.y + dy[d];
        if (inMap(nx, ny) && tpgrid[nx][ny] == opp) score += 8;
    }
    return score;
}

void computeDistances(int color, int tpgrid[GRIDSIZE][GRIDSIZE], int distMap[GRIDSIZE][GRIDSIZE], int mode) {
    for (int i = 0; i < GRIDSIZE; ++i)
        for (int j = 0; j < GRIDSIZE; ++j)
            distMap[i][j] = 100;

    queue<pair<int, int>> q;
    for (int i = 0; i < GRIDSIZE; ++i) {
        for (int j = 0; j < GRIDSIZE; ++j) {
            if (tpgrid[i][j] == color) {
                distMap[i][j] = 0;
                q.push({ i, j });
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

            if (mode == 1) {
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
            else {
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

double evaluate(int tpgrid[GRIDSIZE][GRIDSIZE], int turnID) {
    int whiteDistQ[GRIDSIZE][GRIDSIZE], blackDistQ[GRIDSIZE][GRIDSIZE];
    int whiteDistK[GRIDSIZE][GRIDSIZE], blackDistK[GRIDSIZE][GRIDSIZE];

    computeDistances(grid_white, tpgrid, whiteDistQ, 1);
    computeDistances(grid_black, tpgrid, blackDistQ, 1);
    computeDistances(grid_white, tpgrid, whiteDistK, 2);
    computeDistances(grid_black, tpgrid, blackDistK, 2);

    double t1 = 0, t2 = 0, p1 = 0, p2 = 0;

    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            if (tpgrid[i][j] != 0) continue;

            if (whiteDistQ[i][j] < blackDistQ[i][j]) t1 += 1.0;
            else if (whiteDistQ[i][j] > blackDistQ[i][j]) t1 -= 1.0;

            if (whiteDistK[i][j] < blackDistK[i][j]) t2 += 1.0;
            else if (whiteDistK[i][j] > blackDistK[i][j]) t2 -= 1.0;

            p1 += (pow(2, -whiteDistQ[i][j]) - pow(2, -blackDistQ[i][j]));
            double diffK = (double)(blackDistK[i][j] - whiteDistK[i][j]) / 6.0;
            p2 += max(-1.0, min(1.0, diffK));
        }
    }

    double m = getMobilityScore(grid_white, tpgrid) - getMobilityScore(grid_black, tpgrid);

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

    double score = a * t1 + b * t2 + c * p1 + d * p2 + e * m;
    return (currBotColor == grid_white) ? score : -score;
}

void get_newgrid(int tmpgrid[GRIDSIZE][GRIDSIZE], Move& move, int color) {
    Point start = move.initgrid, newg = move.newgrid, arrow = move.arows;
    tmpgrid[start.x][start.y] = 0;
    tmpgrid[newg.x][newg.y] = color;
    tmpgrid[arrow.x][arrow.y] = 2;
}

bool isSameMove(const Move& m1, const Move& m2) {
    return m1.initgrid.x == m2.initgrid.x && m1.initgrid.y == m2.initgrid.y &&
        m1.newgrid.x == m2.newgrid.x && m1.newgrid.y == m2.newgrid.y &&
        m1.arows.x == m2.arows.x && m1.arows.y == m2.arows.y;
}

double terminalScore(bool isMaxPlayer) {
    return isMaxPlayer ? -1e8 : 1e8;
}

// ========================== 最终 MinMax（带历史启发 + TT + αβ）==========================
double MinMax(int grid[GRIDSIZE][GRIDSIZE], int depth, bool isMax, int turnID, double arfa, double beta, unsigned long long h) {
    if (stop_searching || timeIsUp()) {
        stop_searching = true;
        return evaluate(grid, turnID);
    }

    int index = h & (TT_SIZE - 1);
    Move bestMoveFromTT;
    bool hasBestMoveFromTT = false;
    const double alphaOrig = arfa;
    const double betaOrig = beta;

    if (TT[index].key == h) {
        bestMoveFromTT = TT[index].bestMove;
        hasBestMoveFromTT = true;

        if (TT[index].depth >= depth) {
            if (TT[index].flag == 0) return TT[index].value;
            if (TT[index].flag == 1 && TT[index].value >= beta) return TT[index].value;
            if (TT[index].flag == 2 && TT[index].value <= arfa) return TT[index].value;
        }
    }

    if (depth == 0) {
        return evaluate(grid, turnID);
    }

    Move bestMoveInThisNode;
    bool foundBest = false;

    if (isMax) {
        vector<Move> moves = get_valid_moves(currBotColor, grid);
        if (moves.empty()) return terminalScore(true);
        double currentmax = -1e9;

        stable_sort(moves.begin(), moves.end(), [&](const Move& a, const Move& b) {
            int cidx = (currBotColor == grid_black) ? 0 : 1;
            int ha = history[cidx][a.initgrid.x][a.initgrid.y];
            int hb = history[cidx][b.initgrid.x][b.initgrid.y];
            if (ha != hb) return ha > hb;
            return quickMoveScore(a, currBotColor, grid) > quickMoveScore(b, currBotColor, grid);
        });

        if (hasBestMoveFromTT) {
            for (size_t i = 0; i < moves.size(); i++) {
                if (isSameMove(moves[i], bestMoveFromTT)) {
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

            double tpscore = MinMax(tmpgrid, depth - 1, false, turnID, arfa, beta, next_h);

            if (tpscore > currentmax) {
                currentmax = tpscore;
                bestMoveInThisNode = m;
                foundBest = true;
            }

            if (currentmax > beta) {
                int cidx = (currBotColor == grid_black) ? 0 : 1;
                history[cidx][m.initgrid.x][m.initgrid.y] += 120;
                break;
            }
            if (currentmax > arfa) arfa = currentmax;
        }

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
    else {
        vector<Move> moves = get_valid_moves((-1) * currBotColor, grid);
        if (moves.empty()) return terminalScore(false);
        double currentmin = 1e9;

        stable_sort(moves.begin(), moves.end(), [&](const Move& a, const Move& b) {
            int oppColor = -currBotColor;
            int cidx = (oppColor == grid_black) ? 0 : 1;
            int ha = history[cidx][a.initgrid.x][a.initgrid.y];
            int hb = history[cidx][b.initgrid.x][b.initgrid.y];
            if (ha != hb) return ha > hb;
            return quickMoveScore(a, oppColor, grid) > quickMoveScore(b, oppColor, grid);
        });

        if (hasBestMoveFromTT) {
            for (size_t i = 0; i < moves.size(); i++) {
                if (isSameMove(moves[i], bestMoveFromTT)) {
                    swap(moves[0], moves[i]);
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

            double tpscore = MinMax(tmpgrid, depth - 1, true, turnID, arfa, beta, next_h);

            if (tpscore < currentmin) {
                currentmin = tpscore;
                bestMoveInThisNode = m;
                foundBest = true;
            }

            if (currentmin < arfa) {
                int oppColor = -currBotColor;
                int cidx = (oppColor == grid_black) ? 0 : 1;
                history[cidx][m.initgrid.x][m.initgrid.y] += 120;
                break;
            }
            if (currentmin < beta) beta = currentmin;
        }

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

double MTDF(unsigned long long h, double f, int depth, int turnID) {
    double g = f;
    double upperbound = 1e9;
    double lowerbound = -1e9;

    while (lowerbound < upperbound - 0.001) {
        double beta = (g == lowerbound) ? g + 0.01 : g;
        g = MinMax(gridInfo, depth, true, turnID, beta - 0.01, beta, h);
        if (stop_searching) break;
        if (g < beta) upperbound = g;
        else lowerbound = g;
    }
    return g;
}

int main() {
    int x0, y0, x1, y1, x2, y2;

    gridInfo[0][2] = grid_black;
    gridInfo[2][0] = grid_black;
    gridInfo[5][0] = grid_black;
    gridInfo[7][2] = grid_black;

    gridInfo[0][5] = grid_white;
    gridInfo[2][7] = grid_white;
    gridInfo[5][7] = grid_white;
    gridInfo[7][5] = grid_white;

    initZobrist();
    currentHash = computeInitialHash();

    int turnID;
    cin >> turnID;

    currBotColor = grid_white;
    for (int i = 0; i < turnID; i++) {
        cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
        if (x0 == -1)
            currBotColor = grid_black;
        else {
            ProcStep(x0, y0, x1, y1, x2, y2, -currBotColor, false);
            currentHash = updateHash(currentHash, x0, y0, x1, y1, x2, y2, -currBotColor);
        }

        if (i < turnID - 1) {
            cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
            if (x0 >= 0) {
                ProcStep(x0, y0, x1, y1, x2, y2, currBotColor, false);
                currentHash = updateHash(currentHash, x0, y0, x1, y1, x2, y2, currBotColor);
            }
        }
    }

    startTimer();
    stop_searching = false;
    vector<Move> rootMoves = get_valid_moves(currBotColor, gridInfo);
    Move overallBestMove = rootMoves.empty() ? Move() : rootMoves[0];
    double last_score = 0;

    for (int d = 1; d <= MAX_DEPTH; d++) {
        memset(history, 0, sizeof(history));
        double current_score = MTDF(currentHash, last_score, d, turnID);

        if (stop_searching) break;
        last_score = current_score;

        int index = currentHash & (TT_SIZE - 1);
        if (TT[index].key == currentHash) {
            overallBestMove = TT[index].bestMove;
        }
    }

    bool legalBest = false;
    for (auto& m : rootMoves) {
        if (isSameMove(m, overallBestMove)) {
            legalBest = true;
            break;
        }
    }
    if (!legalBest) overallBestMove = rootMoves[0];

    int startX = overallBestMove.initgrid.x;
    int startY = overallBestMove.initgrid.y;
    int resultX = overallBestMove.newgrid.x;
    int resultY = overallBestMove.newgrid.y;
    int obstacleX = overallBestMove.arows.x;
    int obstacleY = overallBestMove.arows.y;

    cout << startX << ' ' << startY << ' ' << resultX << ' ' << resultY << ' ' << obstacleX << ' ' << obstacleY << endl;
    return 0;
}