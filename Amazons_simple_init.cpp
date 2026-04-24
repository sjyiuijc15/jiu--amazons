
#include <iostream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>
#include<queue>
#include<algorithm>
#include<cmath>

#define GRIDSIZE 8
#define OBSTACLE 2
#define judge_black 0
#define judge_white 1
#define grid_black 1
#define grid_white -1

using namespace std;
int currBotColor; // 本方所执子颜色（1为黑，-1为白，棋盘状态亦同）
int gridInfo[GRIDSIZE][GRIDSIZE] = { 0 }; // 先x后y，记录棋盘状态
int dx[] = { -1,-1,-1,0,0,1,1,1 };
int dy[] = { -1,0,1,-1,1,-1,0,1 };

// 评估因子结构体
struct EvaluationResult {
    double t1, t2, p1, p2, m;
};



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


// 棋子坐标结构体
struct Point{
	int x; // 横坐标
	int y; // 纵坐标
	Point(int x0, int y0) : x(x0), y(y0) {} // 构造函数
};


// 一步走法(原位置->移动->射箭)
struct Move{
	Point initgrid;
	Point newgrid;
	Point arows;
	Move(Point init, Point newgrid, Point arows) : initgrid(init), newgrid(newgrid), arows(arows) {}
};


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
vector<Point> get_move_pos(Point cu_point) {
    vector<Point> vecpo;
    // 8个方向
    for (int d = 0; d < 8; d++) {
        int px = cu_point.x;
        int py = cu_point.y;
        while (true) {
            px += dx[d];
            py += dy[d];
            if (LegalStep(px, py, gridInfo)) {
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
vector<Move> get_valid_moves(int color = currBotColor){
	vector<Move> all_valid_moves;// 全部走法
	vector<Point> starts; //四个初始棋子位置

	//获取四个亚马逊的位置
	for(int x = 0; x < GRIDSIZE; x++){
		for(int y = 0; y < GRIDSIZE; y++){
			if(gridInfo[x][y] == color){
				starts.emplace_back(x, y);
			}
		}
	}

	for(auto& p : starts){
		vector<Point> move_positions = get_move_pos(p);

		for(auto& new_p : move_positions){
			// 临时棋盘，模拟移动到新位置
			int temp_grid[GRIDSIZE][GRIDSIZE];
			memcpy(temp_grid, gridInfo, sizeof(gridInfo));
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
//----------------------------------评估函数开始--------------------------

// 使用广度优先搜索 (BFS) 计算所有空格到棋子的最短距离
// mode: 1 为 Queen 走法, 2 为 King 走法
void computeDistances(int color, int distMap[GRIDSIZE][GRIDSIZE], int mode) {
    for (int i = 0; i < GRIDSIZE; ++i)
        for (int j = 0; j < GRIDSIZE; ++j)
            distMap[i][j] = 100; // 初始化为无穷大

    queue<pair<int, int>> q;
    for (int i = 0; i < GRIDSIZE; ++i) {
        for (int j = 0; j < GRIDSIZE; ++j) {
            if (gridInfo[i][j] == color) {
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
                while (inMap(nx, ny) && gridInfo[nx][ny] == 0) {
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
                if (inMap(nx, ny) && gridInfo[nx][ny] == 0) {
                    if (distMap[nx][ny] > distMap[curr.first][curr.second] + 1) {
                        distMap[nx][ny] = distMap[curr.first][curr.second] + 1;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }
}
//计算黑白棋子的灵活度， (Mobility) 这里使用简化法：统计可行着法数，并惩罚最小灵活度棋子
double getMobilityScore(int color) {
    int totalMoves = 0;
    int minMoves = 9999; // 初始设为一个较大值，用于记录四个棋子中的最小值
    
    // 1. 找到当前颜色（我方或对方）的所有棋子位置
    vector<Point> queens;
    for (int x = 0; x < GRIDSIZE; x++) {
        for (int y = 0; y < GRIDSIZE; y++) {
            if (gridInfo[x][y] == color) {
                queens.emplace_back(x, y);
            }
        }
    }

    // 2. 遍历每一个皇后，计算其可行着法
    for (auto& q : queens) {
        int currentQueenMoves = 0;
        
        // 获取该棋子一步之内能到达的所有位置（Queen走法）
        vector<Point> move_positions = get_move_pos(q);
        
        for (auto& new_p : move_positions) {
            // 模拟移动：为了准确计算射箭位置，需要临时改变棋盘状态
            int original_val = gridInfo[q.x][q.y];
            gridInfo[q.x][q.y] = 0;
            int target_original_val = gridInfo[new_p.x][new_p.y];
            gridInfo[new_p.x][new_p.y] = color;

            // 获取在该移动位置下能射箭的所有位置
            // 注意：此处 get_arrow_pos 内部应使用当前的 gridInfo
            vector<Point> arrows = get_arrow_pos(gridInfo, new_p);
            currentQueenMoves += (int)arrows.size();

            // 还原棋盘状态
            gridInfo[new_p.x][new_p.y] = target_original_val;
            gridInfo[q.x][q.y] = original_val;
        }

        // 累计总着法数
        totalMoves += currentQueenMoves;
        // 记录四个棋子中“最不灵活”的那个棋子的着法数 
        if (currentQueenMoves < minMoves) {
            minMoves = currentQueenMoves;
        }
    }

    // 如果该色棋子已经全部无法移动，minMoves 应设为 0
    if (queens.empty() || minMoves == 9999) minMoves = 0;

    // 根据论文公式：总灵活度 = 所有棋子着法总和 + 最小灵活度值 
    return (double)totalMoves + (double)minMoves;
}
// 评估函数核心实现
double evaluate(int turnID) {
    int whiteDistQ[GRIDSIZE][GRIDSIZE], blackDistQ[GRIDSIZE][GRIDSIZE];
    int whiteDistK[GRIDSIZE][GRIDSIZE], blackDistK[GRIDSIZE][GRIDSIZE];

    // 1. 计算距离矩阵 [cite: 342, 470]
    computeDistances(grid_white, whiteDistQ, 1);
    computeDistances(grid_black, blackDistQ, 1);
    computeDistances(grid_white, whiteDistK, 2);
    computeDistances(grid_black, blackDistK, 2);

    double t1 = 0, t2 = 0, p1 = 0, p2 = 0;

    for (int i = 0; i < GRIDSIZE; i++) {
        for (int j = 0; j < GRIDSIZE; j++) {
            if (gridInfo[i][j] != 0) continue;

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
    double m = getMobilityScore(grid_white) - getMobilityScore(grid_black);

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
		else
			ProcStep(x0, y0, x1, y1, x2, y2, -currBotColor, false); // 模拟对方落子

																	// 然后是本方当时的行动
																	// 对手行动总比自己行动多一个
		if (i < turnID - 1)
		{
			cin >> x0 >> y0 >> x1 >> y1 >> x2 >> y2;
			if (x0 >= 0)
				ProcStep(x0, y0, x1, y1, x2, y2, currBotColor, false); // 模拟本方落子
		}
	}

	
	/*********************************************************************************************************/
	/***在下面填充你的代码，决策结果（本方将落子的位置）存入startX、startY、resultX、resultY、obstacleX、obstacleY中*****/
	//下面仅为随机策略的示例代码，且效率低，可删除
	
	
	/****在上方填充你的代码，决策结果（本方将落子的位置）存入startX、startY、resultX、resultY、obstacleX、obstacleY中****/
	/*********************************************************************************************************/
	
	// cout << startX << ' ' << startY << ' ' << resultX << ' ' << resultY << ' ' << obstacleX << ' ' << obstacleY << endl;
	return 0;
}