
#include <iostream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <vector>


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