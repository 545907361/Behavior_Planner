#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

//影响大多数状态的默认行为
int SPEED_LIMIT = 10;

//车道上的所有车流量（自我以外）都遵循这些速度
vector<int> LANE_SPEEDS = { 6,7,8,9 };

//应该有流量的可用“单元格”数量
double TRAFFIC_DENSITY = 0.15;

//在每个时间步，自我都可以将加速度设置为介于 
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
vector<int> GOAL = { 300, 0 };

//这些会影响可视化
int FRAMES_PER_SECOND = 4;
int AMOUNT_OF_ROAD_VISIBLE = 40;

int main() {

	Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

	road.update_width = AMOUNT_OF_ROAD_VISIBLE;

	road.populate_traffic();

	int goal_s = GOAL[0];
	int goal_lane = GOAL[1];

	//配置数据：速度限制，num_lanes，goal_s，goal_lane，max_acceleration

	int num_lanes = LANE_SPEEDS.size();
	vector<int> ego_config = { SPEED_LIMIT,num_lanes,goal_s,goal_lane,MAX_ACCEL };

	road.add_ego(2, 0, ego_config);
	int timestep = 0;

	while (road.get_ego().s <= GOAL[0]) {
		timestep++;
		if (timestep > 100) {
			break;
		}
		road.advance();
		road.display(timestep);
		//time.sleep(float(1.0) / FRAMES_PER_SECOND);
	}
	Vehicle ego = road.get_ego();
	if (ego.lane == GOAL[1])
	{
		cout << "您在 " << timestep << " 秒内达到目标！" << endl;
		if (timestep > 35)
		{
			cout << "但是花了太长时间才达到目标。快点！" << endl;
		}
	}
	else
	{
		cout << "您错过了目标。您在车道 " << ego.lane << " 而不是 " << GOAL[1] << "." << endl;
	}

	return 0;
}