#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

//Ӱ������״̬��Ĭ����Ϊ
int SPEED_LIMIT = 10;

//�����ϵ����г��������������⣩����ѭ��Щ�ٶ�
vector<int> LANE_SPEEDS = { 6,7,8,9 };

//Ӧ���������Ŀ��á���Ԫ������
double TRAFFIC_DENSITY = 0.15;

//��ÿ��ʱ�䲽�����Ҷ����Խ����ٶ�����Ϊ���� 
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
vector<int> GOAL = { 300, 0 };

//��Щ��Ӱ����ӻ�
int FRAMES_PER_SECOND = 4;
int AMOUNT_OF_ROAD_VISIBLE = 40;

int main() {

	Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

	road.update_width = AMOUNT_OF_ROAD_VISIBLE;

	road.populate_traffic();

	int goal_s = GOAL[0];
	int goal_lane = GOAL[1];

	//�������ݣ��ٶ����ƣ�num_lanes��goal_s��goal_lane��max_acceleration

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
		cout << "���� " << timestep << " ���ڴﵽĿ�꣡" << endl;
		if (timestep > 35)
		{
			cout << "���ǻ���̫��ʱ��ŴﵽĿ�ꡣ��㣡" << endl;
		}
	}
	else
	{
		cout << "�������Ŀ�ꡣ���ڳ��� " << ego.lane << " ������ " << GOAL[1] << "." << endl;
	}

	return 0;
}