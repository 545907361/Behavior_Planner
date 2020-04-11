#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);


float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
	/*
	成本根据预期车道的距离（用于规划车道变更）和最终的轨迹车道而增加。
    随着车辆接近目标距离，离开球道的成本也会变得更大。
	*/
	float cost;
	float distance = data["distance_to_goal"];
	if (distance > 0) {
		cost = 1 - 2 * exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
	}
	else {
		cost = 1;
	}
	return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
	/*
	对于具有预期车道和最终车道的交通流量慢于车辆目标速度的轨迹，成本会更高。
	*/

	float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
	if (proposed_speed_intended < 0) {
		proposed_speed_intended = vehicle.target_speed;
	}

	float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
	if (proposed_speed_final < 0) {
		proposed_speed_final = vehicle.target_speed;
	}

	float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed;

	return cost;
}

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
	/*
	车道上的所有非自我车辆都具有相同的速度，因此要获得车道的速度限制，
    我们只能在那条车道上找到一辆车。
	*/
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
		int key = it->first;
		Vehicle vehicle = it->second[0];
		if (vehicle.lane == lane && key != -1) {
			return vehicle.v;
		}
	}
	//在车道上没有发现车辆
	return -1.0;
}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
	/*
	对加权成本函数求和可得到轨迹的总成本。
	*/
	map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
	float cost = 0.0;

	//在此处添加其他费用功能。
	vector< function<float(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = { goal_distance_cost, inefficiency_cost };
	vector<float> weight_list = { REACH_GOAL, EFFICIENCY };

	for (int i = 0; i < cf_list.size(); i++) {
		float new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
		cost += new_cost;
	}

	return cost;

}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
	/*
	生成用于成本函数的辅助数据：
    indended_lane：如果车辆正在计划或执行车道变更，则当前车道+/- 1。
    final_lane：轨迹终点处的车辆车道。
    distance_to_goal：车辆到目标的距离。

    请注意，同时包含indended_lane和final_lane有助于区分计划和执行
    成本函数的变化
	*/
	map<string, float> trajectory_data;
	Vehicle trajectory_last = trajectory[1];
	float intended_lane;

	if (trajectory_last.state.compare("PLCL") == 0) {
		intended_lane = trajectory_last.lane + 1;
	}
	else if (trajectory_last.state.compare("PLCR") == 0) {
		intended_lane = trajectory_last.lane - 1;
	}
	else {
		intended_lane = trajectory_last.lane;
	}

	float distance_to_goal = vehicle.goal_s - trajectory_last.s;
	float final_lane = trajectory_last.lane;
	trajectory_data["intended_lane"] = intended_lane;
	trajectory_data["final_lane"] = final_lane;
	trajectory_data["distance_to_goal"] = distance_to_goal;
	return trajectory_data;
}
