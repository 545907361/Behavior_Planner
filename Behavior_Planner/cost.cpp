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
	�ɱ�����Ԥ�ڳ����ľ��루���ڹ滮��������������յĹ켣���������ӡ�
    ���ų����ӽ�Ŀ����룬�뿪����ĳɱ�Ҳ���ø���
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
	���ھ���Ԥ�ڳ��������ճ����Ľ�ͨ�������ڳ���Ŀ���ٶȵĹ켣���ɱ�����ߡ�
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
	�����ϵ����з����ҳ�����������ͬ���ٶȣ����Ҫ��ó������ٶ����ƣ�
    ����ֻ���������������ҵ�һ������
	*/
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
		int key = it->first;
		Vehicle vehicle = it->second[0];
		if (vehicle.lane == lane && key != -1) {
			return vehicle.v;
		}
	}
	//�ڳ�����û�з��ֳ���
	return -1.0;
}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
	/*
	�Լ�Ȩ�ɱ�������Ϳɵõ��켣���ܳɱ���
	*/
	map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
	float cost = 0.0;

	//�ڴ˴�����������ù��ܡ�
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
	�������ڳɱ������ĸ������ݣ�
    indended_lane������������ڼƻ���ִ�г����������ǰ����+/- 1��
    final_lane���켣�յ㴦�ĳ���������
    distance_to_goal��������Ŀ��ľ��롣

    ��ע�⣬ͬʱ����indended_lane��final_lane���������ּƻ���ִ��
    �ɱ������ı仯
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
