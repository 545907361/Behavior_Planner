#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

	this->lane = lane;
	this->s = s;
	this->v = v;
	this->a = a;
	this->state = state;
	max_acceleration = -1;

}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
	/*
		在这里，您可以从“行为计划”伪代码中实现transition_function代码
		课堂的概念。您的目标将是返回最佳（最低成本）的轨迹
		进入下一个状态。

		输入：预测图。这是带有预测的车辆ID密钥的地图
		车辆轨迹作为值。轨迹是Vehicle对象的向量，表示
		在当前时间步和将来一个时间步的车辆。
		输出：对应于下一个自我车辆状态的最佳（最低成本）轨迹。

		*/
	vector<string> states = successor_states();
	float cost;
	vector<float> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
		vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
		if (trajectory.size() != 0) {
			cost = calculate_cost(*this, predictions, trajectory);
			costs.push_back(cost);
			final_trajectories.push_back(trajectory);
		}
	}

	vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);
	return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
	/*
		根据FSM的当前状态提供可能的下一个状态
		在课程中讨论过，除了换道发生
		因此，LCL和LCR只能瞬间过渡回KL。
		*/
	vector<string> states;
	states.push_back("KL");
	string state = this->state;
	if (state.compare("KL") == 0) {
		states.push_back("PLCL");
		states.push_back("PLCR");
	}
	else if (state.compare("PLCL") == 0) {
		if (lane != lanes_available - 1) {
			states.push_back("PLCL");
			states.push_back("LCL");
		}
	}
	else if (state.compare("PLCR") == 0) {
		if (lane != 0) {
			states.push_back("PLCR");
			states.push_back("LCR");
		}
	}
	//如果状态为“ LCL”或“ LCR”，则只需返回“ KL”
	return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	/*
	给定可能的下一个状态，请生成适当的轨迹以实现下一个状态。
	*/
	vector<Vehicle> trajectory;
	if (state.compare("CS") == 0) {
		trajectory = constant_speed_trajectory();
	}
	else if (state.compare("KL") == 0) {
		trajectory = keep_lane_trajectory(predictions);
	}
	else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
		trajectory = lane_change_trajectory(state, predictions);
	}
	else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
		trajectory = prep_lane_change_trajectory(state, predictions);
	}
	return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
	/*
	获取下一个时间步运动学（位置，速度，加速度） 
    给定车道 尝试选择最大速度和加速度，
    给定其他车辆位置和加速度/速度约束。
	*/
	float max_velocity_accel_limit = this->max_acceleration + this->v;
	float new_position;
	float new_velocity;
	float new_accel;
	Vehicle vehicle_ahead;
	Vehicle vehicle_behind;

	if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

		if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
			new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
		}
		else {
			float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
			new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
		}
	}
	else {
		new_velocity = min(max_velocity_accel_limit, this->target_speed);
	}

	new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
	new_position = this->s + new_velocity + new_accel / 2.0;
	return{ new_position, new_velocity, new_accel };

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
	/*
	生成恒定速度的轨迹
	*/
	float next_pos = position_at(1);
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->v, this->a, this->state),
								  Vehicle(this->lane, next_pos, this->v, 0, this->state) };
	return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
	/*
	生成保持车道轨迹。
	*/
	vector<Vehicle> trajectory = { Vehicle(lane, this->s, this->v, this->a, state) };
	vector<float> kinematics = get_kinematics(predictions, this->lane);
	float new_s = kinematics[0];
	float new_v = kinematics[1];
	float new_a = kinematics[2];
	trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
	return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	/*
	生成轨迹，为换道做好准备。
	*/
	float new_s;
	float new_v;
	float new_a;
	Vehicle vehicle_behind;
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->v, this->a, this->state) };
	vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

	if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
		//保持当前车道的速度，以免与后面的汽车相撞。
		new_s = curr_lane_new_kinematics[0];
		new_v = curr_lane_new_kinematics[1];
		new_a = curr_lane_new_kinematics[2];

	}
	else {
		vector<float> best_kinematics;
		vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
		//选择运动速度最低的运动。
		if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
			best_kinematics = next_lane_new_kinematics;
		}
		else {
			best_kinematics = curr_lane_new_kinematics;
		}
		new_s = best_kinematics[0];
		new_v = best_kinematics[1];
		new_a = best_kinematics[2];
	}

	trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
	return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
	/*
	生成车道变更轨迹。
	*/
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory;
	Vehicle next_lane_vehicle;
	//检查是否可以更改车道（检查是否有其他车辆占用该车位）。
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		next_lane_vehicle = it->second[0];
		if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
			//如果无法更改车道，则返回空轨迹。
			return trajectory;
		}
	}
	trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
	vector<float> kinematics = get_kinematics(predictions, new_lane);
	trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
	return trajectory;
}

void Vehicle::increment(int dt = 1) {
	this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
	return this->s + this->v*t + this->a*t*t / 2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	/*
	如果在当前车辆后面找到车辆，则返回true，否则返回false。通过的参考
    如果找到车辆，则更新rVehicle。
	*/
	int max_s = -1;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
			max_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
	/*
	如果在当前车辆之前找到车辆，则返回true，否则返回false。通过的参考
    如果找到车辆，则更新rVehicle。
	*/
	int min_s = this->goal_s;
	bool found_vehicle = false;
	Vehicle temp_vehicle;
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		temp_vehicle = it->second[0];
		if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
			min_s = temp_vehicle.s;
			rVehicle = temp_vehicle;
			found_vehicle = true;
		}
	}
	return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
	/*
	生成要使用的非自我驾驶车辆的预测
    自我车辆的轨迹生成。
	*/
	vector<Vehicle> predictions;
	for (int i = 0; i < horizon; i++) {
		float next_s = position_at(i);
		float next_v = 0;
		if (i < horizon - 1) {
			next_v = position_at(i + 1) - s;
		}
		predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
	}
	return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
	/*
	使用轨迹的最后状态设置自我车辆的状态和运动学。
	*/
	Vehicle next_state = trajectory[1];
	this->state = next_state.state;
	this->lane = next_state.lane;
	this->s = next_state.s;
	this->v = next_state.v;
	this->a = next_state.a;
}

void Vehicle::configure(vector<int> road_data) {
	/*
	在仿真开始之前由仿真器调用。集各种
    影响自我车辆的参数。
	*/
	target_speed = road_data[0];
	lanes_available = road_data[1];
	goal_s = road_data[2];
	goal_lane = road_data[3];
	max_acceleration = road_data[4];
}