#ifndef SRC_PATH_HPP_
#define SRC_PATH_HPP_

#include <iostream>
#include "spline.h"

using namespace std;

#define EST_PATH_PTS_NO 50
#define REFERENCE_VELOCITY 49.5
#define REFERENCE_ACCELERATION 0.4
#define TARGET_DISTANCE 90
#define LANES_NO 3

struct vehicle_data
{
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	double ref_vel;
	int cur_lane;
	int target_lane;
};

enum vehicle_state{keep_lane, change_lane_left, change_lane_right};

int GetCurrentLane(double d);

void DefineVehiclePath(vector<double> & next_x_vals,
					   vector<double> & next_y_vals,
					   const vector<double> & prev_path_x,
					   const vector<double> & prev_path_y,
					   const vector<double> & map_waypoints_x,
					   const vector<double> & map_waypoints_y,
					   const vector<double> & map_waypoints_s,
					   const vehicle_data & data);

void SetSpeedAndTargetLane(vector< vector<double> > & sensor_fusion,
						   const int path_size,
						   double & car_s_val,
						   const double & car_d,
						   double & ref_velocity,
						   const int & cur_lane,
						   int & target_lane,
						   vehicle_state & v_State);


#endif /* SRC_PATH_HPP_ */
