#include "path.hpp"
#include "helper.hpp"

int GetCurrentLane(double d)
{
	int cur_lane = (int)(d/4.0);
	if(cur_lane >= 0 && cur_lane < LANES_NO)
	{
		return cur_lane;
	}
	else
	{
		return -1;
	}
}

void DefineVehiclePath(vector<double> & next_x_vals,
					   vector<double> & next_y_vals,
					   const vector<double> & prev_path_x,
					   const vector<double> & prev_path_y,
					   const vector<double> & map_waypoints_x,
					   const vector<double> & map_waypoints_y,
					   const vector<double> & map_waypoints_s,
					   const vehicle_data & car_data)
{
	vector<double> ptsx, ptsy;

	double ref_x = car_data.x;
	double ref_y = car_data.y;
	double ref_angle = deg2rad(car_data.yaw);

	int path_size = prev_path_x.size();

	if(path_size < 2)
	{
		double p_x = car_data.x - cos(car_data.yaw);
		double p_y = car_data.y - sin(car_data.yaw);

		ptsx.push_back(p_x);
		ptsy.push_back(p_y);

		ptsx.push_back(car_data.x);
		ptsy.push_back(car_data.y);

	}
	else
	{
		ref_x = prev_path_x[path_size-1];
		ref_y = prev_path_y[path_size-1];

		double ref_x_prev = prev_path_x[path_size-2];
		double ref_y_prev = prev_path_y[path_size-2];

		if((fabs(ref_x - ref_x-ref_x_prev) > 0.001)  && (fabs(ref_y-ref_y_prev) > 0.001))
		{
			ref_angle = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
			ptsx.push_back(ref_x_prev);
			ptsy.push_back(ref_y_prev);
		}

		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y);
	}

	for(int dist = 30; dist <= 90; dist += 30)
	{
		int target_dist;
		if(car_data.target_lane != car_data.cur_lane)
		{
			target_dist = int((car_data.ref_vel/30) * dist);
		}
		else
		{
			target_dist = dist;
		}
		vector<double> waypnt = getXY(car_data.s + target_dist,
									  (2+4*car_data.target_lane),
									  map_waypoints_s,
									  map_waypoints_x,
									  map_waypoints_y);

		ptsx.push_back(waypnt[0]);
		ptsy.push_back(waypnt[1]);
	}

	for(int idx = 0; idx < ptsx.size(); idx++)
	{
		double shift_x = ptsx[idx] - ref_x;
		double shift_y = ptsy[idx] - ref_y;

		ptsx[idx] = (shift_x * cos(0-ref_angle)) - (shift_y * sin(0-ref_angle));
		ptsy[idx] = (shift_x * sin(0-ref_angle)) + (shift_y * cos(0-ref_angle));

	}

	tk::spline s;

	s.set_points(ptsx, ptsy);

	double target_x = TARGET_DISTANCE;
	double target_y = s(target_x);
	double target_dis = distance(0, 0, target_x, target_y);
	double x_add_on = 0;

	for(int idx = 0 ; idx < prev_path_x.size(); idx++)
	{
		next_x_vals.push_back(prev_path_x[idx]);
		next_y_vals.push_back(prev_path_y[idx]);
	}

	for(int idx = 0 ; idx < (EST_PATH_PTS_NO - prev_path_x.size()); idx++)
	{
		double N = target_dis / ((0.02 * car_data.ref_vel)/2.24);
		double pnt_x = x_add_on + (target_x / N);
		double pnt_y = s(pnt_x);

		x_add_on = pnt_x;

		double xref = pnt_x;
		double yref = pnt_y;

		pnt_x = (xref * cos(ref_angle)) - (yref * sin(ref_angle));
		pnt_y = (xref * sin(ref_angle)) + (yref * cos(ref_angle));

		pnt_x += ref_x;
		pnt_y += ref_y;

		next_x_vals.push_back(pnt_x);
		next_y_vals.push_back(pnt_y);

	}

	return;
}

void SetSpeedAndTargetLane(vector< vector<double> > & sensor_fusion,
						   int path_size,
						   double & car_s_val,
						   double & ref_velocity,
						   int & cur_lane,
						   int & target_lane,
						   vehicle_state & v_state)
{
  	bool too_close = false;
  	bool right_lane_available = false;
  	bool left_lane_available = false;

  	int closest_forward_left_car_s = 1000000;
  	int closest_forward_left_car_speed = 0;
  	int closest_forward_right_car_s = 1000000;
  	int closest_forward_right_car_speed = 0;
  	int forward_car_speed = 0;
  	int forward_car_s = 100000;

  	if(v_state == keep_lane)
  	{
  		std::cout << "Keep Lane" << std::endl;
  	}
  	else if(v_state == change_lane_left)
  	{
  		std::cout << "Change lane left" << std::endl;
  	}
  	else if(v_state == change_lane_right)
  	{
  		std::cout << "Change lane right" << std::endl;
  	}

  	left_lane_available = ((cur_lane + 1) == LANES_NO) ? false : true;
   	right_lane_available = ((cur_lane - 1) < 0) ? false : true;

  	for(int i = 0; i < sensor_fusion.size(); i++)
  	{
  		float d = sensor_fusion[i][6];
  		double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double x_car_speed = sqrt(vx*vx + vy*vy);
  		double x_car_s = sensor_fusion[i][5];

  		x_car_s += (0.02 * path_size * x_car_speed);

  		if((d < (4*cur_lane+4)) && (d > (4*cur_lane)))
  		{
  			if(x_car_s < forward_car_s && x_car_s > car_s_val)
  			{
  				forward_car_s = x_car_s;
  				forward_car_speed = x_car_speed;
  			}

  			if((x_car_s > car_s_val) && ((x_car_s - car_s_val) < 30))
  			{
  				too_close = true;
  			}
  		}
  		else
  		{
  			if((d > (4*cur_lane + 4)) && (d < (4*cur_lane + 8)) && left_lane_available)
  			{
  				if(((x_car_s > car_s_val) && ((x_car_s - car_s_val) < 35)) ||
  				   ((x_car_s < car_s_val) && ((car_s_val - x_car_s) < 10)))
  				{
  					left_lane_available = false;
  				}
  				else if (x_car_s > car_s_val && x_car_s < closest_forward_left_car_s)
				{
  					closest_forward_left_car_s = x_car_s;
  					closest_forward_left_car_speed = x_car_speed;
				}
  			}
  			else if((d > (4*cur_lane - 4)) && (d < (4*cur_lane)) && right_lane_available)
      		{
  				if(((x_car_s > car_s_val) && ((x_car_s - car_s_val) < 35)) ||
  				   ((x_car_s < car_s_val) && ((car_s_val - x_car_s) < 10)))
   				{
  					right_lane_available = false;
   				}
  				else if (x_car_s > car_s_val && x_car_s < closest_forward_right_car_s)
				{
  					closest_forward_right_car_s = x_car_s;
  					closest_forward_right_car_speed = x_car_speed;
				}
  			}
  		}
  	}

  	if(v_state == keep_lane)
  	{
  		if(too_close)
  		{
  			if(left_lane_available && !right_lane_available)
  			{
  				target_lane = cur_lane + 1;
  			    v_state = change_lane_left;
  			    ref_velocity -= (0.75 * REFERENCE_ACCELERATION);
  			}
  			else if(right_lane_available && !left_lane_available)
  			{
  				target_lane = cur_lane - 1;
  			    v_state = change_lane_right;
  			  ref_velocity -= (0.75 * REFERENCE_ACCELERATION);
  			}
  			else if(right_lane_available && left_lane_available)
  			{
  				if(closest_forward_right_car_speed > closest_forward_left_car_speed)
  				{
  					target_lane = cur_lane - 1;
  					v_state = change_lane_right;
  				}
  				else
  				{
      				target_lane = cur_lane + 1;
      			    v_state = change_lane_left;
  				}
  				ref_velocity -= (0.75 * REFERENCE_ACCELERATION);
  			}
  			else
  			{
  				if(ref_velocity > forward_car_speed && (forward_car_s - car_s_val) < 30)
  				{
  					ref_velocity -= (0.75 * REFERENCE_ACCELERATION);
  				}
  			}
  		}
      	else
        {
      		target_lane = cur_lane;

      		if(ref_velocity < REFERENCE_VELOCITY)
      		{
      			ref_velocity += REFERENCE_ACCELERATION;
      		}
   		}
  	}
  	else
  	{
  		if(target_lane == cur_lane)
  		{
  			v_state = keep_lane;
  			ref_velocity += REFERENCE_ACCELERATION;
  		}
  		ref_velocity -= (0.75 * REFERENCE_ACCELERATION);
  	}
}
