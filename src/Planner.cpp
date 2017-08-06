#include <fstream>
#include <math.h>
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "States.h"
#include "Planner.h"
#include "HighwayMap.h"
#include "Vehicle.h"
#include "Utils.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void Planner::Update(vector<double> localisation, vector<vector<double>> sensor_fusion, vector<double> prev_path_x, vector<double> prev_path_y, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
    // PHASE ONE - Get ego vehicle localisation data
    double car_x = localisation[0];
    double car_y = localisation[1];
    double car_s = localisation[2];
    double car_d = localisation[3];
    double car_yaw = localisation[4];
    double car_speed = localisation[5];

    // PHASE TWO - get sensor fusion data for all the vehicles on the highway and determine the closest one
    vehicles.clear();
    for(int index = 0; index < sensor_fusion.size(); ++index) {
        double id = sensor_fusion[index][0];
        double vx = sensor_fusion[index][1];
        double vy = sensor_fusion[index][2];
        double vvx = sensor_fusion[index][3];
        double vvy = sensor_fusion[index][4];
        double vs = sensor_fusion[index][5];
        double vd = sensor_fusion[index][6];
        Vehicle vehicle(id, vx, vy, vvx, vvy, vs, vd);
        vehicles.push_back(vehicle);    
    }    
    int closest_vehicle = ClosestVehicle(car_d, car_s, 30);
    
    // PHASE FOUR - Start planning    
    if(previous_state.size() == 0) {        
        ego_state = { KEEP_LANE, GetLane(car_d) };
        vector<double> vehicle_state = {car_s, 0, 0, car_d, 0, 0};
        GenerateTrajectory(vehicle_state, closest_vehicle, next_x_vals, next_y_vals, previous_state);
    } else {				
        int idx = previous_state.size() - prev_path_x.size();
        vector<double> vehicle_state = {previous_state[idx][0], previous_state[idx][1], previous_state[idx][2], previous_state[idx][3], previous_state[idx][4], previous_state[idx][5]};
        previous_state.clear();
        double cost = GenerateTrajectory(vehicle_state, closest_vehicle, next_x_vals, next_y_vals, previous_state);
        if(cost == 1.0) {            
            next_x_vals.clear(); 
            next_y_vals.clear();
            for(int index = 0; index < prev_path_x.size(); ++index) {
                next_x_vals.push_back(prev_path_x[index]);
                next_y_vals.push_back(prev_path_y[index]);
            }
        }                            
    }
}

int Planner::ClosestVehicle(double vd, double vs, double dist)
{    	
	int index = -1;
	double min_ds = dist;
	for(int idx = 0; idx < vehicles.size(); ++idx) {
		if(GetLane(vd) == GetLane(vehicles[idx].vd)) {		
            double d = vehicles[idx].vs - vs;
            if(d < 0 && (vs + dist) > max_s) {
                double ds = vs + dist - max_s;
                if(vehicles[idx].vs < ds && (vehicles[idx].vs + max_s-vs) < min_ds) {
                    min_ds = vehicles[idx].vs + max_s - vs;
                    index = idx;
                }			
            } else if( d > 0 && d < min_ds ) {
                min_ds = d;
                index = idx;
            }
        }
	}
	return index;
}

int Planner::GenerateTrajectory(const vector<double> &vehicle_state, const int closest_vehicle, vector<double> &next_x, vector<double> &next_y, vector<vector<double> > &next_sva)
{    
	double cost = 0.0;
	unsigned int next_state = ego_state[0];
	
	double local_car_s, s0;
    tk::spline  spline_fit_sx;
	tk::spline	spline_fit_sy;
	tk::spline	spline_fit_sdx;
	tk::spline	spline_fit_sdy;
	
	FitWaypoints(vehicle_state[0], local_car_s, s0, spline_fit_sx, spline_fit_sy, spline_fit_sdx, spline_fit_sdy);	
	vector<double> s_start = {local_car_s, vehicle_state[1], vehicle_state[2]};
	vector<double> d_start = {vehicle_state[3], vehicle_state[4], vehicle_state[5]};
	double max_dist_per_step = 0.4;
	
	double goal_s = s_start[0] + max_dist_per_step * TRAJECTORY_STEPS;
	
	if(vehicle_state[1] < max_dist_per_step / 2.5) {
		goal_s = s_start[0] + max_dist_per_step * TRAJECTORY_STEPS / 2.0;
		max_dist_per_step /= 2.0;
	}
	vector<double> s_goal = {goal_s, max_dist_per_step, 0};
	
	double goal_d = 2.0;
	if(vehicle_state[3] > 8.0) {
		goal_d = 9.5;
	} else if(vehicle_state[3] > 4.0) {
		goal_d = 6.25;
	}
	
	vector<double> d_goal = {goal_d, 0, 0};
	Vehicle veh = vehicles[closest_vehicle];
    double velocity = veh.GetVelocity();

	if(ego_state[0] == CHANGE_LEFT) {		
		if(GetLane(vehicle_state[3]) == ego_state[1]) {
			next_state = KEEP_LANE;
		} else {
			d_goal[0] = ego_state[1]*4.0 + 2.0;
		}
	} else if(ego_state[0] == CHANGE_RIGHT) {		
		if(GetLane(vehicle_state[3]) == ego_state[1]) {
			next_state = KEEP_LANE;
		} else {
			d_goal[0] = ego_state[1]*4.0 + 2.0;
		}
	}
    else if(closest_vehicle >= 0 && velocity < max_dist_per_step) {   		
    	if(CanChangeLaneToLeft(vehicle_state[3], vehicle_state[0], vehicle_state[2])) {
    		d_goal[0] -= 4.0;
    		next_state = CHANGE_LEFT;
    		ego_state[1] = GetLane(d_goal[0]);
    	}
    	else if(CanChangeLaneToRight(vehicle_state[3], vehicle_state[0], vehicle_state[2])) {
    		d_goal[0] += 4.0;
    		next_state = CHANGE_RIGHT;
    		ego_state[1] = GetLane(d_goal[0]);
    	} else {     		
    		double ds = veh.GetSDistance( vehicle_state[0] );  
            s_goal[0] = s_start[0] + ds + velocity * TRAJECTORY_STEPS - 20;
    		s_goal[1] = velocity;
		}   	
    } else {    	
    	next_state = KEEP_LANE;
    }
	
    auto jmt_s_params = JMT(s_start, s_goal, TRAJECTORY_STEPS);
	auto jmt_d_params = JMT(d_start, d_goal, TRAJECTORY_STEPS);
    ego_state[0] = next_state;
	
	double previous_s = 0;
	int num_reuse = 0;
	
	double px = 0.0;
	double py = 0.0;
	for(int t = 0; t < TRAJECTORY_STEPS / 2; ++t) {	
		double s   = QuinticS(jmt_s_params, t);
		double s_v = QuinticD(jmt_s_params, t);
		double s_a = QuinticA(jmt_s_params, t);
		
		if(t < num_reuse) {  	
			s   = next_sva[t - 1][0] - s0;
			if(s < 0) s += max_s;			
			s_v = next_sva[t - 1][1];
			s_a = next_sva[t - 1][2];
		}
		
		double d = QuinticS(jmt_d_params, t);
		double d_v = QuinticD(jmt_d_params, t);
		double d_a = QuinticA(jmt_d_params, t);
		
		if(t > 0 && (s - previous_s) > max_dist_per_step) {
			s = previous_s + max_dist_per_step;
		}
		
		auto xy = FitXY(s, d, spline_fit_sx, spline_fit_sy, spline_fit_sdx, spline_fit_sdy);
		if(t > 0) { 
			double dxy = GetDistance(px, py, xy[0], xy[1]);
			if(dxy > max_dist_per_step) {				
				while(dxy > max_dist_per_step) {
					s -= 0.01;
					xy = FitXY(s, d, spline_fit_sx, spline_fit_sy, spline_fit_sdx, spline_fit_sdy);
					dxy = GetDistance(px, py, xy[0], xy[1]);
				}
			}
		}
		
		previous_s = s;		
		next_x.push_back(xy[0]);
		next_y.push_back(xy[1]);
		px = xy[0];
		py = xy[1];
		
		s += s0;
		if(s > max_s) s -= max_s;
		next_sva.push_back({s, s_v, s_a, d, d_v, d_a, xy[0], xy[1]});
	}
	return cost;
}

vector<Vehicle> Planner::GetVehiclesInLane(double car_s, double car_d, double dist)
{    
	double s1, s2;  
	vector<Vehicle> vehicles_found;
	
	s1 = car_s -dist;
	if(s1 < 0) {
		s1 += max_s;
	}
	s2 = car_s +dist;
	if(s2 > max_s) {
		s2 -= max_s;
	}
	
	for(int i = 0; i < vehicles.size(); ++i) {
		if(GetLane(car_d) == GetLane(vehicles[i].vd)) {			
            if(s2 > s1 && vehicles[i].vs > s1 && vehicles[i].vs < s2) {
                vehicles_found.push_back(vehicles[i]);
            }
            else if(s1 > s2 && (vehicles[i].vs < s1 || vehicles[i].vs > s2)) {
                vehicles_found.push_back(vehicles[i]);
            }
        }
	}
	return vehicles_found;
}

bool Planner::CanChangeLaneToLeft(double car_d, double car_s, double car_speed)
{
	if (GetLane(car_d) == 0) {
		return false; 
    } else {
        vector<Vehicle> vehicles_found = GetVehiclesInLane(car_s, car_d - 4.0, 25);	
        if(vehicles_found.empty()) 
            return true;
        else
            return false;
    }
}

bool Planner::CanChangeLaneToRight(double car_d, double car_s, double car_speed)
{	
	if (GetLane(car_d) == 2)
    {
		return false; 
    } else {
        vector<Vehicle> vehicles_found = GetVehiclesInLane(car_s, car_d + 4.0, 25);	
        if(vehicles_found.empty()) 
            return true;
        else
            return false;
    }	
}

vector<double> Planner::FitWaypointsS(double vs, const vector<double>& map_s, double &local_vs, vector<int>& map_idx, int num_prev, int num_next)
{    
	if(vs > map_s[map_s.size()-1]) {
		vs = map_s[map_s.size()-1];
	}

	auto it = lower_bound(map_s.begin(), map_s.end(), vs);
	int idx = it - map_s.begin();
	
	for(int index = num_prev; index > 0; --index) {
		map_idx.push_back((map_s.size()+idx-index) % map_s.size());
	}
	
	for(int index = 0; index < num_next; ++index) {
		map_idx.push_back((idx+index) % map_s.size());
	}
	
	vector<double> local_map_waypoints_s;
	
	double s0 = map_s[map_idx[0]];
	local_map_waypoints_s.push_back(0.0);
	
	for(int index = 1; index < map_idx.size(); ++index) {
		double ds = map_s[map_idx[index]] - s0;
		
		if(ds < 0) { 
			ds = map_s[map_idx[index]] + max_s - s0;
		}
		local_map_waypoints_s.push_back(ds);
	}
	
	if(vs < s0) {
		local_vs = vs + max_s - s0;
	} else {
		local_vs = vs - s0;
	}
	
	return local_map_waypoints_s;
}

void Planner::FitWaypoints(double vs, double& local_vs, double& s0,  
                           tk::spline &spline_fit_sx, tk::spline &spline_fit_sy, tk::spline &spline_fit_sdx, tk::spline	&spline_fit_sdy)
{
	vector<int>	map_s_idx;
	
	auto map_fit_s = FitWaypointsS(vs, highwayMap.map_waypoints_s, local_vs, map_s_idx, 15, 15);
	s0 = highwayMap.map_waypoints_s[map_s_idx[0]];

	vector<double> map_fit_x;
	vector<double> map_fit_y;
	vector<double> map_fit_dx;
	vector<double> map_fit_dy;
	
	for(int i = 0; i < map_s_idx.size(); ++i) {		
		map_fit_x.push_back(highwayMap.map_waypoints_x[map_s_idx[i]]);
		map_fit_y.push_back(highwayMap.map_waypoints_y[map_s_idx[i]]);
		map_fit_dx.push_back(highwayMap.map_waypoints_dx[map_s_idx[i]]);
		map_fit_dy.push_back(highwayMap.map_waypoints_dy[map_s_idx[i]]);
	}
	
	spline_fit_sx.set_points(map_fit_s, map_fit_x);
	spline_fit_sy.set_points(map_fit_s, map_fit_y);
	spline_fit_sdx.set_points(map_fit_s, map_fit_dx);
	spline_fit_sdy.set_points(map_fit_s, map_fit_dy);
}

vector<double> Planner::JMT(vector< double> start, vector <double> end, double T)
{
    MatrixXd A(3, 3);
    A << pow(T, 3), pow(T, 4), pow(T, 5),
         3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
         6*T, 12*pow(T, 2), 20*pow(T, 3);
    
    VectorXd B(3);
    B[0] = end[0] - (start[0]+start[1]*T+start[2]*pow(T, 2)/2.0);
    B[1] = end[1] - (start[1]+start[2]*T);
    B[2] = end[2] - start[2];
    
    VectorXd x(3);
    x = A.inverse()*B;
         
    return {start[0],start[1],start[2]/2.0,x[0],x[1],x[2]};
}

vector<double> Planner::FitXY(double s, double d, const tk::spline &spline_fit_sx, const tk::spline &spline_fit_sy, const tk::spline &spline_fit_sdx, const tk::spline &spline_fit_sdy)
{
	double x = spline_fit_sx(s);
	double y = spline_fit_sy(s);
	double dx = spline_fit_sdx(s);
	double dy = spline_fit_sdy(s);
	
	x += dx*d;
	y += dy*d;
	
	return {x, y};
}