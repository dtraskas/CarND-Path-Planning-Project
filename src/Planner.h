#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "spline.h"
#include "HighwayMap.h"
#include "Vehicle.h"

using namespace std;

class Planner {
public:    
    Planner(HighwayMap &_highwayMap) : highwayMap(_highwayMap) {};
    ~Planner() = default;
    void Update(vector<double> localisation, vector<vector<double>> sensor_fusion, vector<double> prev_path_x, vector<double> prev_path_y, vector<double> &next_x_vals, vector<double> &next_y_vals);
    int ClosestVehicle(double vd, double vs, double dist);
    int GenerateTrajectory(const vector<double> &vehicle_state, const int closest_vehicle, vector<double> &next_x, vector<double> &next_y, vector<vector<double> > &next_sva);
    
    vector<double> JMT(vector<double> start, vector <double> end, double T);
    vector<double> FitXY(double s, double d, const tk::spline &spline_fit_sx, const tk::spline &spline_fit_sy, const tk::spline &spline_fit_sdx, const tk::spline &spline_fit_sdy);
    void FitWaypoints(double vs, double& local_vs, double& s0, tk::spline &spline_fit_sx, tk::spline &spline_fit_sy, tk::spline &spline_fit_sdx, tk::spline &spline_fit_sdy);
    vector<double> FitWaypointsS(double vs, const vector<double>& map_s, double &local_vs, vector<int>& map_idx, int num_prev, int num_next);
    void MaxAccelerationAndJerk(vector<double> r, double dt, double &max_acc, double &max_jerk);

    bool CanChangeLaneToLeft(double car_d, double car_s, double car_speed);
    bool CanChangeLaneToRight(double car_d, double car_s, double car_speed);
    vector<Vehicle> GetVehiclesInLane(double car_s, double car_d, double dist);

private:
    double max_s = 6945.554;
    const int TRAJECTORY_STEPS = 250;

    vector<Vehicle> vehicles;
    vector<vector<double> > previous_state;
    vector<int> ego_state;        
    HighwayMap &highwayMap;
};

#endif //PLANNER_H