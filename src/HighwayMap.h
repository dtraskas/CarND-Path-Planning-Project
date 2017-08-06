#ifndef HIGHWAY_MAP_H
#define HIGHWAY_MAP_H

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

class HighwayMap {
public:
  
  HighwayMap(std::string filename);
  ~HighwayMap() = default;

  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  std::vector<double> getFrenet(double x, double y, double theta);
  // Transform from Frenet s,d coordinates to Cartesian x,y
  std::vector<double> getXY(double s, double d);

public:
  double max_s;  
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
};

#endif //HIGHWAY_MAP_H
