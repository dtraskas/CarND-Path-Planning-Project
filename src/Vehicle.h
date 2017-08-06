#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:
  Vehicle(double id, double x, double y, double vx, double vy, double s, double d);              
  ~Vehicle() = default;
  double GetSDistance(double s);    
  double GetVelocity();

  double id;
  double vx;
  double vy;
  double vvx;
  double vvy;
  double vs;
  double vd;
};

#endif //VEHICLE_H
