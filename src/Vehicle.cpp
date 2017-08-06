#include "Vehicle.h"
#include <math.h>

Vehicle::Vehicle(double id, double x, double y, double vx, double vy, double s, double d) : id(id), vx(x), vy(y), vvx(vx), vvy(vy), vs(s), vd(d) {}

double Vehicle::GetSDistance(double s)
{
    double max_s = 6945.554;
    double d = vs - s;
    if(d < 0) d = vs + max_s - s;    
    return d;
}

double Vehicle::GetVelocity() 
{
    return sqrt(vvx * vvx + vvy * vvy) * 0.02;
}