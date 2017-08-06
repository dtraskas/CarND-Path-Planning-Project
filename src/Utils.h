#ifndef UTILS_H
#define UTILS_H

int GetLane(double d)
{
	int lane_index = 0;
	if(d > 8.0) {
	    lane_index  = 2;
	} else if(d > 4.0) {
		lane_index  = 1;
	}	
	return lane_index;
}

double QuinticS(vector<double> p, double t)
{
	return p[0]+p[1]*t+p[2]*pow(t,2)+p[3]*pow(t,3)+p[4]*pow(t,4)+p[5]*pow(t,5);
}

double QuinticD(vector<double> p, double t)
{
	return p[1]+2*p[2]*t+3*p[3]*pow(t,2)+4*p[4]*pow(t,3)+5*p[5]*pow(t,4);
}

double QuinticA(vector<double> p, double t)
{
	return 2*p[2]+6*p[3]*t+12*p[4]*pow(t, 2)+20*p[5]*pow(t, 3);
}

double GetDistance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

#endif //UTILS_H