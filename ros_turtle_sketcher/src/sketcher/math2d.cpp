#include "sketcher/math2d.hpp"
#include <math.h>

namespace m2d {

double deg2rad(double radAngle)
{
	return radAngle * M_PI / 180;
}

double normalizeAngle(double x)
{
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0) {
        x += 2 * M_PI;
    }

    return x - M_PI;
}

double getDistance(double x1, double y1, double x2, double y2)
{
	double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
//	cout << "Distance: " << distance << endl;
	return distance;
}

}
