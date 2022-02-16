#include "obstacles.h"
#include <cmath>

Obstacle compute_obstacle(double d1_size, double d2_size, vec2 d1_position, vec2 d2_position)
{
	// Minkowski addition
	double r = d1_size + d2_size;

	//Computing tangent lines to circle passing through the point self.position
	double dx = d1_position.x - d2_position.x;
	double a = dx * dx - r * r;
	double b = 2 * dx * (d1_position.y - d2_position.y);
	double c = (d2_position.y - d1_position.y) * (d2_position.y - d1_position.y) - r * r;
	double Delta = b * b - 4 * a * c;

	//Angular coefficient
	double m1 = (-b + sqrt(Delta)) / (2 * a);
	double m2 = (-b - sqrt(Delta)) / (2 * a);
	//Intersection with y axis
	double q1 = d1_position.y - m1 * d1_position.x;
	double q2 = d1_position.y - m2 * d1_position.x;

	//(xt1,yt1) - first tangent point.
	double a1 = 1 + m1 * m1;
	double b1 = 2 * m1 * q1 - 2 * d2_position.x - m1 * 2 * d2_position.y;

	double xt1 = (-b1) / (2 * a1);
	double yt1 = m1 * xt1 + q1;

	//(xt2,yt2) - Second tangent point
	double a2 = 1 + m2 * m2;
	double b2 = 2 * m2 * q2 - 2 * d2_position.x - m2 * 2 * d2_position.y;

	double xt2 = (-b2) / (2 * a2);
	double yt2 = m2 * xt2 + q2;

	//Construct obstacle
	Obstacle o;
	o.position = d2_position;
	o.radius = r;
	o.T1.x = xt1;
	o.T1.y = yt1;
	o.T2.x = xt2;
	o.T2.y = yt2;
	
	return o;
}

Barycoords barycentric(vec2 A, vec2 B, vec2 C, vec2 P)
{
	Barycoords bc;
	bc.gamma = ((A.y - B.y) * P.x + (B.x - A.x) * P.y + A.x * B.y - B.x * A.y) /
			   ((A.y - B.y) * C.x + (B.x - A.x) * C.y + A.x * B.y - B.x * A.y);
	bc.beta = ((A.y - C.y) * P.x + (C.x - A.x) * P.y + A.x * C.y - C.x * A.y) /
			  ((A.y - C.y) * B.x + (C.x - A.x) * B.y + A.x * C.y - C.x * A.y);
	bc.alpha = 1 - bc.beta - bc.gamma;

	return bc;
}