#include "obstacle.h"
#include <math.h>

Obstacle compute_obstacle(vec2 pos1, vec2 pos2, double size1, double size2)
{
	// Minkowski addition
	double r = size1 + size2;

	//Computing tangent lines to circle passing through the point self.position
	double dx = pos1.x - pos2.x;
	double a = dx * dx - r * r;
	double b = 2 * dx * (pos1.y - pos2.y);
	double c = (pos2.y - pos1.y) * (pos2.y - pos1.y) - r * r;
	double Delta = b * b - 4 * a * c;

	//Angular coefficient
	double m1 = (-b + sqrt(Delta)) / (2 * a);
	double m2 = (-b - sqrt(Delta)) / (2 * a);
	//Intersection with y axis
	double q1 = pos1.y - m1 * pos1.x;
	double q2 = pos1.y - m2 * pos1.x;

	//(xt1,yt1) - first tangent point.
	double a1 = 1 + m1 * m1;
	double b1 = 2 * m1 * q1 - 2 * pos2.x - m1 * 2 * pos2.y;

	double xt1 = (-b1) / (2 * a1);
	double yt1 = m1 * xt1 + q1;

	//(xt2,yt2) - Second tangent point
	double a2 = 1 + m2 * m2;
	double b2 = 2 * m2 * q2 - 2 * pos2.x - m2 * 2 * pos2.y;

	double xt2 = (-b2) / (2 * a2);
	double yt2 = m2 * xt2 + q2;

	//Construct obstacle
	Obstacle o;
	o.position = d2->position;
	o.radius = r;
	o.T1.x = xt1;
	o.T1.y = yt1;
	o.T2.x = xt2;
	o.T2.y = yt2;
	// printf("T1.x= %.3f,T1.y= %.3f\n",o.T1.x,o.T1.y);
	return o;
}

barycoords barycentric(vec2 A, vec2 B, vec2 C, vec2 P)
{
	barycoords bc;
	bc.gamma = ((A.y - B.y) * P.x + (B.x - A.x) * P.y + A.x * B.y - B.x * A.y) /
			   ((A.y - B.y) * C.x + (B.x - A.x) * C.y + A.x * B.y - B.x * A.y);
	bc.beta = ((A.y - C.y) * P.x + (C.x - A.x) * P.y + A.x * C.y - C.x * A.y) /
			  ((A.y - C.y) * B.x + (C.x - A.x) * B.y + A.x * C.y - C.x * A.y);
	bc.alpha = 1 - bc.beta - bc.gamma;

	return bc;
}