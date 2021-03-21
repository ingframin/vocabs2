#include "drone.h"
#include <vector>
#include <cmath>
#include "vec2.h"
#define MAX_ANGLE 0.01

static uint32_t ids = 0;

double generateGaussian(double mean, double stdDev)
{
	static double spare;
	static bool hasSpare = false;

	if (hasSpare)
	{
		hasSpare = false;
		return spare * stdDev + mean;
	}
	else
	{
		double u, v, s;
		do
		{
			u = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
			v = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
			s = u * u + v * v;
		} while (s >= 1.0 || s == 0.0);
		s = sqrt(-2.0 * log(s) / s);
		spare = v * s;
		hasSpare = true;
		return mean + stdDev * u * s;
	}
}

Obstacle compute_obstacle(Drone& d1, Drone& d2)
{
	// Minkowski addition
	double r = d1.size + d2.size;

	//Computing tangent lines to circle passing through the point self.position
	double dx = d1.position.x - d2.position.x;
	double a = dx * dx - r * r;
	double b = 2 * dx * (d1.position.y - d2.position.y);
	double c = (d2.position.y - d1.position.y) * (d2.position.y - d1.position.y) - r * r;
	double Delta = b * b - 4 * a * c;

	//Angular coefficient
	double m1 = (-b + sqrt(Delta)) / (2 * a);
	double m2 = (-b - sqrt(Delta)) / (2 * a);
	//Intersection with y axis
	double q1 = d1.position.y - m1 * d1.position.x;
	double q2 = d1.position.y - m2 * d1.position.x;

	//(xt1,yt1) - first tangent point.
	double a1 = 1 + m1 * m1;
	double b1 = 2 * m1 * q1 - 2 * d2.position.x - m1 * 2 * d2.position.y;

	double xt1 = (-b1) / (2 * a1);
	double yt1 = m1 * xt1 + q1;

	//(xt2,yt2) - Second tangent point
	double a2 = 1 + m2 * m2;
	double b2 = 2 * m2 * q2 - 2 * d2.position.x - m2 * 2 * d2.position.y;

	double xt2 = (-b2) / (2 * a2);
	double yt2 = m2 * xt2 + q2;

	//Construct obstacle
	Obstacle o;
	o.position = d2.position;
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

Drone::Drone(double x, double y, double vx, double vy, double size){
	id = ids;
	ids++;
	position.x = x;
	position.y = y;
	speed.x = vx;
	speed.y = vy;
	_speed_mod = speed.mod();
	
	waypoints.push_back({0,0});
	waypoints[0].x = x;
	waypoints[0].y = y;
	this->size = size;

}


void Drone::move(double dt){
	if (position.distance(waypoints.back()) < size)
	{
		popWaypoint();
	}

	steer(waypoints.back());

	vec2 dP = speed.prodK(dt);
	position = position.add(dP);
	
}



void Drone::steer(vec2& waypoint)
{

	vec2 dir = speed.norm();

	vec2 dirp = waypoint.sub(position);

	dirp = dirp.norm();

	double C = dir.dot(dirp);

	double angle = 0;

	if (C < -0.9999)
	{
		speed.x = -speed.x;
		speed.y = -speed.y;
	}
	else if (C >= -0.9999 && C < 0.9999)
	{
		angle = -acos(C);
		speed = speed.rotate(angle);
	}
}

bool Drone::collision(Drone& d2)
{
	Obstacle o = compute_obstacle(*this, d2);

	vec2 dif = speed.sub(d2.speed);
	if (dif.mod() > 1.9 * _speed_mod)
	{
		return true;
	}
	vec2 ds = dif.add(position);

	Barycoords bc = barycentric(position, o.T2, o.T1, ds);

	if (bc.alpha > 0 && bc.beta > 0 && bc.gamma > 0)
	{
		return true;
	}

	return false;
}

void Drone::avoid(Drone& d2, double error)
{
	Drone dx = d2;
	if (error > 0)
	{
		vec2 pos_error;
		pos_error.x = generateGaussian(0, 5);
		pos_error = pos_error.rotate(2 * std::numbers::pi * rand() / RAND_MAX);

		dx.position = dx.position.add(pos_error);
	}

	if (collision(dx))
	{

		vec2 dir = speed.norm();
		double theta = atan2(dir.y, dir.x);
		vec2 p2rel = dx.position.sub(position);
		double thetaP2 = atan2(p2rel.y, p2rel.x);
		if (fabs(thetaP2) > fabs(theta))
		{
			vec2 escape = speed.rotateHalfPI(-1);
			escape = escape.norm();
			escape = escape.prodK(4 * (size + dx.size));
			escape = escape.add(position);
			pushWaypoint(escape);
		}
	}
}

void Drone::stopAndWait(Drone& d2, double error)
{
	Drone dx = d2;
	if (error > 0)
	{
		vec2 pos_error;
		pos_error.x = generateGaussian(0, error);
		pos_error = pos_error.rotate(2 * std::numbers::pi * rand() / RAND_MAX);

		dx.position = dx.position.add(pos_error);
	}

	if (collision(dx))
	{

		if (dx.id > id)
		{

			speed.x = 0;
			speed.y = 0;
		}
	}
	else
	{
		speed.x = _speed_mod;
		speed.y = 0;
	}
}

void Drone::pushWaypoint(vec2 wp)
{	
	waypoints.push_back(wp);
}

void Drone::popWaypoint()
{
	waypoints.pop_back();
}


