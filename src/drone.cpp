#include "drone.h"
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI   3.14159265358979323846264338327950288
#endif

#include "vec2.h"
#include "obstacles.h"

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



Drone::Drone(double x, double y, double vx, double vy, double size){
	id = ids;
	ids++;
	position.x = x;
	position.y = y;
	velocity.x = vx;
	velocity.y = vy;
		
	waypoints.push_back({0,0});
	waypoints[0].x = x;
	waypoints[0].y = y;
	this->size = size;

}
Drone::Drone(const Drone& d2){
	id = d2.id;
	position = d2.position;
	velocity = d2.velocity;
	for(auto wp : d2.waypoints){
		waypoints.push_back(wp);
	}
	size = d2.size;
}

void Drone::move(double dt){
	if (position.distance(waypoints.back()) < size)
	{
		popWaypoint();
	}

	steer(waypoints.back());

	vec2 dP = velocity.prodK(dt);
	position = position.add(dP);
	
}



void Drone::steer(const vec2& waypoint)
{

	vec2 dir = velocity.norm();

	vec2 dirp = waypoint.sub(position);

	dirp = dirp.norm();

	double C = dir.dot(dirp);

	double angle = 0;

	if (C < -0.9999)
	{
		velocity.x = -velocity.x;
		velocity.y = -velocity.y;
	}
	else if (C >= -0.9999 && C < 0.9999)
	{
		angle = -acos(C);
		velocity = velocity.rotate(angle);
	}
}

bool Drone::collision(const Drone& d2) const
{
	Obstacle o = compute_obstacle(radius(), d2.radius(),currentPosition(),d2.currentPosition());

	vec2 dif = currentVelocity().sub(d2.currentVelocity());
	if (dif.mod() > 1.9 * currentVelocity().mod())
	{
		return true;
	}
	vec2 ds = dif.add(currentPosition());

	Barycoords bc = barycentric(currentPosition(), o.T2, o.T1, ds);

	if (bc.alpha > 0 && bc.beta > 0 && bc.gamma > 0)
	{
		return true;
	}

	return false;
}

void Drone::avoid(const Drone& d2, double error)
{
	Drone dx {d2};
	if (error > 0)
	{
		vec2 pos_error;
		pos_error.x = generateGaussian(0, 5);
		pos_error = pos_error.rotate(2 *M_PI * rand() / RAND_MAX);

		dx.position = dx.position.add(pos_error);
	}

	if (collision(dx))
	{

		vec2 dir = velocity.norm();
		double theta = atan2(dir.y, dir.x);
		vec2 p2rel = dx.position.sub(position);
		double thetaP2 = atan2(p2rel.y, p2rel.x);
		if (fabs(thetaP2) > fabs(theta))
		{
			vec2 escape = velocity.rotateHalfPI(-1);
			escape = escape.norm();
			escape = escape.prodK(4 * (size + dx.size));
			escape = escape.add(position);
			pushWaypoint(escape);
		}
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
