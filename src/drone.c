#include "drone.h"
#include <stdlib.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include "vec2.h"
#include "obstacle.h"
#define MAX_ANGLE 0.01
#include "flightplan.h"

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

inline vec2 generateGaussian2D(double mean, double stdDev)
{
	
	double u, v, s;
	do
	{
		u = (rand()/((double)RAND_MAX)) * 2.0 - 1.0;
		v = (rand()/((double)RAND_MAX)) * 2.0 - 1.0;
		s = u * u + v * v;
	} while (s >= 1.0 || s == 0.0);
	s = sqrt(-2.0 * log(s) / s);
	
	vec2 ret = {mean + stdDev * u * s, mean + stdDev * v * s};
	return ret;
	
}


Drone DR_newDrone(double x, double y, double vx, double vy, double size)
{
	Drone d;
	d.id = ids;
	ids++;
	d.position.x = x;
	d.position.y = y;
	d.velocity.x = vx;
	d.velocity.y = vy;
	d.fp = FP_newFlightPlan(4);
	// FP_push_waypoint(d.fp,d.position);
	d.size = size;
	return d;
}

void DR_move(Drone *d, double dt)
{
	//This has to become a new function
	if (v2_distance(d->position, FP_current_wp(d->fp)) < d->size)
	{
		// printf("D%d Reached: %.3f,%.3f\n", d->id, d->waypoints[d->current_wp].x, d->waypoints[d->current_wp].y);
		FP_pop_waypoint(d->fp);
	}

	DR_goto(d,FP_current_wp(d->fp));

	vec2 dP = v2_scale(d->velocity, dt);
	d->position = v2_add(d->position, dP);
}

void DR_goto(Drone *d, vec2 waypoint)
{

	vec2 dir = v2_normalize(d->velocity);

	vec2 dirp = v2_diff(waypoint, d->position);

	dirp = v2_normalize(dirp);

	double angle = v2_angle_between(dirp,dir);

	d->velocity = v2_rotate(d->velocity, angle);
}

bool DR_collision(Drone *d1, Drone *d2)
{
	Obstacle o = compute_obstacle(d1->position, d2->position,d1->size,d2->size);
	double speed = v2_mod(d1->velocity);
	vec2 dif = v2_diff(d1->velocity, d2->velocity);
	if (v2_mod(dif) > 1.9 * speed)
	{
		return true;
	}
	vec2 ds = v2_add(dif, d1->position);

	barycoords bc = barycentric(d1->position, o.T2, o.T1, ds);

	if (bc.alpha > 0 && bc.beta > 0 && bc.gamma > 0)
	{
		return true;
	}

	return false;
}

void DR_avoid(Drone *d, Drone *d2, double error)
{
	Drone dx = *d2;
	if (error > 0)
	{
		vec2 pos_error;
		pos_error.x = generateGaussian(0, 5);
		pos_error = v2_rotate(pos_error, 2 * M_PI * rand() / RAND_MAX);

		dx.position = v2_add(dx.position, pos_error);
	}

	if (DR_collision(d, &dx))
	{

		vec2 dir = v2_normalize(d->velocity);
		double theta = atan2(dir.y, dir.x);
		vec2 p2rel = v2_diff(dx.position, d->position);
		double thetaP2 = atan2(p2rel.y, p2rel.x);
		if (fabs(thetaP2) > fabs(theta))
		{
			vec2 escape = v2_rotateHalfPI(d->velocity, -1);
			escape = v2_normalize(escape);
			escape = v2_scale(escape, 4 * (d->size + dx.size));
			escape = v2_add(escape, d->position);
			FP_push_waypoint(d->fp, escape);
		}
	}
}

void DR_stopAndWait(Drone *d, Drone *d2, double error)
{
	Drone dx = *d2;
	double speed = v2_mod(d->velocity);
	if (error > 0)
	{
		vec2 pos_error;
		pos_error.x = generateGaussian(0, error);
		pos_error = v2_rotate(pos_error, 2 * M_PI * rand() / RAND_MAX);

		dx.position = v2_add(dx.position, pos_error);
	}

	if (DR_collision(d, &dx))
	{

		if (dx.id > d->id)
		{

			d->velocity.x = 0;
			d->velocity.y = 0;
		}
	}
	else
	{
		d->velocity.x = speed;
		d->velocity.y = 0;
	}
}

void DR_freeDrone(Drone *d)
{
	FP_free_FlightPlan(d->fp);
	free(d);
}

