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
	d.speed.x = vx;
	d.speed.y = vy;
	d._speed_mod = v2_mod(d.speed);
	d.waypoints = malloc(2 * sizeof(vec2));
	d.wp_len = 2;
	d.curr_wp = 0;
	d.waypoints[0].x = x;
	d.waypoints[0].y = y;
	d.size = size;
	return d;
}

void DR_move(Drone *d, double dt)
{

	if (v2_distance(d->position, d->waypoints[d->curr_wp]) < d->size)
	{
		// printf("D%d Reached: %.3f,%.3f\n", d->id, d->waypoints[d->curr_wp].x, d->waypoints[d->curr_wp].y);
		DR_pop_waypoint(d);
	}

	DR_goto(d, d->waypoints[d->curr_wp]);

	vec2 dP = v2_scale(d->speed, dt);
	d->position = v2_add(d->position, dP);
}

void DR_goto(Drone *d, vec2 waypoint)
{

	vec2 dir = v2_norm(d->speed);

	vec2 dirp = v2_sub(waypoint, d->position);

	dirp = v2_norm(dirp);

	double C = v2_dot(dir, dirp);

	double angle = 0;

	if (C < -0.9999)
	{
		//angle = M_PI;
		d->speed.x = -d->speed.x;
		d->speed.y = -d->speed.y;
	}
	else if (C >= -0.9999 && C < 0.9999)
	{
		angle = -acos(C);
		d->speed = v2_rotate(d->speed, angle);
	}
}

bool DR_collision(Drone *d1, Drone *d2)
{
	Obstacle o = compute_obstacle(d1->position, d2->position,d1->size,d2->size);

	vec2 dif = v2_sub(d1->speed, d2->speed);
	if (v2_mod(dif) > 1.9 * d1->_speed_mod)
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

		vec2 dir = v2_norm(d->speed);
		double theta = atan2(dir.y, dir.x);
		vec2 p2rel = v2_sub(dx.position, d->position);
		double thetaP2 = atan2(p2rel.y, p2rel.x);
		if (fabs(thetaP2) > fabs(theta))
		{
			vec2 escape = v2_rotateHalfPI(d->speed, -1);
			escape = v2_norm(escape);
			escape = v2_scale(escape, 4 * (d->size + dx.size));
			escape = v2_add(escape, d->position);
			DR_push_waypoint(d, escape);
		}
	}
}

void DR_stopAndWait(Drone *d, Drone *d2, double error)
{
	Drone dx = *d2;
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

			d->speed.x = 0;
			d->speed.y = 0;
		}
	}
	else
	{
		d->speed.x = d->_speed_mod;
		d->speed.y = 0;
	}
}

void DR_push_waypoint(Drone *d, vec2 wp)
{

	if (d->curr_wp == (d->wp_len - 1))
	{
		d->wp_len = (d->wp_len + (d->wp_len >> 1));
		d->waypoints = realloc(d->waypoints, (d->wp_len + (d->wp_len >> 1)) * sizeof(vec2));
	}
	d->curr_wp += 1;
	d->waypoints[d->curr_wp] = wp;
}
void DR_pop_waypoint(Drone *d)
{
	d->curr_wp = (d->curr_wp > 0) ? d->curr_wp - 1 : 0;
}

void DR_freeDrone(Drone *d)
{
	free(d->waypoints);
	free(d);
}

//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp){
	if (fp->curr_wp == (fp->wp_len - 1))
	{
		fp->wp_len = (fp->wp_len + (fp->wp_len >> 1));
		fp->waypoints = realloc(fp->waypoints, DROUND(fp->wp_len*1.5) * sizeof(vec2));
	}
	fp->curr_wp += 1;
	fp->waypoints[fp->curr_wp] = wp;

}

//pop removes the top waypoints (but it does not shrink the waypoint array)
vec2 FP_pop_waypoint(FlightPlan *fp){
	vec2 ret = fp->waypoints[fp->curr_wp];
	fp->curr_wp = (fp->curr_wp > 0) ? fp->curr_wp - 1 : 0;
	return ret;

}