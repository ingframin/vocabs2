
/* 
Vocabs2 - velocity obstacle for drones simulator
Copyright (C) 2023  Franco Minucci

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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
#include "flightplan.h"

static uint64_t ids = 0;

static inline double generateGaussian(double mean, double stdDev)
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

static inline vec2 generateGaussian2D(double mean, double stdDev)
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
	d.size = size;
	return d;
}

void DR_move(Drone *d, double dt)
{
	//This has to become a new function
	if (v2_distance(d->position, FP_current_wp(d->fp)) < d->size)
	{	
		FP_pop_waypoint(d->fp);
	}
	
	DR_goto(d,FP_current_wp(d->fp));

	vec2 dP = v2_scale(d->velocity, dt);
	d->position = v2_add(d->position, dP);
}

void DR_goto(Drone *d, vec2 waypoint)
{

	vec2 dirp = v2_diff(waypoint, d->position);

	dirp = v2_normalize(dirp);
	double vmod = v2_mod(d->velocity);

	d->velocity = v2_scale(dirp,vmod);
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

	barycoords bc = v2_barycentric(d1->position, o.T2, o.T1, ds);

	if (bc.alpha > 0 && bc.beta > 0 && bc.gamma > 0)
	{
		return true;
	}

	return false;
}

void DR_avoid(Drone *d, Drone *d2, double error)
{
	/*
	* This needs a complete rewrite...
	*/
	Drone dx = *d2;

	//This should be part of the communication rather than the avoidance function
	if (error > 0)
	{
		vec2 pos_error= generateGaussian2D(0, error);
		
		dx.position = v2_add(dx.position, pos_error);
	}

	if (DR_collision(d, &dx))
	{
		/*The goal of this code is to only calculate the maneuver if the incoming drone
		* is on the leftof the current drone.
		* This is not actually necessary and both drones can share the rsponsibility of avoiding each other.
		* This algorithm also does not take into account the presence of other drones.
		*/
		vec2 dir = v2_normalize(d->velocity);
		// This whole mess could just be replaced by a coordinate conversion.
		// Each drone should place the others in its own frame of reference.
		// In reality this could be something like East North Up (or Down) coordinates rather than Earth Centered Earth Fixed coordinates.
		// On a larger scale, it might even be worth using something like WGS84.
		double theta = atan2(dir.y, dir.x);
		vec2 p2rel = v2_diff(dx.position, d->position);
		double thetaP2 = atan2(p2rel.y, p2rel.x);

		if (fabs(thetaP2) > fabs(theta))
		{
			vec2 escape = v2_rotatePI(v2_rotateLeftHalfPI(d->velocity));
			// If only C had function composition like Haskell...
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
	
}

