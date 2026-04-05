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
#include "math2d.h"
#include "flightplan.h"

// Initialize static member
uint64_t Drone::next_id = 0;

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


// Drone constructor
Drone::Drone(double x, double y, double vx, double vy, double size)
{
	// Validate and clamp input parameters
	if (isnan(x) || isinf(x)) x = 0.0;
	if (isnan(y) || isinf(y)) y = 0.0;
	if (isnan(vx) || isinf(vx)) vx = 0.0;
	if (isnan(vy) || isinf(vy)) vy = 0.0;
	
	// Validate size parameter
	if (size <= 0 || isnan(size) || isinf(size)) {
		size = DEFAULT_DRONE_SIZE;
	} else if (size < MIN_DRONE_SIZE) {
		size = MIN_DRONE_SIZE;
	}
	
	id = next_id;
	next_id++;
	position.x = x;
	position.y = y;
	velocity.x = vx;
	velocity.y = vy;
	fp = new FlightPlan(4);
	if (fp == NULL) {
		// Handle memory allocation failure
		fp = new FlightPlan(2); // Try smaller size
		if (fp == NULL) {
			// If still fails, create minimal drone
			this->size = size;
			return;
		}
	}
	this->size = size;
}

// Default constructor
Drone::Drone() : id(0), size(DEFAULT_DRONE_SIZE), fp(NULL) {
	position.x = 0.0;
	position.y = 0.0;
	velocity.x = 0.0;
	velocity.y = 0.0;
}

// Drone destructor
Drone::~Drone()
{
	if (fp != NULL) {
		delete fp;
		fp = NULL;
	}
}

// Copy constructor
Drone::Drone(const Drone& other)
{
	id = next_id;
	next_id++;
	position = other.position;
	velocity = other.velocity;
	size = other.size;
	// Create a new flight plan and copy waypoints
	if (other.fp != NULL) {
		fp = new FlightPlan(other.fp->getLength());
		if (fp != NULL) {
			// Copy waypoints manually
			for (int64_t i = 0; i < other.fp->getCurrentWp() + 1; i++) {
				fp->pushWaypoint(other.fp->getWaypoints()[i]);
			}
		}
	} else {
		fp = NULL;
	}
}

// Assignment operator
Drone& Drone::operator=(const Drone& other)
{
	if (this == &other) {
		return *this;
	}
	
	position = other.position;
	velocity = other.velocity;
	size = other.size;
	
	// Handle flight plan
	if (fp != NULL) {
		delete fp;
	}
	
	// Create a new flight plan and copy waypoints
	if (other.fp != NULL) {
		fp = new FlightPlan(other.fp->getLength());
		if (fp != NULL) {
			// Copy waypoints manually
			for (int64_t i = 0; i < other.fp->getCurrentWp() + 1; i++) {
				fp->pushWaypoint(other.fp->getWaypoints()[i]);
			}
		}
	} else {
		fp = NULL;
	}
	
	return *this;
}

void Drone::move(double dt)
{
	// Validate inputs
	if (fp == NULL) return;
	if (dt <= 0 || isnan(dt) || isinf(dt)) return;
	
	//This has to become a new function
	if (position.distanceTo(fp->currentWp()) < size)
	{
		fp->popWaypoint();
	}
	
	gotoWaypoint(fp->currentWp());

	vec2 dP = velocity * dt;
	position = position + dP;
}

void Drone::gotoWaypoint(vec2 waypoint)
{
	vec2 dirp = waypoint - position;
	
	// Handle zero vector case
	if (dirp.isZero(1e-12)) {
		// If already at waypoint, keep current velocity
		return;
	}

	dirp = dirp.normalize();
	double vmod = velocity.mod();

	velocity = dirp * vmod;
}

bool Drone::collision(const Drone* d2) const
{
	// Validate inputs
	if (d2 == NULL) return false;
	
	// Handle case where drones have same position
	if ((position - d2->position).isZero(1e-12)) {
		return true; // Already colliding
	}
	
	Obstacle o = compute_obstacle(position, d2->position, size, d2->size);
	double speed = velocity.mod();
	
	// Handle zero speed case
	if (speed < MIN_DRONE_SPEED) {
		// If drone is not moving, check distance only
		return position.distanceTo(d2->position) < (size + d2->size);
	}
	
	vec2 dif = velocity - d2->velocity;
	if (dif.mod() > 1.9 * speed)
	{
		return true;
	}
	vec2 ds = dif + position;

	barycoords bc = v2_barycentric(position, o.T2, o.T1, ds);

	if (bc.alpha > 0 && bc.beta > 0 && bc.gamma > 0)
	{
		return true;
	}

	return false;
}

void Drone::avoid(const Drone* d2, double error)
{
	// Validate inputs
	if (d2 == NULL || fp == NULL) return;
	if (error < 0) error = 0; // Negative error doesn't make sense
	
	/*
	* This needs a complete rewrite...
	*/
	Drone dx = *d2;

	//This should be part of the communication rather than the avoidance function
	if (error > 0)
	{
		vec2 pos_error= generateGaussian2D(0, error);
		
		// Validate generated error vector
		if (isnan(pos_error.x) || isnan(pos_error.y) || isinf(pos_error.x) || isinf(pos_error.y)) {
			pos_error = (vec2){0, 0};
		}
		
		dx.setPosition(dx.getPosition() + pos_error);
	}

	if (collision(&dx))
	{
		/*The goal of this code is to only calculate the maneuver if the incoming drone
		* is on the leftof the current drone.
		* This is not actually necessary and both drones can share the rsponsibility of avoiding each other.
		* This algorithm also does not take into account the presence of other drones.
		*/
		vec2 dir = velocity.normalize();
		// This whole mess could just be replaced by a coordinate conversion.
		// Each drone should place the others in its own frame of reference.
		// In reality this could be something like East North Up (or Down) coordinates rather than Earth Centered Earth Fixed coordinates.
		// On a larger scale, it might even be worth using something like WGS84.
		
		// Handle zero velocity case
		if (dir.isZero(1e-12)) {
			// If not moving, just add a waypoint to stay clear
			vec2 escape = position - dx.getPosition();
			if (!escape.isZero(1e-12)) {
				escape = escape.normalize();
				escape = escape * (2 * (size + dx.getSize()));
				escape = escape + position;
				fp->pushWaypoint(escape);
			}
			return;
		}
		
		double theta = atan2(dir.y, dir.x);
		vec2 p2rel = dx.getPosition() - position;
		double thetaP2 = atan2(p2rel.y, p2rel.x);

		if (fabs(thetaP2) > fabs(theta))
		{
			vec2 escape = velocity.rotateLeftHalfPI().reverse();
			// If only C had function composition like Haskell...
			escape = escape.normalize();
			if (!isnan(escape.x) && !isnan(escape.y)) {
				escape = escape * (4 * (size + dx.getSize()));
				escape = escape + position;
				fp->pushWaypoint(escape);
			}
		}
	}
}

void Drone::stopAndWait(const Drone* d2, double error)
{
	// Validate inputs
	if (d2 == NULL) return;
	if (error < 0) error = 0; // Negative error doesn't make sense
	
	Drone dx = *d2;
	double speed = velocity.mod();
	
	// Handle invalid speed
	if (isnan(speed) || isinf(speed)) {
		speed = 0;
	}
	
	if (error > 0)
	{
		vec2 pos_error;
		pos_error.x = generateGaussian(0, error);
		pos_error = pos_error.rotate(2 * M_PI * rand() / RAND_MAX);
		
		// Validate generated error
		if (isnan(pos_error.x) || isnan(pos_error.y) || isinf(pos_error.x) || isinf(pos_error.y)) {
			pos_error = (vec2){0, 0};
		}
		
		dx.setPosition(dx.getPosition() + pos_error);
	}

	if (collision(&dx))
	{
		if (dx.getId() > id)
		{
			velocity.x = 0;
			velocity.y = 0;
		}
	}
	else
	{
		// Only restore speed if it was valid
		if (!isnan(speed) && !isinf(speed)) {
			velocity.x = speed;
			velocity.y = 0;
		}
	}
}

// DroneSystem constructor
DroneSystem::DroneSystem(size_t num_drones, double speed) {
    length = num_drones;
    drones = new Drone[num_drones];
    
    // Initialize drone positions and waypoints
    vec2 start_positions[] = {{0.0, 0.0}, {1000.0, 0.0}};
    
    for (size_t i = 0; i < num_drones; i++) {
        drones[i] = Drone(
            start_positions[i % 2].x, 
            start_positions[i % 2].y, 
            speed, 0.0, 1
        );
    }
}

// DroneSystem destructor
DroneSystem::~DroneSystem() {
    if (drones != NULL) {
        delete[] drones;
        drones = NULL;
    }
    length = 0;
}