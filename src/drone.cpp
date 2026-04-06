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
#include <memory>
#include <stdlib.h>
#include <stdio.h>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include "math2d.h"
#include "flightplan.h"

// Initialize static member
uint64_t Drone::next_id = 0;

// Static random number generator for Gaussian distributions
static std::mt19937& getRandomEngine() {
    static std::random_device rd;
    static std::mt19937 engine(rd());
    return engine;
}

static inline double generateGaussian(double mean, double stdDev)
{
    std::normal_distribution<double> dist(mean, stdDev);
    return dist(getRandomEngine());
}

static inline vec2 generateGaussian2D(double mean, double stdDev)
{
    std::normal_distribution<double> dist(mean, stdDev);
    vec2 ret = {dist(getRandomEngine()), dist(getRandomEngine())};
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
	try {
		fp = std::make_unique<FlightPlan>(4);
	} catch (const std::bad_alloc&) {
		try {
			// Try smaller size if allocation fails
			fp = std::make_unique<FlightPlan>(2);
		} catch (const std::bad_alloc&) {
			// If still fails, create minimal drone without flight plan
			fp = nullptr;
		}
	}
	this->size = size;
}

// Default constructor
Drone::Drone() : id(0), size(DEFAULT_DRONE_SIZE), fp(nullptr) {
	position.x = 0.0;
	position.y = 0.0;
	velocity.x = 0.0;
	velocity.y = 0.0;
}

// Drone destructor
// Destructor - not needed with unique_ptr
Drone::~Drone() = default;

// Copy constructor
Drone::Drone(const Drone& other)
{
	id = next_id;
	next_id++;
	position = other.position;
	velocity = other.velocity;
	size = other.size;
	// Create a new flight plan and copy waypoints
	if (other.fp) {
		fp = std::make_unique<FlightPlan>(other.fp->getLength());
		// Copy waypoints manually
		for (int64_t i = 0; i < other.fp->getCurrentWp() + 1; i++) {
			fp->pushWaypoint(other.fp->getWaypoints()[i]);
		}
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
	
	// Create a new flight plan and copy waypoints
	if (other.fp) {
		fp = std::make_unique<FlightPlan>(other.fp->getLength());
		// Copy waypoints manually
		for (int64_t i = 0; i < other.fp->getCurrentWp() + 1; i++) {
			fp->pushWaypoint(other.fp->getWaypoints()[i]);
		}
	} else {
		fp = nullptr;
	}
	
	return *this;
}

// Move constructor
Drone::Drone(Drone&& other) noexcept
	: id(other.id), position(other.position), velocity(other.velocity), size(other.size), fp(std::move(other.fp))
{
	// Reset the source object to a valid state
	other.id = 0;
	other.position = {0, 0};
	other.velocity = {0, 0};
	other.size = DEFAULT_DRONE_SIZE;
	other.fp = nullptr;
}

// Move assignment operator
Drone& Drone::operator=(Drone&& other) noexcept
{
	if (this == &other) {
		return *this;
	}
	
	// Move all members
	position = other.position;
	velocity = other.velocity;
	size = other.size;
	fp = std::move(other.fp);
	
	// Reset the source object to a valid state
	other.id = 0;
	other.position = {0, 0};
	other.velocity = {0, 0};
	other.size = DEFAULT_DRONE_SIZE;
	other.fp = nullptr;
	
	return *this;
}

void Drone::move(double dt)
{
	// Validate inputs
	if (!fp || dt <= 0 || isnan(dt) || isinf(dt)) return;
	
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
	
	// Use proper velocity obstacle for collision detection
	Obstacle vo = compute_velocity_obstacle(position, velocity, 
	                                      d2->position, d2->velocity, 
	                                      size, d2->size);
	
	// Check if current velocity is inside the velocity obstacle
	// A velocity is collision-free if it's outside the VO cone
	
	// For simple collision detection, we can use barycentric coordinates
	// to check if the origin (zero relative velocity) is inside the VO
	barycoords bc = v2_barycentric(vo.T2, vo.T1, vo.position, vec2(0, 0));
	
	// If origin is inside the VO triangle, collision is imminent
	if (bc.alpha > 0 && bc.beta > 0 && bc.gamma > 0)
	{
		// Origin is inside VO - check distance for immediate collision
		return position.distanceTo(d2->position) < (size + d2->size);
	}
	
	// Additional check: if relative velocity points directly at obstacle
	vec2 rel_vel = d2->velocity - velocity;
	vec2 to_obstacle = vo.position - vec2(0, 0);
	
	// Check if relative velocity is in the direction of the obstacle
	if (rel_vel.dot(to_obstacle) > 0 && 
	    rel_vel.mod() > 1e-6 && 
	    to_obstacle.mod() > 1e-6)
	{
		// Compute angle between relative velocity and obstacle direction
		double cos_angle = rel_vel.dot(to_obstacle) / (rel_vel.mod() * to_obstacle.mod());
		if (cos_angle > 0.95) { // Within ~18 degrees
			// Check time to collision
			double distance = position.distanceTo(d2->position);
			double closing_speed = rel_vel.mod();
			if (closing_speed > 1e-6) {
				double time_to_collision = distance / closing_speed;
				if (time_to_collision < 2.0) { // Collision within 2 seconds
					return true;
				}
			}
		}
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
    // Initialize drone positions and waypoints
    vec2 start_positions[] = {{0.0, 0.0}, {1000.0, 0.0}};
    
    drones.reserve(num_drones);
    for (size_t i = 0; i < num_drones; i++) {
        drones.emplace_back(
            start_positions[i % 2].x, 
            start_positions[i % 2].y, 
            speed, 0.0, 1
        );
    }
}