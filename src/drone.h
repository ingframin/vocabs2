
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

#ifndef DRONE_H
#define DRONE_H
#include "math2d.h"
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "flightplan.h"

// Minimum valid values for drones
#define MIN_DRONE_SIZE 0.1          // Minimum drone size in meters
#define MIN_DRONE_SPEED 1e-6       // Minimum drone speed in m/s
#define MAX_DRONE_SPEED 100.0      // Maximum drone speed in m/s
#define DEFAULT_DRONE_SIZE 1.0    // Default drone size in meters

typedef struct
{
  uint64_t id;      /**< Unique ID */
  vec2 position;    /**< Current position */
  vec2 velocity;    /**< Current velocity */
  double size;      /**< Physical size of the drone (meters) */
  FlightPlan* fp;   /**< Stack of subsequent waypoints to be reached */
  
} Drone;

// Drone system structure to manage multiple drones
typedef struct {
    Drone* drones;  // Array of drones
    size_t length;   // Number of drones in the array
} DroneSystem;

// Initialize a drone system with the specified number of drones
DroneSystem DRS_init_drone_system(size_t num_drones, double speed);

// Free memory allocated for a drone system
void DRS_free_drone_system(DroneSystem* system);

/**
 * @brief Initialize a new drone
 * @param x Initial x position
 * @param y Initial y position
 * @param vx Initial x velocity (m/s)
 * @param vy Initial y velocity (m/s)
 * @param size Drone size in meters (must be > 0)
 * @return Initialized Drone structure
 */
Drone DR_newDrone(double x, double y, double vx, double vy, double size);

/**
 * @brief Free memory used by a drone
 * @param d Pointer to drone to free (can be NULL)
 */
void DR_freeDrone(Drone *d);

/**
 * @brief Move the drone by velocity x time delta
 * @param d Pointer to drone
 * @param dt Time delta in seconds (must be > 0)
 */
void DR_move(Drone *d, double dt);

/**
 * @brief Steer towards next waypoint (does not move the drone)
 * @param d Pointer to drone
 * @param waypoint Target waypoint
 */
void DR_goto(Drone *d, vec2 waypoint);

/**
 * @brief Check if drones are on a collision route
 * @param d1 Pointer to first drone
 * @param d2 Pointer to second drone
 * @return True if collision is imminent, false otherwise
 */
bool DR_collision(Drone *d1, Drone *d2);

/**
 * @brief Compute avoidance maneuver and add escape waypoint
 * @param d Pointer to drone to maneuver
 * @param d2 Pointer to drone to avoid
 * @param error Positional error magnitude
 */
void DR_avoid(Drone *d, Drone *d2, double error);

/**
 * @brief Stop and wait avoidance strategy
 * @param d Pointer to drone to maneuver
 * @param d2 Pointer to drone to avoid
 * @param error Positional error magnitude
 */
void DR_stopAndWait(Drone *d, Drone *d2, double error);


#endif
