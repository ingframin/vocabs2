
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

#ifndef FLIGHTPLAN_H
#define FLIGHTPLAN_H
#include "math2d.h"
#include <stdint.h>
#include <stdbool.h>

// Minimum valid values for flight plans
#define MIN_FLIGHTPLAN_LENGTH 2  // Minimum waypoints array length

typedef struct flight_plan
{
  vec2 *waypoints; /**< Array of waypoints (resizes automatically) */
  int64_t length;  /**< Current allocated length of waypoints array */
  int64_t current_wp; /**< Index of the current waypoint (-1 = empty) */

}FlightPlan;

/**
 * @brief Initialize a new flight plan
 * @param length Initial capacity of waypoints array (must be > 0)
 * @return Pointer to new FlightPlan, or NULL if allocation fails
 */
FlightPlan* FP_newFlightPlan(int64_t length);

/**
 * @brief Add a waypoint to the flight plan (LIFO stack)
 * @param fp Pointer to FlightPlan
 * @param wp Waypoint to add
 */
void FP_push_waypoint(FlightPlan *fp, vec2 wp);

/**
 * @brief Remove and return the top waypoint
 * @param fp Pointer to FlightPlan
 * @return Top waypoint, or (0,0) if empty
 */
vec2 FP_pop_waypoint(FlightPlan *fp);

/**
 * @brief Get the current waypoint without removing it
 * @param fp Pointer to FlightPlan
 * @return Current waypoint, or (0,0) if empty
 */
vec2 FP_current_wp(FlightPlan* fp);

/**
 * @brief Check if flight plan is empty
 * @param fp Pointer to FlightPlan
 * @return True if empty, false otherwise
 */
bool FP_isFlightPlanEmpty(FlightPlan* fp);

/**
 * @brief Free memory used by a FlightPlan
 * @param fp Pointer to FlightPlan to free (can be NULL)
 */
void FP_free_FlightPlan(FlightPlan* fp);
#endif