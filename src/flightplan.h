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

class FlightPlan {
private:
    vec2 *waypoints; /**< Array of waypoints (resizes automatically) */
    int64_t length;  /**< Current allocated length of waypoints array */
    int64_t current_wp; /**< Index of the current waypoint (-1 = empty) */

public:
    // Constructor
    FlightPlan(int64_t length);
    
    // Destructor
    ~FlightPlan();
    
    // Copy constructor
    FlightPlan(const FlightPlan& other);
    
    // Assignment operator
    FlightPlan& operator=(const FlightPlan& other);
    
    // Flight plan operations
    void pushWaypoint(vec2 wp);
    vec2 popWaypoint();
    vec2 currentWp() const;
    bool isEmpty() const;
    
    // Getters for internal state (for compatibility)
    vec2* getWaypoints() const { return waypoints; }
    int64_t getLength() const { return length; }
    int64_t getCurrentWp() const { return current_wp; }
};

#endif