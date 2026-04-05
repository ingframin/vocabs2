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

#include "flightplan.h"
#include "math2d.h"
#include <vector>

// Constructor
FlightPlan::FlightPlan(size_t initial_capacity) {
    waypoints.reserve(initial_capacity);
    current_wp = -1; // Start with empty flight plan
}

// Add a waypoint to the flight plan
void FlightPlan::pushWaypoint(vec2 wp){
    waypoints.push_back(wp);
    current_wp = waypoints.size() - 1;
}

// Pop "removes" the top waypoint
// If the FlightPlan is empty, returns the 0-position waypoint
vec2 FlightPlan::popWaypoint(){
	if(current_wp < 0){
		// Already empty
		vec2 ZERO = {0, 0};
		return ZERO;
	}
    
	if (!waypoints.empty()) {
		waypoints.pop_back();
	}
    
	current_wp = waypoints.empty() ? -1 : waypoints.size() - 1;
    
	if(current_wp < 0){
		vec2 ZERO = {0, 0};
        return ZERO;
    }
    
	return waypoints[current_wp];
}

vec2 FlightPlan::currentWp() const{
	if(current_wp < 0){
		// Flight plan is empty, return zero vector
		vec2 ZERO = {0, 0};
		return ZERO;
	}
	
	// Check bounds
	if (current_wp >= static_cast<int64_t>(waypoints.size())) {
		return waypoints.back();
	}
	
	return waypoints[current_wp];
}

// Are there waypoints in the FlightPlan?
bool FlightPlan::isEmpty() const{
	return (current_wp < 0);
}