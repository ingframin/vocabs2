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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Constructor
FlightPlan::FlightPlan(int64_t length){
	// Validate input parameters
	if (length <= 0) {
		length = MIN_FLIGHTPLAN_LENGTH;
	}
	
	// Allocate memory for waypoints array
	waypoints = (vec2*)malloc((size_t)length * sizeof(vec2));
	if (waypoints == NULL) {
		// Handle memory allocation failure
		this->length = 0;
		current_wp = -1;
		return;
	}
	
	// Initialize flight plan
	current_wp = -1; // Start with empty flight plan
	this->length = length;
}

// Destructor
FlightPlan::~FlightPlan(){
	// Free waypoints array if it exists
	if (waypoints != NULL) {
		free(waypoints);
		waypoints = NULL;
	}
}

// Copy constructor
FlightPlan::FlightPlan(const FlightPlan& other) {
	length = other.length;
	current_wp = other.current_wp;
	
	// Allocate memory for waypoints array
	waypoints = (vec2*)malloc((size_t)length * sizeof(vec2));
	if (waypoints != NULL && other.waypoints != NULL) {
		// Copy waypoints
		for (int64_t i = 0; i <= other.current_wp; i++) {
			waypoints[i] = other.waypoints[i];
		}
	} else {
		waypoints = NULL;
		current_wp = -1;
	}
}

// Assignment operator
FlightPlan& FlightPlan::operator=(const FlightPlan& other) {
	if (this == &other) {
		return *this;
	}
	
	// Free existing waypoints
	if (waypoints != NULL) {
		free(waypoints);
	}
	
	// Copy data
	length = other.length;
	current_wp = other.current_wp;
	
	// Allocate memory for waypoints array
	waypoints = (vec2*)malloc((size_t)length * sizeof(vec2));
	if (waypoints != NULL && other.waypoints != NULL) {
		// Copy waypoints
		for (int64_t i = 0; i <= other.current_wp; i++) {
			waypoints[i] = other.waypoints[i];
		}
	} else {
		waypoints = NULL;
		current_wp = -1;
	}
	
	return *this;
}

// Increase the waypoint array size if needed
void FlightPlan::pushWaypoint(vec2 wp){
	current_wp += 1;
	if (current_wp == (length - 1))
	{
		// Calculate new length (grow by 50%)
		int64_t new_length = length + (length >> 1);
		
		// Reallocate waypoints array
		vec2* new_waypoints = (vec2*)realloc(waypoints, (size_t)new_length * sizeof(vec2));
		if (new_waypoints == NULL) {
			// Reallocation failed - keep existing array but prevent overflow
			current_wp -= 1;
			return;
		}
		
		waypoints = new_waypoints;
		length = new_length;
	}
	
	// Add the waypoint
	waypoints[current_wp] = wp;
}

// Pop "removes" the top waypoint (but it does not shrink the waypoint array)
// If the FlightPlan is empty, returns the 0-position waypoint
vec2 FlightPlan::popWaypoint(){
	if(current_wp < 0){
		// Already empty
		vec2 ZERO = {0, 0};
		return ZERO;
	}
    
	current_wp -= 1;
    
	if(current_wp < 0){
		current_wp = -1;
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
	
	// Check bounds - need to cast away const for this modification
	int64_t wp_index = current_wp;
	if (wp_index >= length) {
		wp_index = length - 1;
	}
	
	return waypoints[wp_index];
}

// Are there waypoints in the FlightPlan?
bool FlightPlan::isEmpty() const{
	return (current_wp < 0);
}