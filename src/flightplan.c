
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

FlightPlan* FP_newFlightPlan(int64_t length){
	// Validate input parameters
	if (length <= 0) {
		length = MIN_FLIGHTPLAN_LENGTH;
	}
	
	// Allocate memory for flight plan structure
	FlightPlan* fp = (FlightPlan*)malloc(sizeof(FlightPlan));
	if (fp == NULL) {
		return NULL; // Memory allocation failed
	}
	
	// Allocate memory for waypoints array
	fp->waypoints = (vec2*)malloc((size_t)length * sizeof(vec2));
	if (fp->waypoints == NULL) {
		// Cleanup on failure
		free(fp);
		return NULL; // Memory allocation failed
	}
	
	// Initialize flight plan
	fp->current_wp = -1; // Start with empty flight plan
	fp->length = length;
	
	return fp;
}

//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp){
	// Validate input
	if (fp == NULL || fp->waypoints == NULL) {
		return;
	}
	
	fp->current_wp += 1;
	if (fp->current_wp == (fp->length - 1))
	{
		// Calculate new length (grow by 50%)
		int64_t new_length = fp->length + (fp->length >> 1);
		
		// Reallocate waypoints array
		vec2* new_waypoints = (vec2*)realloc(fp->waypoints, (size_t)new_length * sizeof(vec2));
		if (new_waypoints == NULL) {
			// Reallocation failed - keep existing array but prevent overflow
			fp->current_wp -= 1;
			return;
		}
		
		fp->waypoints = new_waypoints;
		fp->length = new_length;
	}
	
	// Add the waypoint
	fp->waypoints[fp->current_wp] = wp;
}

//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp){
	// Validate input
	if (fp == NULL || fp->waypoints == NULL) {
		vec2 ZERO = {0, 0};
		return ZERO;
	}
	
	if(fp->current_wp < 0){
		// Already empty
		vec2 ZERO = {0, 0};
		return ZERO;
	}
    
	fp->current_wp -= 1;
    
	if(fp->current_wp < 0){
		fp->current_wp = -1;
		vec2 ZERO = {0, 0};
        return ZERO;
    }
    
	return fp->waypoints[fp->current_wp];    
}

vec2 FP_current_wp(FlightPlan* fp){
	// Validate input
	if (fp == NULL || fp->waypoints == NULL) {
		vec2 ZERO = {0, 0};
		return ZERO;
	}
	
	if(fp->current_wp < 0){
		// Flight plan is empty, return zero vector
		vec2 ZERO = {0, 0};
		return ZERO;
	}
	
	// Check bounds
	if (fp->current_wp >= fp->length) {
		fp->current_wp = fp->length - 1;
	}
	
	return fp->waypoints[fp->current_wp];
}

//Are there waypoints in the FlightPlan?
bool FP_isFlightPlanEmpty(FlightPlan* fp){
	// Validate input
	if (fp == NULL) {
		return true;
	}
	return (fp->current_wp < 0);
}

void FP_free_FlightPlan(FlightPlan* fp){
	// Validate input
	if (fp == NULL) {
		return;
	}
	
	// Free waypoints array if it exists
	if (fp->waypoints != NULL) {
		free(fp->waypoints);
		fp->waypoints = NULL;
	}
	
	// Free flight plan structure
	free(fp);
}