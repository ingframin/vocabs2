
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
#include "vec2.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

FlightPlan* FP_newFlightPlan(int64_t length){
	FlightPlan* fp = (FlightPlan*)malloc(sizeof(FlightPlan));
	if(fp == NULL){
		printf("failed init");
	}
	fp->waypoints = (vec2*)malloc(length*sizeof(vec2));
	fp->current_wp = -1;
	fp->length = length;
	return fp;
}

//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp){
	fp->current_wp += 1;
	if (fp->current_wp == (fp->length - 1))
	{
		fp->length = (fp->length + (fp->length >> 1));
		fp->waypoints = (vec2*)realloc(fp->waypoints, ((size_t)ceil(fp->length*1.5)) * sizeof(vec2));
	}
	
	fp->waypoints[fp->current_wp] = wp;
	
}

//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp){
	
    if(fp->current_wp < 0){
        
		fp->current_wp = -1;
		vec2 ZERO;
        return ZERO;
    }
    
	int current = fp->current_wp;
	fp->current_wp -= 1;
	return fp->waypoints[current];    
}

vec2 FP_current_wp(FlightPlan* fp){
	if(fp->current_wp <0){
		return fp->waypoints[0];
	}
	return fp->waypoints[fp->current_wp];
}

//Are there waypoints in the FlightPlan?
bool FP_isFlightPlanEmpty(FlightPlan* fp){
	return (fp->current_wp < 0);
}
void FP_free_FlightPlan(FlightPlan* fp){
	free(fp->waypoints);
	free(fp);
}