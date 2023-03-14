#include "flightplan.h"
#include "vec2.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

FlightPlan* FP_newFlightPlan(size_t length){
	FlightPlan* fp = malloc(sizeof(FlightPlan));
	if(fp == NULL){
		printf("failed init");
	}
	fp->waypoints = malloc(length*sizeof(vec2));
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
		fp->waypoints = realloc(fp->waypoints, ((size_t)ceil(fp->length*1.5)) * sizeof(vec2));
	}
	
	fp->waypoints[fp->current_wp] = wp;
	
    fp->empty = false;
}

//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp){
	vec2 ret;
    if(fp->current_wp <= 0){
        fp->empty = true;
		fp->current_wp = -1;
        ret = fp->waypoints[0];
    }
    else{
        fp->current_wp -= 1;
		ret = fp->waypoints[fp->current_wp];
    }
	
	return ret;

}

vec2 FP_current_wp(FlightPlan* fp){
	if(fp->current_wp <=0){
		return fp->waypoints[0];
	}
	return fp->waypoints[fp->current_wp];
}

//Are there waypoints in the FlightPlan?
bool FP_isFlightPlanEmpty(FlightPlan* fp){
	return (fp->current_wp >= 0)
}
void FP_free_FlightPlan(FlightPlan* fp){
	free(fp->waypoints);
	free(fp);
}