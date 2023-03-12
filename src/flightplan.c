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
	fp->current_wp = 0;
	fp->length = length;
    fp->empty = true;
	return fp;
}

//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp){
	if (fp->current_wp == (fp->length - 1))
	{
		fp->length = (fp->length + (fp->length >> 1));
		fp->waypoints = realloc(fp->waypoints, ((size_t)floor(fp->length*1.5)) * sizeof(vec2));
	}
	fp->current_wp += 1;
	fp->waypoints[fp->current_wp] = wp;
    fp->empty = false;
}

//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp){
	vec2 ret = fp->waypoints[fp->current_wp];
    if(fp->current_wp == 0){
        fp->empty = true;
        
    }
    else{
        fp->current_wp -= 1;
    }
	
	return ret;

}

vec2 FP_current_wp(FlightPlan* fp){
	return fp->waypoints[fp->current_wp];
}

void FP_free_FlightPlan(FlightPlan* fp){
	free(fp->waypoints);
	free(fp);
}