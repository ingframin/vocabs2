#include "fligthplan.h"
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>


FlightPlan* FP_newFlightPlan(size_t length){
	FlightPlan* fp = malloc(sizeof(FlightPlan));
	fp->waypoints = malloc(length*sizeof(vec2));
	fp->curr_wp = 0;
	fp->wp_len = length;
    fp->empty = true;
	return fp;
}

//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp){
	if (fp->curr_wp == (fp->wp_len - 1))
	{
		fp->wp_len = (fp->wp_len + (fp->wp_len >> 1));
		fp->waypoints = realloc(fp->waypoints, ((size_t)floor(fp->wp_len*1.5)) * sizeof(vec2));
	}
	fp->curr_wp += 1;
	fp->waypoints[fp->curr_wp] = wp;
    fp->empty = false;
}

//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp){
	vec2 ret = fp->waypoints[fp->curr_wp];
    if(fp->curr_wp == 0){
        fp->empty = true;
        
    }
    else{
        fp->curr_wp - 1;
    }
	
	return ret;

}

vec2 FP_current_wp(FlightPlan* fp){
	return fp->waypoints[fp->curr_wp];
}

void FP_free_FlightPlan(FlightPlan* fp){
	free(fp->waypoints);
	free(fp);
}