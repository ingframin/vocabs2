#ifndef FLIGHTPLAN_H
#define FLIGTHPLAN_H
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct flight_plan
{
  vec2 *waypoints;//flight plan (array of waypoints that rescales automagically when adding new waypoints)
  size_t wp_len;  //flight plan length
  size_t curr_wp; //Index of the current waypoint
  bool empty;
}FlightPlan;

//Initialize a new fligth plan
FlightPlan* FP_newFlightPlan(size_t length);
//Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp);
//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp);
//get the current waypoint
vec2 FP_current_wp(FlightPlan* fp);
//Free memory used by a FlightPlan
void FP_free_FlightPlan(FlightPlan* fp);
#endif