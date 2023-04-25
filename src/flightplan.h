#ifndef FLIGHTPLAN_H
#define FLIGHTPLAN_H
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct flight_plan
{
  vec2 *waypoints;//flight plan (array of waypoints that rescales automagically when adding new waypoints)
  int64_t length;  //flight plan length
  int64_t current_wp; //Index of the current waypoint

}FlightPlan;

//Initialize a new fligth plan
FlightPlan* FP_newFlightPlan(int64_t length);
//Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp);
//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
vec2 FP_pop_waypoint(FlightPlan *fp);
//get the current waypoint
vec2 FP_current_wp(FlightPlan* fp);
//Are there waypoints in the FlightPlan?
bool FP_isFlightPlanEmpty(FlightPlan* fp);
//Free memory used by a FlightPlan
void FP_free_FlightPlan(FlightPlan* fp);
#endif