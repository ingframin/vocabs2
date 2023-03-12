#ifndef TEST_FLIGHTPLAN
#define TEST_FLIGHTPLAN
#include "flightplan.h"
#include <stdbool.h>

//Initialize a new fligth plan
bool test_newFlightPlan();
//Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
//Increase the waypoint array size if needed
bool test_push_waypoint();
//pop "removes" the top waypoint (but it does not shrink the waypoint array)
//If the FlightPlan is empty, returns the 0-position waypoint
bool test_pop_waypoint();
//get the current waypoint
bool test_current_wp();
//Free memory used by a FlightPlan
bool test_free_FlightPlan();

#endif