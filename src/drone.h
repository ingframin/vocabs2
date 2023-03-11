#ifndef DRONE_H
#define DRONE_H
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct flight_plan
{
  vec2 *waypoints;//flight plan (array of waypoints that rescales automagically when adding new waypoints)
  size_t wp_len;  //flight plan length
  size_t curr_wp; //Index of the current waypoint

}FlightPlan;

typedef struct
{
  uint32_t id;      //Unique ID
  vec2 position;    // current position
  vec2 velocity;    //current velocity
  double size;      //Physical size of the drone
  FlightPlan* fp;   //stack of susequent waypoints to be reached
  
} Drone;



//Initialize a new drone.
Drone DR_newDrone(double x, double y, double vx, double vy, double size);
//Free memory used by a drone
void DR_freeDrone(Drone *d);
//move the drone by velocity x time delta
void DR_move(Drone *d, double dt);
//Steer towards next waypoint (It doesn't move the drone!!)
void DR_goto(Drone *d, vec2 waypoint);
//Are the drones on a collision route?
bool DR_collision(Drone *d1, Drone *d2);
//Compute avoidance maneuver and add escape waypoint
void DR_avoid(Drone *d, Drone *d2, double error);
//Instead of computing an avoidance maneuver waits until no collision is imminent
void DR_stopAndWait(Drone *d, Drone *d2, double error);
//Compute avoidance for a list of drones
void DR_avoidMany(Drone *d, Drone *drones, uint32_t ndrones, double error);

//Initialize a new fligth plan
FlightPlan* FP_newFlightPlan(size_t length);
//Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
//Increase the waypoint array size if needed
void FP_push_waypoint(FlightPlan *fp, vec2 wp);
//pop removes the top waypoints (but it does not shrink the waypoint array)
vec2 FP_pop_waypoint(FlightPlan *fp);
//Free memory used by a FlightPlan
void FP_free_FlightPlan(FlightPlan* fp);
#endif
