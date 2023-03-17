#ifndef DRONE_H
#define DRONE_H
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>
#include "flightplan.h"


typedef struct
{
  uint64_t id;      //Unique ID
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


#endif
