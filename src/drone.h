#ifndef DRONE_H
#define DRONE_H
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>



typedef struct
{
  uint32_t id;       //Unique ID
  vec2 position;     // current position
  vec2 speed;        //current speed
  double _speed_mod; //speed module
  double size;          //Physical size of the drone
  vec2 *waypoints;      //flight plan (array of waypoints that rescales automagically when adding new waypoints)
  uint64_t wp_len;  //flight plan length
  uint64_t curr_wp; //Index of the current waypoint
  
} Drone;

//Initialize a new drone.
Drone DR_newDrone(double x, double y, double vx, double vy, double size);
//Free memory used by a drone
void DR_freeDrone(Drone *d);
//move the drone by speed x time delta
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
//Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
//Increase the waypoint array size if needed
void DR_push_waypoint(Drone *d, vec2 wp);
//pop removes the top waypoints (but it does not shrink the waypoint array)
void DR_pop_waypoint(Drone *d);


#endif
