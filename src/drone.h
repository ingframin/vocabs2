
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
