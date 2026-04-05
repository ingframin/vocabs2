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
#include "math2d.h"
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "flightplan.h"

// Minimum valid values for drones
#define MIN_DRONE_SIZE 0.1          // Minimum drone size in meters
#define MIN_DRONE_SPEED 1e-6       // Minimum drone speed in m/s
#define MAX_DRONE_SPEED 100.0      // Maximum drone speed in m/s
#define DEFAULT_DRONE_SIZE 1.0    // Default drone size in meters

class Drone {
private:
    uint64_t id;      /**< Unique ID */
    vec2 position;    /**< Current position */
    vec2 velocity;    /**< Current velocity */
    double size;      /**< Physical size of the drone (meters) */
    FlightPlan* fp;   /**< Stack of subsequent waypoints to be reached */
    
    static uint64_t next_id;

public:
    // Default constructor
    Drone();
    
    // Constructor
    Drone(double x, double y, double vx, double vy, double size);
    
    // Destructor
    ~Drone();
    
    // Copy constructor
    Drone(const Drone& other);
    
    // Assignment operator
    Drone& operator=(const Drone& other);
    
    // Getters
    uint64_t getId() const { return id; }
    vec2 getPosition() const { return position; }
    vec2 getVelocity() const { return velocity; }
    double getSize() const { return size; }
    FlightPlan* getFlightPlan() const { return fp; }
    
    // Setters
    void setPosition(vec2 pos) { position = pos; }
    void setVelocity(vec2 vel) { velocity = vel; }
    void setSize(double s) { size = s; }
    
    // Drone operations
    void move(double dt);
    void gotoWaypoint(vec2 waypoint);
    bool collision(const Drone* d2) const;
    void avoid(const Drone* d2, double error);
    void stopAndWait(const Drone* d2, double error);
};

// Drone system class to manage multiple drones
class DroneSystem {
private:
    Drone* drones;  // Array of drones
    size_t length;   // Number of drones in the array
    
public:
    // Constructor
    DroneSystem(size_t num_drones, double speed);
    
    // Destructor
    ~DroneSystem();
    
    // Getters
    Drone* getDrones() const { return drones; }
    size_t getLength() const { return length; }
    
    // Access individual drones
    Drone& operator[](size_t index) { return drones[index]; }
    const Drone& operator[](size_t index) const { return drones[index]; }
};

#endif