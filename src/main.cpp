
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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <omp.h>
#include <vector>
#include <cstring>
#include "math2d.h"
#include "flightplan.h"
#include "drone.h"
#include "comms.h"
#include "main.h"

#ifdef TEST
#include "test_vec2.h"
#include "test_flightplan.h"

int main(int argc, char** argv){
  bool tests[] = {
    test_v2_mod(),
    // test_v2_rotateHalfPI(),
    test_v2_rotate(),
    test_newFlightPlan(),
    test_push_waypoint(),
    test_pop_waypoint()
  };
  for(int i =0; i<6;i++){
    printf("T%d pass: %s\n",i,tests[i] ? "true" : "false");
  }
  

  return 0;
}
#else
omp_lock_t writelock;
/* This function is too big and needs to be split into multiple functions */

int main(int argc, char *argv[])
{

  
  // Parse command line arguments first to check for config file override
  parseArguments(argc, argv);
  
  // Load configuration from file (if not already loaded via -c/--config)
  // Check if we need to load the default config
  bool need_default_config = true;
  for (int i = 1; i < argc; i++) {
      if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) {
          need_default_config = false;
          break;
      }
  }
  
  if (need_default_config) {
      load_config();
  }
  
  std::vector<double> collisions(sim_context.len_rates, 0.0);
  omp_init_lock(&writelock);
  t = time(NULL);
  srand(t);

  // speed is in m/s

  vec2 p1 = {500.0, 500.0};
  vec2 p2 = {1000.0, 1000.0};
  

  printf("Rate:\tPcrash:\n");
  omp_set_num_threads(sim_context.num_threads);
//The simulation code should go into a separate function
#pragma omp parallel
  {

    for (uint32_t i = 0; i < sim_context.len_rates; i++)
    {
      printf("rate: %.2f \n", 1000.0/sim_context.rates[i]);

#pragma omp for
      for (uint32_t it = 0; it < sim_context.iterations; it++)
      {
        // Initialize drone system with 2 drones
        DroneSystem drone_system(2, sim_context.speed);
        
        // Set up waypoints for each drone
        for (size_t d = 0; d < drone_system.getLength(); d++) {
            drone_system[d].getFlightPlan()->pushWaypoint(p2);
            drone_system[d].getFlightPlan()->pushWaypoint(p1);
        }
        
        sim_context.rate = sim_context.rates[i];
        //Timer should be part of the simulation object
        bool running = true;
        uint64_t timer = 0;

        while (running)
        {

          if (timer >= sim_context.rate)
          {

            // if (COM_broadcast(drone_system.drones[0].position, drone_system.drones[1].position, sys, l))
            if (COM_broadcast_Pint(sim_context.Ptx, sim_context.Prx, sim_context.Pint))
            {
              
              // Avoid other drones
              for (size_t d1 = 0; d1 < drone_system.getLength(); d1++) {
                for (size_t d2 = 0; d2 < drone_system.getLength(); d2++) {
                  if (d1 != d2) {
                    drone_system[d1].avoid(&drone_system[d2], sim_context.error);
                  }
                }
              }
              
            }
            timer = 0;
          }

          // Move all drones
          for (size_t d = 0; d < drone_system.getLength(); d++) {
            drone_system[d].move(sim_context.dt);
          }
          
          
          // Check if any drone has completed its flight plan
          bool all_drones_have_waypoints = true;
          for (size_t d = 0; d < drone_system.getLength(); d++) {
            if (drone_system[d].getFlightPlan()->isEmpty()) {
              all_drones_have_waypoints = false;
              break;
            }
          }
          if (!all_drones_have_waypoints)
          {
            running = false;
          }


          //This should be improved for better collision detection
          // Check for collisions between any pair of drones
          bool collision_detected = false;
          for (size_t d1 = 0; d1 < drone_system.getLength() && !collision_detected; d1++) {
            for (size_t d2 = d1 + 1; d2 < drone_system.getLength() && !collision_detected; d2++) {
              if (drone_system[d1].getPosition().distanceTo(drone_system[d2].getPosition()) < (drone_system[d1].getSize() + drone_system[d2].getSize())) 
              {
                omp_set_lock(&writelock);
                collisions[i] += 1;
                omp_unset_lock(&writelock);
                collision_detected = true;
                running = false;
              }
            }
          }

          timer += 1;
        }//while
        // printf("Iter: %d",it);
        // DroneSystem destructor will automatically clean up memory
      } //iterations
      if (collisions[i] == 0.0)
      {
        printf("%.3f\n", 1000.0/sim_context.rate);
        break;
      }
    } //rates

  } //openmp

  
  save_results("pinterf_0.0.txt", collisions.data(), sim_context.rates, sim_context.len_rates, sim_context.iterations, sim_context.error, sim_context.l, sim_context.speed, sim_context.prob);
  return 0;
}
#endif
