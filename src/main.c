
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

  
  // Load configuration from file
  load_config();
  
  // Parse command line arguments (can override config)
  parseArguments(argc,argv);
  
  double collisions[sim_context.len_rates];
  for (uint32_t k = 0; k < sim_context.len_rates; k++)
  {
    collisions[k] = 0.0;
  }
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
        DroneSystem drone_system = DRS_init_drone_system(2, sim_context.speed);
        
        // Set up waypoints for each drone
        for (size_t d = 0; d < drone_system.length; d++) {
            FP_push_waypoint(drone_system.drones[d].fp, p2);
            FP_push_waypoint(drone_system.drones[d].fp, p1);
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
            if (COM_broadcast_Pint(0.5,0.5,0.0))
            {
              
              DR_avoid(&drone_system.drones[1], &drone_system.drones[0], sim_context.error);
              DR_avoid(&drone_system.drones[0], &drone_system.drones[1], sim_context.error);
              
            }
            timer = 0;
          }

          DR_move(&drone_system.drones[0], sim_context.dt);
          DR_move(&drone_system.drones[1], sim_context.dt);
          
          
          if (FP_isFlightPlanEmpty(drone_system.drones[0].fp) || FP_isFlightPlanEmpty(drone_system.drones[1].fp))
          {
            running = false;
          }


          //This should be improved for better collision detection
          if (v2_distance(drone_system.drones[0].position, drone_system.drones[1].position) < (drone_system.drones[0].size + drone_system.drones[1].size)) 
          {
            omp_set_lock(&writelock);
            collisions[i] += 1;
            omp_unset_lock(&writelock);
            running = false;
          }

          timer += 1;
        }//while
        // printf("Iter: %d",it);
        //Fix for memory leak: flight plan within drones was allocated with malloc and never freed.
        DRS_free_drone_system(&drone_system);
      } //iterations
      if (collisions[i] == 0.0)
      {
        printf("%.3f\n", 1000.0/sim_context.rate);
        break;
      }
    } //rates

  } //openmp

  
  save_results("pinterf_0.0.txt", collisions, sim_context.rates, sim_context.len_rates, sim_context.iterations, sim_context.error, sim_context.l, sim_context.speed, sim_context.prob);
  return 0;
}
#endif
