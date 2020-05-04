#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "vec2.h"
#include "drone.h"
//#include "video.h"
#include <omp.h>
omp_lock_t writelock;

time_t t;
uint64_t iterations = 10000;

double dt = 1E-3; //seconds

double error = -1.0;

double rates[] = {
    1E-2,
    2.5E-2,
    5E-2,
    1E-1,
    2.5E-1,
    5E-1,
    1.0,
    2.0,
    4.0,
    8.0,
    16.0}; //msg/s

uint64_t len_rates = sizeof(rates) / sizeof(double);

double rate = 1.0;
char prob = 'A';
int main(int argc, char *argv[])
{
  if (argc == 2)
  {
    prob = argv[1][0];
  }
  if (argc == 3)
  {
    error = atof(argv[2]);
  }
  printf("Error: %.3f", error);
  switch (prob)
  {
  case 'E':
    printf("Wi-Fi beacons\n");
    break;
  case 'C':
    printf("ADS-B\n");
    break;
  default:
    printf("No loss\n");
  }

  double collisions[len_rates];
  omp_init_lock(&writelock);
  t = time(NULL);
  srand(t);

  //speed is in m/s

  vec2 p1 = {500.0, 500.0};
  vec2 p2 = {1000.0, 1000.0};
  vec2 p3 = {000.0, 1000.0};

  printf("Rate:\tPcrash:\n");
#pragma omp parallel
  {

    for (uint64_t i = 0; i < len_rates; i++)
    {

      collisions[i] = 0;
#pragma omp for
      for (uint64_t it = 0; it < iterations; it++)
      {

        Drone d1 = DR_newDrone(0.0, 0.0, 20.0, 0.0, 1);
        Drone d2 = DR_newDrone(1000.0, 0.0, 20.0, 0.0, 1);
        DR_push_waypoint(&d1, p2);
        DR_push_waypoint(&d2, p3);
        DR_push_waypoint(&d1, p1);
        DR_push_waypoint(&d2, p1);

        rate = rates[i];

        bool running = true;
        double timer = 0;

        while (running)
        {

          if (timer >= 1 / rate)
          {
            // Change to wi-fi or ads-b functions
            int p = rand() % 1000;
            double lim;
            switch (prob)
            {
            case 'E':

              lim = 1000 * esat(v2_distance(d1.position, d2.position));
              break;
            case 'C':
              lim = 1000 * consiglio(v2_distance(d1.position, d2.position));
              break;
            default:
              lim = 1000.0;
            }

            if (p < lim)
            {

              DR_avoid(&d2, &d1, error);
              DR_avoid(&d1, &d2, error);
            }
            timer = 0;
          }

          DR_move(&d1, dt);
          DR_move(&d2, dt);
          if (d1.waypoints[d1.curr_wp].x == 0 && d1.waypoints[d1.curr_wp].y == 0)
          {
            running = false;
          }
          if (d2.waypoints[d2.curr_wp].x == 0 && d2.waypoints[d2.curr_wp].y == 0)
          {
            running = false;
          }
          if (v2_distance(d1.position, d2.position) < (d1.size + d2.size))
          {
            omp_set_lock(&writelock);
            collisions[i] += 1;
            omp_unset_lock(&writelock);
            running = false;
          }

          timer += dt;
        }
      } //iterations

    } //rates
  }   //openmp

  FILE *results = fopen("results.txt", "a");
  fprintf(results, "Error: %.3f", error);
  switch (prob)
  {
  case 'E':
    fprintf(results, "Wi-Fi beacons\n");
    break;
  case 'C':
    fprintf(results, "ADS-B\n");
    break;
  default:
    fprintf(results, "No loss\n");
  }

  for (uint64_t k = 0; k < len_rates; k++)
  {
    printf("%.3f\t%.6f\n", rates[k], collisions[k] / iterations);
    fprintf(results, "%.3f\t%.10f\n", rates[k], collisions[k] / iterations);
  }
  fclose(results);

  return 0;
}
