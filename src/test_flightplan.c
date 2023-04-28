
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

#include "flightplan.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "vec2.h"
//Initialize a new fligth plan
bool test_newFlightPlan(){
    FlightPlan* fp = FP_newFlightPlan(4);
    
    if(fp==NULL) {
        printf("failed pointer initialization: fp == NULL\n");
        return false;
    }

    if(fp->waypoints == NULL){
        printf("failed pointer initialization: fp->waypoints == NULL\n");
        goto test_failed;
    }
    if(fp->current_wp != -1){
        printf("failed initialization: fp->current_wp != -1\n");
        goto test_failed;
    } 
	if(fp->length != 4){
        printf("failed initialization: fp->length != 4\n");
        goto test_failed;
    } 
    if(FP_isFlightPlanEmpty(fp)){
        printf("failed initialization: fp not empty\n");
        goto test_failed;
    } 
    return true;
test_failed:
        free(fp->waypoints);
        free(fp);
        return false;
}
//Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
//Increase the waypoint array size if needed
bool test_push_waypoint(){
    FlightPlan* fp = FP_newFlightPlan(4);
    vec2 test_wp = {1.0,2.0};
    
    if(fp==NULL) {
        printf("failed pointer initialization: fp == NULL\n");
        goto test_failed;
    }

    if(fp->waypoints == NULL){
        printf("failed pointer initialization: fp->waypoints == NULL\n");
        goto test_failed;
    }

    FP_push_waypoint(fp,test_wp);
    
    if(fp->current_wp != 0){
        printf("failed increment current wp index: fp->current_wp != 0\n");
        goto test_failed;
    } 
	if(fp->length != 4){
        printf("failed initialization: fp->length != 4\n");
        goto test_failed;
    } 
    if(fp->current_wp<0){
        printf("failed initialization: fp is empty\n");
        goto test_failed;
    } 
    if(v2_distance(fp->waypoints[0],test_wp)>1E-6){
        printf("wrong waypoint inserted\n");
        goto test_failed;
    }
    for(int i = 0; i<15;i++){
        FP_push_waypoint(fp,v2_addK(test_wp,i));
    }
    printf("%llu\n", fp->length);
    for(int64_t i =0;i<fp->length;i++){
         printf("(%.9f,\t%.9f)\n",fp->waypoints[i].x,fp->waypoints[i].y);
    }
    return true;
test_failed:
        free(fp->waypoints);
        free(fp);
        return false;

}
// //pop "removes" the top waypoint (but it does not shrink the waypoint array)
// //If the FlightPlan is empty, returns the 0-position waypoint
bool test_pop_waypoint(){
    FlightPlan* fp = FP_newFlightPlan(4);
    vec2 test_wp = {1.0,2.0};
    for(int i = 0; i<15;i++){
        FP_push_waypoint(fp,v2_addK(test_wp,i));
    }
    
    for(int i = 15; i>-1;i--){
        FP_pop_waypoint(fp);
        printf("Current waypoint index: %.lld\n",fp->current_wp);
        vec2 cwp = FP_current_wp(fp);
        printf("Current Waypoint: %.6f;%.6f\n",cwp.x,cwp.y);
        printf("FlightPlan empty?: %s;\n",(FP_isFlightPlanEmpty(fp))?"true":"false");
    }
    return true;

}
// //get the current waypoint
// bool test_current_wp();
// //Free memory used by a FlightPlan
// bool test_free_FlightPlan();