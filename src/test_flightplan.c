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
    if(fp->current_wp != 0){
        printf("failed initialization: fp->current_wp != 0\n");
        goto test_failed;
    } 
	if(fp->length != 4){
        printf("failed initialization: fp->length != 4\n");
        goto test_failed;
    } 
    if(!fp->empty){
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
    
    if(fp->current_wp != 1){
        printf("failed increment current wp index: fp->current_wp != 1\n");
        goto test_failed;
    } 
	if(fp->length != 4){
        printf("failed initialization: fp->length != 4\n");
        goto test_failed;
    } 
    if(fp->empty){
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
    printf("%u\n", fp->length);
    for(int i =0;i<fp->length;i++){
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
// bool test_pop_waypoint();
// //get the current waypoint
// bool test_current_wp();
// //Free memory used by a FlightPlan
// bool test_free_FlightPlan();