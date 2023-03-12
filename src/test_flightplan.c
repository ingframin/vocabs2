#include "flightplan.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

//Initialize a new fligth plan
bool test_newFlightPlan(){
    FlightPlan* fp = FP_newFlightPlan(4);
    
    if(fp==NULL) {
        printf("failed pointer initialization: fp == NULL\n");
        return false;
    }

    if(fp->waypoints == NULL){
        printf("failed pointer initialization: fp->waypoints == NULL\n");
        free(fp);
        return false;
    }
    if(fp->current_wp != 0){
        printf("failed initialization: fp->cur_wp != 0\n");
        free(fp->waypoints);
        free(fp);
        return false;
    } 
	if(fp->length != 4){
        printf("failed initialization: fp->length != 4\n");
        free(fp->waypoints);
        free(fp);
        return false;
    } 
    if(!fp->empty){
        printf("failed initialization: fp not empty\n");
        free(fp->waypoints);
        free(fp);
        return false;
    } 
    return true;

}
// //Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
// //Increase the waypoint array size if needed
// bool test_push_waypoint();
// //pop "removes" the top waypoint (but it does not shrink the waypoint array)
// //If the FlightPlan is empty, returns the 0-position waypoint
// bool test_pop_waypoint();
// //get the current waypoint
// bool test_current_wp();
// //Free memory used by a FlightPlan
// bool test_free_FlightPlan();