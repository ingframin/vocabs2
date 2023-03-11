#include "vec2.h"
#include "test_vec2.h"
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

bool test_v2_mod(){
    printf("test: v2_mod()\n");
    vec2 vs[] = {
        {0.0,0.0},
        {-1.00E-09,	-1.00E-09},
        {-1.00E-09,	1.00E-09},
        {-1.00E-11,	1.00E-09},
        {-1.00E-11,	1.00E-11}
    };
    double expected[]={
        0.000000000E+00,
	    1.414213562E-09,
	    1.414213562E-09,
	    1.000049999E-09,
	    1.414213562E-11
    };
    double results[5];

    printf("result;\texpected;\t -> difference\n");
    for(uint32_t i = 0; i < 5;i++){
        results[i] = fabs(v2_mod(vs[i])-expected[i]);
        printf("%.12f;\t%.12f -> %.9f\n",v2_mod(vs[i]),expected[i],results[i]);
        if(results[i]>1E-9){
            return false;
        }
    }
    

    return true;
}

//rotate sign * PI/2
bool test_v2_rotateHalfPI(){
    vec2 vtest1 = {1.0,0.0};
    vec2 vtest2 = {sqrt(2.0)/2,sqrt(2.0)/2};
    vec2 expected[] ={
        {0.0,1.0},  //+pi/2
        {0.0,-1.0}, //-pi/2
        {-1.0,0.0}, //2x +p/2
        {-1.0,0.0}, //2x -pi/2
        {-sqrt(2.0)/2,sqrt(2.0)/2}//pi/4 -> -pi/4
    };
    vec2 results[5];
    results[0] = v2_rotateHalfPI(vtest1,+1);
    results[1] = v2_rotateHalfPI(vtest1,-1);
    results[2] = v2_rotateHalfPI(v2_rotateHalfPI(vtest1,+1),+1);
    results[3] = v2_rotateHalfPI(v2_rotateHalfPI(vtest1,-1),-1);
    results[4] = v2_rotateHalfPI(vtest2,+1);

    printf("result;\texpected;\t -> result\n");
    for(uint32_t i = 0; i < 4;i++){
        printf("(%.9f,\t%.9f)->(%.9f,\t%.9f)-> (%.9f,\t%.9f)\n",vtest1.x,vtest1.y,expected[i].x,expected[i].y,results[i].x,results[i].y);
        vec2 dif = v2_diff(expected[i],results[i]);
        if(v2_mod(dif)>1E-9){
            return false;
        }
    }
    printf("(%.9f,\t%.9f)->(%.9f,\t%.9f)-> (%.9f,\t%.9f)\n",vtest2.x,vtest2.y,expected[4].x,expected[4].y,results[4].x,results[4].y);
    vec2 dif = v2_diff(expected[4],results[4]);
    if(v2_mod(dif)>1E-9){
        return false;
    }
    return true;


}