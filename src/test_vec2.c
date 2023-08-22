
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

#include "vec2.h"
#include "test_vec2.h"
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <stdio.h>

static vec2 rotations[] = {
{1.0,0.0},
{0.9998476951563913,0.01745240643728351},
{0.9993908270190958,0.03489949670250097},
{0.9986295347545738,0.05233595624294383},
{0.9975640502598242,0.0697564737441253},
{0.9961946980917455,0.08715574274765817},
{0.9945218953682733,0.10452846326765346},
{0.992546151641322,0.12186934340514748},
{0.9902680687415704,0.13917310096006544},
{0.9876883405951378,0.15643446504023087},
{0.984807753012208,0.17364817766693033},
{0.981627183447664,0.1908089953765448},
{0.9781476007338057,0.20791169081775931},
{0.9743700647852352,0.224951054343865},
{0.9702957262759965,0.24192189559966773},
{0.9659258262890683,0.25881904510252074},
{0.9612616959383189,0.27563735581699916},
{0.9563047559630354,0.29237170472273677},
{0.9510565162951535,0.3090169943749474},
{0.9455185755993168,0.32556815445715664},
{0.9396926207859084,0.3420201433256687},
{0.9335804264972017,0.35836794954530027},
{0.9271838545667874,0.374606593415912},
{0.9205048534524404,0.3907311284892737},
{0.9135454576426009,0.40673664307580015},
{0.9063077870366499,0.42261826174069944},
{0.898794046299167,0.4383711467890774},
{0.8910065241883679,0.45399049973954675},
{0.882947592858927,0.4694715627858908},
{0.8746197071393957,0.48480962024633706},
{0.8660254037844387,0.49999999999999994},
{0.8571673007021123,0.5150380749100542},
{0.848048096156426,0.5299192642332049},
{0.838670567945424,0.5446390350150271},
{0.8290375725550417,0.5591929034707469},
{0.8191520442889918,0.573576436351046},
{0.8090169943749475,0.5877852522924731},
{0.7986355100472928,0.6018150231520483},
{0.788010753606722,0.6156614753256582},
{0.7771459614569709,0.6293203910498374},
{0.766044443118978,0.6427876096865393},
{0.754709580222772,0.6560590289905072},
{0.7431448254773942,0.6691306063588582},
{0.7313537016191706,0.6819983600624985},
{0.7193398003386512,0.6946583704589973},
{0.7071067811865476,0.7071067811865476},
{0.6946583704589974,0.7193398003386511},
{0.6819983600624985,0.7313537016191705},
{0.6691306063588582,0.7431448254773941},
{0.6560590289905073,0.754709580222772},
{0.6427876096865394,0.766044443118978},
{0.6293203910498375,0.7771459614569708},
{0.6156614753256583,0.788010753606722},
{0.6018150231520484,0.7986355100472928},
{0.5877852522924731,0.8090169943749475},
{0.573576436351046,0.8191520442889918},
{0.5591929034707468,0.8290375725550417},
{0.5446390350150272,0.8386705679454239},
{0.5299192642332049,0.8480480961564261},
{0.5150380749100543,0.8571673007021122},
{0.5000000000000001,0.8660254037844386},
{0.4848096202463371,0.8746197071393957},
{0.46947156278589086,0.8829475928589269},
{0.4539904997395468,0.8910065241883678},
{0.43837114678907746,0.898794046299167},
{0.42261826174069944,0.9063077870366499},
{0.4067366430758002,0.9135454576426009},
{0.39073112848927394,0.9205048534524403},
{0.37460659341591196,0.9271838545667874},
{0.3583679495453004,0.9335804264972017},
{0.3420201433256688,0.9396926207859083},
{0.32556815445715676,0.9455185755993167},
{0.30901699437494745,0.9510565162951535},
{0.29237170472273677,0.9563047559630354},
{0.27563735581699916,0.9612616959383189},
{0.25881904510252074,0.9659258262890683},
{0.2419218955996679,0.9702957262759965},
{0.22495105434386492,0.9743700647852352},
{0.20791169081775945,0.9781476007338056},
{0.19080899537654492,0.981627183447664},
{0.17364817766693041,0.984807753012208},
{0.15643446504023092,0.9876883405951378},
{0.1391731009600657,0.9902680687415703},
{0.12186934340514749,0.992546151641322},
{0.10452846326765346,0.9945218953682733},
{0.08715574274765814,0.9961946980917455},
{0.06975647374412546,0.9975640502598242},
{0.052335956242943966,0.9986295347545738},
{0.03489949670250108,0.9993908270190958},
{0.017452406437283376,0.9998476951563913},
{6.123233995736766e-17,1.0},
{-0.017452406437283477,0.9998476951563913},
{-0.03489949670250073,0.9993908270190958},
{-0.05233595624294362,0.9986295347545738},
{-0.06975647374412533,0.9975640502598242},
{-0.08715574274765824,0.9961946980917455},
{-0.10452846326765333,0.9945218953682734},
{-0.12186934340514737,0.9925461516413221},
{-0.13917310096006535,0.9902680687415704},
{-0.15643446504023104,0.9876883405951377},
{-0.1736481776669303,0.984807753012208},
{-0.1908089953765448,0.981627183447664},
{-0.20791169081775912,0.9781476007338057},
{-0.2249510543438648,0.9743700647852352},
{-0.24192189559966779,0.9702957262759965},
{-0.25881904510252085,0.9659258262890683},
{-0.27563735581699905,0.9612616959383189},
{-0.29237170472273666,0.9563047559630355},
{-0.30901699437494734,0.9510565162951536},
{-0.3255681544571564,0.9455185755993168},
{-0.3420201433256687,0.9396926207859084},
{-0.35836794954530027,0.9335804264972017},
{-0.37460659341591207,0.9271838545667874},
{-0.3907311284892736,0.9205048534524404},
{-0.4067366430758001,0.913545457642601},
{-0.42261826174069933,0.90630778703665},
{-0.4383711467890775,0.8987940462991669},
{-0.45399049973954675,0.8910065241883679},
{-0.46947156278589053,0.8829475928589271},
{-0.484809620246337,0.8746197071393959},
{-0.4999999999999998,0.8660254037844387},
{-0.5150380749100543,0.8571673007021123},
{-0.5299192642332048,0.8480480961564261},
{-0.5446390350150271,0.838670567945424},
{-0.5591929034707467,0.8290375725550417},
{-0.5735764363510458,0.819152044288992},
{-0.587785252292473,0.8090169943749475},
{-0.6018150231520484,0.7986355100472927},
{-0.6156614753256583,0.788010753606722},
{-0.6293203910498373,0.777145961456971},
{-0.6427876096865394,0.766044443118978},
{-0.6560590289905075,0.7547095802227718},
{-0.6691306063588582,0.7431448254773942},
{-0.6819983600624984,0.7313537016191706},
{-0.694658370458997,0.7193398003386514},
{-0.7071067811865475,0.7071067811865476},
{-0.7193398003386513,0.6946583704589971},
{-0.7313537016191705,0.6819983600624985},
{-0.7431448254773941,0.6691306063588583},
{-0.754709580222772,0.6560590289905073},
{-0.7660444431189779,0.6427876096865395},
{-0.7771459614569707,0.6293203910498377},
{-0.7880107536067219,0.6156614753256584},
{-0.7986355100472929,0.6018150231520482},
{-0.8090169943749473,0.5877852522924732},
{-0.8191520442889916,0.5735764363510464},
{-0.8290375725550416,0.5591929034707469},
{-0.8386705679454242,0.5446390350150269},
{-0.848048096156426,0.5299192642332049},
{-0.8571673007021122,0.5150380749100544},
{-0.8660254037844387,0.49999999999999994},
{-0.8746197071393957,0.48480962024633717},
{-0.8829475928589268,0.4694715627858911},
{-0.8910065241883678,0.45399049973954686},
{-0.898794046299167,0.4383711467890773},
{-0.9063077870366499,0.4226182617406995},
{-0.9135454576426008,0.40673664307580043},
{-0.9205048534524402,0.39073112848927416},
{-0.9271838545667873,0.37460659341591224},
{-0.9335804264972017,0.3583679495453002},
{-0.9396926207859083,0.3420201433256689},
{-0.9455185755993167,0.32556815445715703},
{-0.9510565162951535,0.3090169943749475},
{-0.9563047559630354,0.29237170472273705},
{-0.9612616959383187,0.27563735581699966},
{-0.9659258262890682,0.258819045102521},
{-0.9702957262759965,0.24192189559966773},
{-0.9743700647852352,0.22495105434386478},
{-0.9781476007338057,0.20791169081775931},
{-0.981627183447664,0.19080899537654497},
{-0.984807753012208,0.17364817766693028},
{-0.9876883405951377,0.15643446504023098},
{-0.9902680687415703,0.13917310096006574},
{-0.992546151641322,0.12186934340514755},
{-0.9945218953682733,0.10452846326765373},
{-0.9961946980917455,0.08715574274765864},
{-0.9975640502598242,0.06975647374412552},
{-0.9986295347545738,0.05233595624294381},
{-0.9993908270190958,0.0348994967025007},
{-0.9998476951563913,0.01745240643728344},
{-1.0,1.2246467991473532e-16},
{-0.9998476951563913,-0.017452406437283192},
{-0.9993908270190958,-0.0348994967025009},
{-0.9986295347545738,-0.052335956242943564},
{-0.9975640502598243,-0.06975647374412483},
{-0.9961946980917455,-0.08715574274765794},
{-0.9945218953682734,-0.10452846326765305},
{-0.992546151641322,-0.12186934340514774},
{-0.9902680687415703,-0.13917310096006552},
{-0.9876883405951378,-0.15643446504023073},
{-0.984807753012208,-0.17364817766693047},
{-0.981627183447664,-0.19080899537654472},
{-0.9781476007338057,-0.20791169081775907},
{-0.9743700647852352,-0.22495105434386498},
{-0.9702957262759965,-0.2419218955996675},
{-0.9659258262890684,-0.25881904510252035},
{-0.9612616959383189,-0.275637355816999},
{-0.9563047559630355,-0.2923717047227364},
{-0.9510565162951535,-0.30901699437494773},
{-0.9455185755993167,-0.32556815445715676},
{-0.9396926207859084,-0.34202014332566866},
{-0.9335804264972017,-0.35836794954530043},
{-0.9271838545667874,-0.374606593415912},
{-0.9205048534524404,-0.39073112848927355},
{-0.9135454576426011,-0.4067366430757998},
{-0.90630778703665,-0.4226182617406993},
{-0.8987940462991671,-0.43837114678907707},
{-0.8910065241883681,-0.45399049973954625},
{-0.8829475928589269,-0.46947156278589086},
{-0.8746197071393959,-0.48480962024633695},
{-0.8660254037844386,-0.5000000000000001},
{-0.8571673007021123,-0.5150380749100542},
{-0.8480480961564261,-0.5299192642332048},
{-0.838670567945424,-0.5446390350150271},
{-0.8290375725550417,-0.5591929034707467},
{-0.819152044288992,-0.5735764363510458},
{-0.8090169943749475,-0.587785252292473},
{-0.798635510047293,-0.601815023152048},
{-0.7880107536067222,-0.6156614753256578},
{-0.7771459614569708,-0.6293203910498376},
{-0.766044443118978,-0.6427876096865393},
{-0.7547095802227719,-0.6560590289905074},
{-0.7431448254773942,-0.6691306063588582},
{-0.7313537016191706,-0.6819983600624984},
{-0.7193398003386511,-0.6946583704589973},
{-0.7071067811865477,-0.7071067811865475},
{-0.6946583704589975,-0.7193398003386509},
{-0.6819983600624989,-0.7313537016191701},
{-0.6691306063588585,-0.743144825477394},
{-0.6560590289905077,-0.7547095802227717},
{-0.6427876096865395,-0.7660444431189779},
{-0.6293203910498371,-0.7771459614569711},
{-0.6156614753256581,-0.7880107536067221},
{-0.6018150231520483,-0.7986355100472928},
{-0.5877852522924732,-0.8090169943749473},
{-0.5735764363510464,-0.8191520442889916},
{-0.5591929034707472,-0.8290375725550414},
{-0.544639035015027,-0.8386705679454242},
{-0.529919264233205,-0.848048096156426},
{-0.5150380749100544,-0.8571673007021121},
{-0.5000000000000004,-0.8660254037844385},
{-0.48480962024633684,-0.8746197071393959},
{-0.46947156278589075,-0.882947592858927},
{-0.4539904997395469,-0.8910065241883678},
{-0.43837114678907774,-0.8987940462991668},
{-0.42261826174069994,-0.9063077870366497},
{-0.4067366430758001,-0.913545457642601},
{-0.3907311284892738,-0.9205048534524403},
{-0.3746065934159123,-0.9271838545667873},
{-0.3583679495453007,-0.9335804264972016},
{-0.3420201433256694,-0.9396926207859082},
{-0.32556815445715664,-0.9455185755993168},
{-0.30901699437494756,-0.9510565162951535},
{-0.2923717047227371,-0.9563047559630353},
{-0.2756373558169989,-0.961261695938319},
{-0.25881904510252063,-0.9659258262890683},
{-0.24192189559966779,-0.9702957262759965},
{-0.22495105434386525,-0.9743700647852351},
{-0.2079116908177598,-0.9781476007338056},
{-0.19080899537654547,-0.9816271834476639},
{-0.17364817766693033,-0.984807753012208},
{-0.15643446504023104,-0.9876883405951377},
{-0.13917310096006494,-0.9902680687415704},
{-0.12186934340514717,-0.9925461516413221},
{-0.10452846326765336,-0.9945218953682734},
{-0.08715574274765825,-0.9961946980917455},
{-0.06975647374412558,-0.9975640502598242},
{-0.052335956242944306,-0.9986295347545738},
{-0.03489949670250165,-0.9993908270190957},
{-0.017452406437283498,-0.9998476951563913},
{-1.8369701987210297e-16,-1.0},
{0.01745240643728313,-0.9998476951563913},
{0.03489949670250128,-0.9993908270190958},
{0.052335956242943946,-0.9986295347545738},
{0.06975647374412522,-0.9975640502598243},
{0.08715574274765789,-0.9961946980917455},
{0.10452846326765299,-0.9945218953682734},
{0.12186934340514768,-0.992546151641322},
{0.13917310096006547,-0.9902680687415704},
{0.15643446504023067,-0.9876883405951378},
{0.17364817766692997,-0.9848077530122081},
{0.19080899537654425,-0.9816271834476641},
{0.20791169081775857,-0.9781476007338058},
{0.22495105434386492,-0.9743700647852352},
{0.24192189559966742,-0.9702957262759966},
{0.25881904510252113,-0.9659258262890682},
{0.2756373558169994,-0.9612616959383188},
{0.2923717047227367,-0.9563047559630354},
{0.30901699437494723,-0.9510565162951536},
{0.3255681544571563,-0.945518575599317},
{0.34202014332566816,-0.9396926207859085},
{0.35836794954529955,-0.9335804264972021},
{0.37460659341591196,-0.9271838545667874},
{0.3907311284892735,-0.9205048534524405},
{0.40673664307580054,-0.9135454576426008},
{0.4226182617406996,-0.9063077870366498},
{0.4383711467890774,-0.898794046299167},
{0.45399049973954664,-0.891006524188368},
{0.4694715627858904,-0.8829475928589271},
{0.4848096202463365,-0.8746197071393961},
{0.5000000000000001,-0.8660254037844386},
{0.5150380749100542,-0.8571673007021123},
{0.5299192642332047,-0.8480480961564261},
{0.5446390350150266,-0.8386705679454243},
{0.5591929034707462,-0.8290375725550421},
{0.573576436351046,-0.8191520442889918},
{0.5877852522924729,-0.8090169943749476},
{0.6018150231520479,-0.798635510047293},
{0.6156614753256585,-0.7880107536067218},
{0.6293203910498375,-0.7771459614569708},
{0.6427876096865393,-0.7660444431189781},
{0.656059028990507,-0.7547095802227722},
{0.6691306063588578,-0.7431448254773946},
{0.6819983600624979,-0.731353701619171},
{0.6946583704589966,-0.7193398003386517},
{0.7071067811865474,-0.7071067811865477},
{0.7193398003386509,-0.6946583704589976},
{0.7313537016191707,-0.6819983600624983},
{0.7431448254773942,-0.6691306063588581},
{0.7547095802227719,-0.6560590289905074},
{0.7660444431189778,-0.6427876096865396},
{0.7771459614569706,-0.6293203910498378},
{0.7880107536067216,-0.6156614753256588},
{0.7986355100472928,-0.6018150231520483},
{0.8090169943749473,-0.5877852522924734},
{0.8191520442889916,-0.5735764363510465},
{0.8290375725550414,-0.5591929034707473},
{0.838670567945424,-0.544639035015027},
{0.8480480961564254,-0.5299192642332058},
{0.8571673007021121,-0.5150380749100545},
{0.8660254037844384,-0.5000000000000004},
{0.8746197071393959,-0.4848096202463369},
{0.882947592858927,-0.4694715627858908},
{0.8910065241883678,-0.45399049973954697},
{0.8987940462991671,-0.438371146789077},
{0.9063077870366497,-0.4226182617407},
{0.913545457642601,-0.40673664307580015},
{0.9205048534524399,-0.3907311284892747},
{0.9271838545667873,-0.37460659341591235},
{0.9335804264972015,-0.35836794954530077},
{0.9396926207859084,-0.3420201433256686},
{0.9455185755993165,-0.32556815445715753},
{0.9510565162951535,-0.3090169943749477},
{0.9563047559630357,-0.29237170472273627},
{0.9612616959383187,-0.2756373558169998},
{0.9659258262890683,-0.2588190451025207},
{0.9702957262759965,-0.24192189559966787},
{0.9743700647852351,-0.22495105434386534},
{0.9781476007338056,-0.20791169081775987},
{0.981627183447664,-0.19080899537654467},
{0.9848077530122079,-0.17364817766693127},
{0.9876883405951377,-0.1564344650402311},
{0.9902680687415703,-0.13917310096006588},
{0.992546151641322,-0.12186934340514811},
{0.9945218953682733,-0.10452846326765342},
{0.9961946980917455,-0.08715574274765832},
{0.9975640502598243,-0.06975647374412476},
{0.9986295347545738,-0.05233595624294437},
{0.9993908270190958,-0.034899496702500823},
{0.9998476951563913,-0.01745240643728445}
};

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

//Needs rewriting
// //rotate sign * PI/2
// bool test_v2_rotateHalfPI(){
//     vec2 vtest1 = {1.0,0.0};
//     vec2 vtest2 = {sqrt(2.0)/2,sqrt(2.0)/2};
//     vec2 expected[] ={
//         {0.0,1.0},  //+pi/2
//         {0.0,-1.0}, //-pi/2
//         {-1.0,0.0}, //2x +p/2
//         {-1.0,0.0}, //2x -pi/2
//         {-sqrt(2.0)/2,sqrt(2.0)/2}//pi/4 -> -pi/4
//     };
//     vec2 results[5];
//     results[0] = v2_rotateLeftHalfPI(vtest1);
//     results[1] = v2_reverse(v2_rotateLeftHalfPI(vtest1));
//     results[2] = v2_rotateLeftHalfPI(v2_rotateLeftHalfPI(vtest1));
//     results[3] = v2_rotateLeftHalfPI(v2_rotateLeftHalfPI(vtest1));
//     results[4] = v2_rotateLeftHalfPI(vtest2,+1);

//     printf("result;\texpected;\t -> result\n");
//     for(uint32_t i = 0; i < 4;i++){
//         printf("(%.9f,\t%.9f)->(%.9f,\t%.9f)-> (%.9f,\t%.9f)\n",vtest1.x,vtest1.y,expected[i].x,expected[i].y,results[i].x,results[i].y);
//         vec2 dif = v2_diff(expected[i],results[i]);
//         if(v2_mod(dif)>1E-9){
//             return false;
//         }
//     }
//     printf("(%.9f,\t%.9f)->(%.9f,\t%.9f)-> (%.9f,\t%.9f)\n",vtest2.x,vtest2.y,expected[4].x,expected[4].y,results[4].x,results[4].y);
//     vec2 dif = v2_diff(expected[4],results[4]);
//     if(v2_mod(dif)>1E-9){
//         return false;
//     }
//     return true;


// }

//rotate
bool test_v2_rotate(){
    vec2 vtest = {1.0,0.0};
    double angles[360];
    for(uint16_t i = 0; i < 360; i++){
        angles[i] = i*M_PI/180.0;
    }
    vec2 results[360];
    printf("result;\texpected;\t -> distance\n");
    for(int i =0; i< 360;i++){
        results[i] = v2_rotate(vtest,angles[i]);
        double dist = v2_distance(results[i],rotations[i]);
        printf("(%.9f,\t%.9f)->(%.9f,\t%.9f)-> %.9f\n",results[i].x,results[i].y,rotations[i].x,rotations[i].y,dist);
        if(dist>1E-6){
            printf("v2_rotate Failed!");
            return false;
        }
    }
    return true;
}