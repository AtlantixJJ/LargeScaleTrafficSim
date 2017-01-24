#include "segment.h"
#include "vehicle.h"
#include "cargen.h"
#include "globalsim.h"
#include "randgen.h"
#include <stdio.h>
#include <string.h>

GlobalSimulator GS;

/// T : total simulation seconds
/// vb : verbosity. choosing one is recommended
/// lane_num : lane number
void init(int T,int percent,int entry){
    srand(time(NULL));
    GS.initStim(T,percent,entry);
    GS.Simulate();
}

int main(int argc,char **argv){
    //testFunc();
    init(atoi(argv[1]),atoi(argv[2]),atoi(argv[3]));
    return 0;
}
