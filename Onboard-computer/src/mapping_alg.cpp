#include <sar_drone/mapAlg.h>

using namespace SaR_Drone;
//main
int main (int argc, char ** argv)
{
    ros::init(argc, argv, "mapping_algoritm");

    mapAlg MapAlg;
    
    while (ros::ok()){
        MapAlg.step(0.001);
    }

}