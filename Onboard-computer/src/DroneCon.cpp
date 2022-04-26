#include <sar_drone/ControlleDrone.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;
//main
int main (int argc, char ** argv)
{
    ros::init(argc, argv, "com_with_drone");

    ControlleDrone controlleDrone;

    while (ros::ok()){
        controlleDrone.step(0.01);
    }

    controlleDrone.getPRIO_thread()->join();
}