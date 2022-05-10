#include <sar_drone/SendReceveDataMSDK.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;
//main
int main (int argc, char ** argv)
{
    ros::init(argc, argv, "com_with_MSDK");

    SendReceveDataMSDK MsdkControll;
    
    while (ros::ok()){
        MsdkControll.spin(0.001);
    }

}