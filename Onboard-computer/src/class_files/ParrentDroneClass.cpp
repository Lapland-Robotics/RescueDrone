#include <sar_drone/ParrentDroneClass.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;

ParrentDroneClass::ParrentDroneClass(){
    ready = false;
    drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");

    while(!ready){
        if (activateSDK().result){
            ROS_WARN("OSDK activated successfull");
            ready = true;
        }
        else{
            ROS_WARN("Faild to activate OSDK");
        }
        spin(0.5);
    }

}

ParrentDroneClass::~ParrentDroneClass(){

}

ParrentDroneClass::ServiceAck ParrentDroneClass::activateSDK(){
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result){
        ROS_INFO("ack.info: set = %i id = %i", activation.response.cmd_set,activation.response.cmd_id);
        ROS_INFO("ack.data: %i", activation.response.ack_data);
    }
    return ServiceAck(activation.response.result, activation.response.cmd_set,activation.response.cmd_id, activation.response.ack_data);
}