#include <sar_drone/Position.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;

Position::Position()
{
    attitudeSub = nh.subscribe<geometry_msgs::QuaternionStamped>("dji_sdk/attitude", 10, [this](const geometry_msgs::QuaternionStamped::ConstPtr& msg){
        current_atti = msg->quaternion;
    });
    gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10, [this](const sensor_msgs::NavSatFix::ConstPtr& msg){
        current_gps = *msg;
    });
    localPosition = nh.subscribe<geometry_msgs::PointStamped>("dji_sdk/local_position", 10, [this](const geometry_msgs::PointStamped::ConstPtr& msg){
        current_local_pos = msg->point;
    });

    flightStatus = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, [this] (const std_msgs::UInt8::ConstPtr& msg){
        flight_status = msg->data;
    });

    set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

    while(!set_local_position()){
        spin(1.0);
        ROS_INFO_STREAM("No GPS fix");
    }

    while(current_gps.latitude == 0 && current_gps.longitude == 0){
        spin(1.0);
    }

    ROS_INFO_STREAM("Got GPS fix");
}

Position::~Position()
{

}

bool Position::set_local_position(){
    dji_sdk::SetLocalPosRef tempValue;
    set_local_pos_reference.call(tempValue);
    return tempValue.response.result;
}