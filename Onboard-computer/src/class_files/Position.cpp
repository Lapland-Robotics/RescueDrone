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


    if(ready){
        while(!set_local_position()){
            spin(1.0);
            ROS_INFO_STREAM("No GPS fix");
        }

        while(current_gps.latitude == 0 && current_gps.longitude == 0){
            spin(1.0);
        }
        
        ROS_INFO_STREAM("Got GPS fix");
    }
    else{
        ROS_INFO_STREAM("No GPS so go in test mode");
    }

}

Position::~Position()
{

}

bool Position::set_local_position(){
    dji_sdk::SetLocalPosRef tempValue;
    set_local_pos_reference.call(tempValue);
    return tempValue.response.result;
}

geometry_msgs::Point Position::translateGPS(sensor_msgs::NavSatFix origin, sensor_msgs::NavSatFix offset, bool debug_print, bool area_corners){
    if(debug_print){ROS_INFO_STREAM("\nCORX: " << origin.longitude << "\tCORY: " << origin.latitude << "\nCOFX: " << offset.longitude << "\tCOFY: " << offset.latitude);}
    origin.longitude = origin.longitude * M_PI / 180;
    origin.latitude = origin.latitude * M_PI / 180;
    offset.longitude = offset.longitude * M_PI / 180;
    offset.latitude = offset.latitude * M_PI / 180;

    long double distance = std::abs(RADIUS_EARTH * acos((sin(origin.latitude) * sin(offset.latitude)) + (cos(origin.latitude) * cos(offset.latitude) * cos(origin.longitude - offset.longitude))));

    long double phi = cos(origin.latitude) * sin(offset.latitude) - sin(origin.latitude) * cos(offset.latitude) * cos(offset.longitude - origin.longitude);
    long double lon = sin(offset.longitude - origin.longitude) * cos(offset.latitude);
    long double heading = atan2(lon, phi);
    
    geometry_msgs::Point tmp;
    tmp.x = cos(heading) * distance;
    tmp.y = sin(heading) * distance;
    tmp.z = area_corners ? distance : offset.altitude - home_altitude_pos;

    if(debug_print){ROS_INFO_STREAM("dis: " << distance << "\thead: " << heading << "\nx: " << tmp.x << "\ty: " << tmp.y);}

    return tmp;
}