#ifndef POSSITION_H
#define POSSITION_H

//parrent class
#include <sar_drone/ParrentDroneClass.h>

//class specific includes
#include <sensor_msgs/NavSatFix.h>

#include <std_msgs/UInt8.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf/tf.h>

#include <dji_sdk/SetLocalPosRef.h>

#include <dji_sdk/dji_sdk.h>

namespace SaR_Drone{

    class Position: public ParrentDroneClass
    {
    public:
        Position();
        ~Position();

        sensor_msgs::NavSatFix getGPS(){return current_gps;}
        geometry_msgs::Vector3 getRotation(){
            geometry_msgs::Vector3 ans;

            tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(current_atti.x, current_atti.y, current_atti.z, current_atti.w));
            R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
            ans.z = angleabs(ans.z);
            return ans;}
        geometry_msgs::Point getPos(){return current_local_pos;}
        uint8_t getFlightStatus(){return flight_status;}
        
        double angleabs(double angle){
            if(std::abs(angle) > M_PI){
                angle += angle > 0? (-2 * M_PI) : (2 * M_PI);
            }
            if(std::abs(angle + M_PI) < YAW_THRESHOLD){
                angle += (2 * M_PI);
            }
            return angle;
        }

    private:
        bool set_local_position();

        ros::ServiceClient set_local_pos_reference;

        ros::Subscriber attitudeSub;
        ros::Subscriber gpsSub;
        ros::Subscriber localPosition;
        ros::Subscriber flightStatus;

        uint8_t flight_status = 255;
        sensor_msgs::NavSatFix current_gps;
        geometry_msgs::Quaternion current_atti;
        geometry_msgs::Point current_local_pos;
    };
}

#endif