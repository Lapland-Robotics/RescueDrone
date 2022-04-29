#ifndef PARRENTDRONECLASS_H
#define PARRENTDRONECLASS_H

//system includes
#include <sar_drone/defines.h>

//ROS includes
#include <ros/ros.h>

//DJI SDK include
#include <dji_sdk/Activation.h>

//DJI SDK Core
#include <djiosdk/dji_vehicle.hpp>

namespace SaR_Drone{

    class ParrentDroneClass
    {
    public:
        ParrentDroneClass();
        ~ParrentDroneClass();

        struct ServiceAck
        {
            bool         result;
            int          cmd_set;
            int          cmd_id;
            unsigned int ack_data;
            ServiceAck(bool res, int set, int id, unsigned int ack): result(res), cmd_set(set), cmd_id(id), ack_data(ack)
            {}
                ServiceAck()
            {}
        }; 

        bool isReady(){return ready;}
        void spin(double sleepTime){ros::spinOnce(); ros::Duration(sleepTime).sleep();ros::spinOnce();}
    
    protected:
        ros::NodeHandle nh;
        bool ready;

    private:
        ServiceAck activateSDK();

        ros::ServiceClient drone_activation_service;
    };

}

#endif