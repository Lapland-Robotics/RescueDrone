#ifndef MAPALG_H
#define MAPALG_H

//system includes
#include <sar_drone/defines.h>
#include <sar_drone/directions.h>
#include <sar_drone/status.h>
#include <sar_drone/send_mobile.h>

//ROS includes
#include <ros/ros.h>

//Cpp includes
#include <vector>

namespace SaR_Drone{
    class mapAlg
    {
    public:
        mapAlg();
        ~mapAlg();

        void step(double sleepTime);


    private:
        void mapCommandsCalback(const sar_drone::directions::ConstPtr& msg);
        void CreateRoute();

        enum localStatus{
            OFF,
            IDLE,
            GOING_TO_START,
            START_HUMAN,
            START_HUMAN_SEND,
            MOVE_COMMAND_SEND,
            NEXT_MOVE,
            WAIT_MOVING,
            WAIT_STARTING,
            WAIT_MANUAL_OVERRIDE,
            WAIT_HUMAN_DETECTION,
            STOPPING,
            STOP_NOW,
        };

        localStatus local_status;
        localStatus next_local_status;

        statusCodes status_drone;
        
        const uint8_t msg_ID;
        uint8_t route_index;

        bool demo;
        bool got_route;
        bool on_site;
        
        ros::Time start_time;
        ros::Duration elapsed_time;

        std::vector<std::pair<double, double>> area;
        std::vector<std::pair<float, float>> route;
        std::pair<double, double> start_location; //first = lat, second = long
        std::pair<double, double> home_base;

        ros::NodeHandle nh;

        ros::Subscriber map_commands_sub;
        ros::Subscriber drone_status_sub;

        ros::Publisher drone_commands_pub;
        ros::Publisher drone_PRIO_commands_pub;
        ros::Publisher send_mobile_data_pub;
    };
}
#endif