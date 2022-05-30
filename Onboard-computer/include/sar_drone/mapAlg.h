#ifndef MAPALG_H
#define MAPALG_H

//system includes
#include <sar_drone/defines.h>
#include <sar_drone/directions.h>
#include <sar_drone/status.h>
#include <sar_drone/send_mobile.h>
#include <sar_drone/Position.h>
#include <sar_drone/routeplanner.h>

//ROS includes
#include <ros/ros.h>

//Cpp includes
#include <vector>
#include <math.h>

namespace SaR_Drone{
    class mapAlg: public Position
    {
    public:
        mapAlg();
        ~mapAlg();

        void step(double sleepTime);

        enum corners{
            I_AM_A_CICLE = 0,
            BOTTOM_LEFT = 1,
            BOTTOM_RIGHT = 2,
            TOP_LEFT = 3,
            TOP_RIGHT = 4,
        };
        
        struct AreaStruct{
            geometry_msgs::Point Point;
            corners corner = I_AM_A_CICLE;
            bool smallest = false;
        };

    private:
        void mapCommandsCalback(const sar_drone::directions::ConstPtr& msg);
        void CreateRoute(const std::vector<sar_drone::coordinates> &area);
        geometry_msgs::Point rotatePoint(double x, double y, double angle);

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

        std::vector<std::pair<float, float>> route;
        
        sensor_msgs::NavSatFix start_location; //first = lat, second = long
        sensor_msgs::NavSatFix home_base;
        sensor_msgs::NavSatFix fake_origin;

        ros::Subscriber map_commands_sub;
        ros::Subscriber drone_status_sub;

        ros::Publisher drone_commands_pub;
        ros::Publisher drone_PRIO_commands_pub;
        ros::Publisher send_mobile_data_pub;
    };

    std::ostream& operator<<(std::ostream& os, const mapAlg::corners& data)
    {
        switch(data){
            case mapAlg::corners::I_AM_A_CICLE:
                return os << "i'm not a corner";
            case mapAlg::corners::BOTTOM_LEFT :
                return os << "bottom left corner";
            case mapAlg::corners::BOTTOM_RIGHT:
                return os << "bottom right corner";
            case mapAlg::corners::TOP_LEFT:
                return os << "top left corner";
            case mapAlg::corners::TOP_RIGHT:
                return os << "top right corner";
            default:
                return os << "yeah i'm not even supose to exist";
        }
    }

    std::ostream& operator<<(std::ostream& os, const mapAlg::AreaStruct& data)
    {
        return os << std::setprecision(4) << (data.smallest ? "\e[0;32m" : "\e[0m") << data.corner << ": {\"x\" = " << data.Point.x << ", \"y\" = " << data.Point.y << ", \"distance to origin\" = " << data.Point.z << "}";
    }

    std::ostream& operator<<(std::ostream& os, const std::vector<mapAlg::AreaStruct> &data)
    {
        os << "There are " << data.size() << " corners\n";
        for(auto &i : data){
            os << i << "\n";
        }
        return os;
    }
}
#endif