#ifndef CONTROLLEDRONE_H
#define CONTROLLEDRONE_H

//parrent class
//#include <sar_drone/ParrentDroneClass.h>
#include <sar_drone/Position.h>

//class specific includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>

#include <thread>
#include <math.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Joy.h>

#include <sar_drone/directions.h>
#include <sar_drone/status.h>
#include <sar_drone/send_mobile.h>

namespace SaR_Drone{

    class ControlleDrone: public Position
    {
    public:
        ControlleDrone();
        ~ControlleDrone();

        void directionsCallback(const sar_drone::directions::ConstPtr& msg);
        void directionsPRIOCallback(const sar_drone::directions::ConstPtr& msg);

        std::thread* getPRIO_thread(){return &PRIO_thread_spinner;}

        ros::CallbackQueue PRIO_Calback_queue;

        void step(double sleepTime);

    private:

        statusCodes Status;
        statusCodes OldStatus;

        void updateStatus(statusCodes newStatus){
            sar_drone::status pub_msg;
            pub_msg.Status = newStatus;
            drone_status_pub.publish(pub_msg);

            OldStatus = Status;
            Status = newStatus;
        }

        enum Moving{
            IDLE, 
            MOVING,
            ROTATING,
            BREAKING,
        };

        bool StartMoveRelativeGround(float x, float y, float z, bool headless);
        bool StartMoveRelativeBody(float x, float y, float z);
        //bool rotate
        float getDirectionAngle(float x, float y);

        bool flying;
        bool landing;

        ServiceAck obtainCtrlAuthority();
        bool takeoff();
        bool land();

        bool takeoff_land(int task);


        ros::ServiceClient drone_task_service;
        ros::ServiceClient sdk_ctrl_authority_service;

        ros::Subscriber drone_commands_sub;
        ros::Subscriber drone_PRIO_commands_sub;

        ros::Publisher drone_ctrl_pos_yaw_pub;
        ros::Publisher drone_status_pub;

        ros::NodeHandle nh_PRIO;

        std::thread PRIO_thread_spinner;

        //moving var
        Moving state;
        uint8_t counter;

        uint8_t fail_counter;
        geometry_msgs::Point fail_pos;

        float xTarget;
        float yTarget;
        float zTarget;
        float rTarget;

        float lastx, lasty, lastz;

        uint8_t flag;

        const uint8_t breakflag = (DJISDK::VERTICAL_VELOCITY   |
                                   DJISDK::HORIZONTAL_VELOCITY |
                                   DJISDK::YAW_RATE            |
                                   DJISDK::HORIZONTAL_GROUND   |
                                   DJISDK::STABLE_ENABLE);

        ros::Time start_time;
        ros::Time start_move_time;
        ros::Duration elapsed_time;
        ros::Duration elapsed_move_time;
    };

}

#endif