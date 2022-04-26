#include <sar_drone/ControlleDrone.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;

ControlleDrone::ControlleDrone(){
    if(ready){
        flying = false;
        landing = false;
        ros::CallbackQueue *PRIO_Calback_queue_ptr = &PRIO_Calback_queue;
        nh_PRIO.setCallbackQueue(PRIO_Calback_queue_ptr);

        drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
        sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
        drone_commands_sub = nh.subscribe(DIRECTIONS_TOPPIC, 50, &ControlleDrone::directionsCallback, this);
        drone_PRIO_commands_sub = nh_PRIO.subscribe(DIRECTIONS_PRIO_TOPPIC, 50, &ControlleDrone::directionsPRIOCallback, this);
        drone_status_pub = nh.advertise<sar_drone::status>(STATUS_TOPPIC, 10);
        drone_ctrl_pos_yaw_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);


        updateStatus(STARTING_UP);

        PRIO_thread_spinner = std::thread([this, PRIO_Calback_queue_ptr](){
            ros::SingleThreadedSpinner PRIO_spinner;
            PRIO_spinner.spin(PRIO_Calback_queue_ptr);
        });

        ServiceAck ack = obtainCtrlAuthority();
        if (ack.result){
            ROS_INFO("Obtain SDK control Authority successfully");
        }
        else{
            if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0){
                ROS_INFO("Obtain SDK control Authority in progess, send the cmd again");
                obtainCtrlAuthority();
            }
            else{
                ROS_WARN("Failed Obtain SDK control Authority");
                ready = false;
            }
        }

        // while (ros::ok()){
            spin(1.0);
            ROS_INFO_STREAM("\nGPS:\n\tlatitude: " <<(double) getGPS().latitude << "\n\tlongitude: " << getGPS().longitude << "\n\taltitude: " << getGPS().altitude << "\nRotation: " << getRotation().z << "\npoint \n\tx: " << getPos().x << "\n\ty: " << getPos().y << "\n\tz: " << getPos().z);
        //     spin(1.0);
        // }
        
    }
    else{ //debugging and testing

        ROS_WARN_STREAM("entering debug and testing mode. Not connected to drone");

        ros::CallbackQueue *PRIO_Calback_queue_ptr = &PRIO_Calback_queue;

        nh_PRIO.setCallbackQueue(PRIO_Calback_queue_ptr);

        drone_commands_sub = nh.subscribe(DIRECTIONS_TOPPIC, 50, &ControlleDrone::directionsCallback, this);
        drone_PRIO_commands_sub = nh_PRIO.subscribe(DIRECTIONS_PRIO_TOPPIC, 50, &ControlleDrone::directionsPRIOCallback, this);

        PRIO_thread_spinner = std::thread([this, PRIO_Calback_queue_ptr](){
            ros::SingleThreadedSpinner PRIO_spinner;
            PRIO_spinner.spin(PRIO_Calback_queue_ptr);
        });

        //ros::spin();
    }
    
    updateStatus(ON_GROUND);
}

ControlleDrone::~ControlleDrone(){
}

void ControlleDrone::directionsCallback(const sar_drone::directions::ConstPtr& msg){
    //ROS_INFO_STREAM("I got " << (uint)msg->Directions << " from " << (uint)msg->ID);
    if(flying && !landing)
    { 
        switch(Status){
            case MAPPING_ALGORITM_NEXT_STEP:{
                switch (static_cast<msgCommands>(msg->Command)){
                    case MA_MOVE_RELATIVE_GROUND_HEADLESS:{
                        ros::Time start_time = ros::Time::now();
                        StartMoveRelativeGround(msg->x, msg->y, msg->z, true);
                        break;
                    }

                    case MA_MOVE_RELATIVE_GROUND:{
                        ros::Time start_time = ros::Time::now();
                        StartMoveRelativeGround(msg->x, msg->y, msg->z, false);
                        break;
                    }
                    
                    default:{
                        ROS_WARN_STREAM("wrong command for directions Calback (" << (int) msg->Command << ")");
                        break;
                    }
                }
                break;
            }
            case HUMAN_DETECTION_NEXT_STEP:{
                ROS_INFO_STREAM("Human detection controll");
            }
        }
        ros::Duration(0.1).sleep();
    }
}

void ControlleDrone::directionsPRIOCallback(const sar_drone::directions::ConstPtr& msg){
    switch (static_cast<msgCommands>(msg->Command))    {
    case TAKE_OFF:{
        if(getFlightStatus() == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR){
            ROS_WARN_STREAM("drone alreay flying");
        }
        else{
            ROS_INFO_STREAM("takeoff");
            updateStatus(TAKING_OFF);
            flying = takeoff();
        }
        updateStatus(MAPPING_ALGORITM_NEXT_STEP);
        break;
    }

    case LAND:{
        ROS_INFO_STREAM("land");        
        updateStatus(LANDING);

        if(!flying && getFlightStatus() == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR){
            ROS_WARN_STREAM("drone is flying");
            flying = true;
        }

        if(!landing && flying){
            state = IDLE;
            landing = true;
            landing = land();
            if(!landing){
                flying = false;
                updateStatus(ON_GROUND);
            }
        }
        else{
            ROS_INFO_STREAM("already on the ground");
            updateStatus(ON_GROUND);
        }
        break;
    }
    
    case HD_FOUND_PERSON_I_THINK:{
        ROS_INFO_STREAM("found potential person");
        // whoIsInControl = HUMANDETECTION;
    }
    default:
        ROS_WARN_STREAM("wrong command for directions PRIO Calback");
        break;
    }
}

void ControlleDrone::step(double sleepTime){
    spin(sleepTime);
    elapsed_time = ros::Time::now() - start_time;
    elapsed_move_time = ros::Time::now() - start_move_time;
    if(elapsed_time > ros::Duration(0.02)){
        start_time = ros::Time::now();
        if(state == MOVING){
            geometry_msgs::Point pos = getPos();
            float xcmd = xTarget - pos.x;
            float ycmd = yTarget - pos.y;
            float zcmd = zTarget - pos.z;

            if(elapsed_move_time > ros::Duration(1.0)){
                ROS_INFO_STREAM("\nx: " << xcmd << "\t" << pos.x << "\ny: " << ycmd << "\t" << pos.y << "\nz: " << zcmd << "\t" << pos.z << "\nr: " << getRotation().z);
                start_move_time = ros::Time::now();
                fail_pos = getPos();
                if((std::abs(fail_pos.x - lastx) < 0.2) && (std::abs(fail_pos.y - lasty) < 0.2) && (std::abs(fail_pos.z - lastz) < 0.1)){
                    ROS_WARN_STREAM("drone stoped moving to early \n\tx: " << fail_pos.x << " - " << lastx << " = " << fail_pos.x - lastx << "\n\tx: " << fail_pos.y << " - " << lasty << " = " << fail_pos.y - lasty << "\n\tx: " << fail_pos.z << " - " << lastz << " = " << fail_pos.z - lastz );
                    if (fail_counter < 5){
                        fail_counter ++;
                    }
                    else{
                        state = IDLE;
                        ROS_ERROR_STREAM("drone stoped moving to early");
                        updateStatus(MAPPING_ALGORITM_DIDNT_FINISH_MOVE);
                    }
                } 
                else{
                    fail_counter = 0;
                } 
                lastx = fail_pos.x;
                lasty = fail_pos.y;
                lastz = fail_pos.z;
            }

            if(std::abs(xcmd) < SPEED_MIDDLE * 1.5){
                xcmd = (std::abs(xcmd) >= SPEED_SLOW) ? ((xcmd > 0) ? SPEED_SLOW : SPEED_SLOW * -1) : xcmd;
            }
            else if(std::abs(xcmd) < SPEED_FAST * 2){
                xcmd = xcmd > 0 ? SPEED_MIDDLE : SPEED_MIDDLE * -1;
            }
            else if(std::abs(xcmd) < DISTANCE_SLOWER){
                xcmd = xcmd > 0 ? SPEED_FAST : SPEED_FAST * -1;
            }
            else{
                xcmd = xcmd > 0 ? SPEED_MAX : SPEED_MAX * -1;
            }
            
            if(std::abs(ycmd) < SPEED_MIDDLE * 1.5){
                ycmd = (std::abs(ycmd) >= SPEED_SLOW) ? ((ycmd > 0) ? SPEED_SLOW : SPEED_SLOW * -1) : ycmd;
            }
            else if(std::abs(ycmd) < SPEED_FAST * 2){
                ycmd = ycmd > 0 ? SPEED_MIDDLE : SPEED_MIDDLE * -1;
            }
            else if(std::abs(ycmd) < DISTANCE_SLOWER){
                ycmd = ycmd > 0 ? SPEED_FAST : SPEED_FAST * -1;
            }
            else{
                ycmd = ycmd > 0 ? SPEED_MAX : SPEED_MAX * -1;
            }

            // if(std::abs(zcmd) < SPEED_MIDDLE){
            //     zcmd = (std::abs(zcmd) >= SPEED_SLOW) ? ((zcmd > 0) ? SPEED_SLOW : SPEED_SLOW * -1) : zcmd;
            // }
            // else if(std::abs(zcmd) < SPEED_FAST * 2){
            //     zcmd = zcmd > 0 ? SPEED_MIDDLE : SPEED_MIDDLE * -1;
            // }
            // else{
            //     zcmd = zcmd > 0 ? SPEED_FAST : SPEED_FAST * -1;
            // }
            sensor_msgs::Joy controlVelYawRate;

            controlVelYawRate.axes.push_back(xcmd);
            controlVelYawRate.axes.push_back(ycmd);
            controlVelYawRate.axes.push_back(zcmd);
            controlVelYawRate.axes.push_back(rTarget);
            controlVelYawRate.axes.push_back(flag);
            drone_ctrl_pos_yaw_pub.publish(controlVelYawRate);

            if(std::abs(xcmd) < 0.5 && std::abs(ycmd) < 0.5 && std::abs(zcmd) < 0.2){
                
                // ROS_INFO_STREAM("counter: " <<(int) counter);
                if(counter < 50){
                    counter ++;
                }
                else{
                    counter = 0;
                    state = BREAKING;
                }
            }
            else{
                counter = 0;
            }
        }
        else if(state == BREAKING){
            if(counter < 50){
                counter ++;
            }
            else{
                state = IDLE;
                ROS_INFO_STREAM("step finished");
                updateStatus(MAPPING_ALGORITM_NEXT_STEP);
            }

            sensor_msgs::Joy controlVelYawRate;

            controlVelYawRate.axes.push_back(0);
            controlVelYawRate.axes.push_back(0);
            controlVelYawRate.axes.push_back(0);
            controlVelYawRate.axes.push_back(0);
            controlVelYawRate.axes.push_back(breakflag);
            drone_ctrl_pos_yaw_pub.publish(controlVelYawRate);
        }
    }
}

bool ControlleDrone::StartMoveRelativeGround(float x, float y, float z, bool headless){
    ROS_INFO_STREAM("Start Step");
    updateStatus(MAPPING_ALGORITM_MOVING);

    state = MOVING;
    counter = 0;

    fail_counter = 0;
    fail_pos = getPos();
    ROS_INFO_STREAM("z:" << z << "\t" << fail_pos.z);
    xTarget = fail_pos.x + x;
    yTarget = fail_pos.y + y;
    zTarget = fail_pos.z + z;

    rTarget = headless ? getRotation().z : getDirectionAngle(x, y);

    lastx = 0;
    lasty = 0;
    lastz = 0;

    flag = (DJISDK::VERTICAL_VELOCITY   |
            DJISDK::HORIZONTAL_VELOCITY |
            DJISDK::YAW_ANGLE           |
            DJISDK::HORIZONTAL_GROUND   |
            DJISDK::STABLE_ENABLE);

    start_time = ros::Time::now();
    start_move_time = ros::Time::now();
}

float ControlleDrone::getDirectionAngle(float x, float y){
    if(x == 0 && y == 0){
        ROS_WARN_STREAM("Im not moving!!!");
        return getRotation().z;
    }
    else if(x == 0){
        return (y > 0) ? (PI / 2) : (-PI / 2); 
    }
    else if(y == 0){
        return (x > 0) ? 0 : PI;
    }
    else{
        float angle = std::abs(atan(y/x));
        if(x > 0 && y > 0){
            return angle;
        }
        else if(x < 0 && y > 0){
            return PI - angle;
        }
        else if(x > 0 && y < 0){
            return angle * -1;
        }
        else if(x < 0 && y < 0){
            return -PI + angle;
        }
        else{
            ROS_ERROR_STREAM("What dit just happen????");
        }
    }
}


ParrentDroneClass::ServiceAck ControlleDrone::obtainCtrlAuthority(){
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result){
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data);
}

bool ControlleDrone::takeoff(){
    float home_altitude = getGPS().altitude;
    
    uint8_t count = 0;
    while(count != 12){
        if(takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)){
            count = 12;
        }
        else if(count >= 5 || landing == true){
            ROS_ERROR_STREAM("faild to send take off command");
            return false;
        }
        else{
            count ++;
            ROS_WARN_STREAM("could not take off try again for " << count + 1<< "time");
            ros::Duration(1.0).sleep();
        }
    }

    ros::Time start_time = ros::Time::now();

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    while(ros::Time::now() - start_time < ros::Duration(10.0) && landing == false){
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(landing == true){
        return false;
    }

    if(getFlightStatus() != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || getGPS().altitude - home_altitude < 1.0){
        ROS_ERROR_STREAM("Takeoff failed");
        return false;
    }
    else{
        ROS_INFO("Succesfful takeoff!");
        ros::spinOnce();
        return true;
    }

}

bool ControlleDrone::land(){
    uint8_t count = 0;

    while(count != 12){
        if(takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND)){
            count = 12;
        }
        else if(count >= 5){
            ROS_ERROR_STREAM("faild to send land command");
            return true;
        }
        else{
            count ++;
            ROS_WARN_STREAM("could not land try again for " << count + 1<< "time");
            ros::Duration(1.0).sleep();
        }
    }

    ros::Time start_time = ros::Time::now();

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    while(ros::Time::now() - start_time < ros::Duration(5.0)){
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    while(getFlightStatus() == DJISDK::M100FlightStatus::M100_STATUS_LANDING){
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(getFlightStatus() != DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND && getFlightStatus() != DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING){
        ROS_ERROR_STREAM("land failed");
        return true;
    }
    else{
        ROS_INFO("Succesfful land!");
        ros::spinOnce();
        return false;
    }
}

bool ControlleDrone::takeoff_land(int task){
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }

    return true;
}