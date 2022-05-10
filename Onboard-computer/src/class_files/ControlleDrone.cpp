#include <sar_drone/ControlleDrone.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;

ControlleDrone::ControlleDrone(){
    if(ready){
        flying = false;
        landing = false;
        gotCtrlAuthority = false;
        home_altitude = 0;

        ros::CallbackQueue *PRIO_Calback_queue_ptr = &PRIO_Calback_queue;
        nh_PRIO.setCallbackQueue(PRIO_Calback_queue_ptr);

        drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
        sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
        set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

        drone_commands_sub = nh.subscribe(DIRECTIONS_TOPPIC, 50, &ControlleDrone::directionsCallback, this);
        drone_PRIO_commands_sub = nh_PRIO.subscribe(DIRECTIONS_PRIO_TOPPIC, 50, &ControlleDrone::directionsPRIOCallback, this);
        
        drone_status_pub = nh.advertise<sar_drone::status>(STATUS_TOPPIC, 10);
        drone_ctrl_pos_yaw_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
        send_mobile_data_pub = nh.advertise<sar_drone::send_mobile>(SEND_TO_MOBILE, 10);

        updateStatus(STARTING_UP);

        PRIO_thread_spinner = std::thread([this, PRIO_Calback_queue_ptr](){
            ros::SingleThreadedSpinner PRIO_spinner;
            PRIO_spinner.spin(PRIO_Calback_queue_ptr);
        });

        ServiceAck ack = obtainCtrlAuthority();
        if (ack.result){
            ROS_INFO("Obtain SDK control Authority successfully");
            gotCtrlAuthority = true;
        }
        else{
            if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0){
                ROS_INFO("Obtain SDK control Authority in progess, send the cmd again");
                obtainCtrlAuthority();
            }
            else{
                ROS_WARN("Failed Obtain SDK control Authority");
                gotCtrlAuthority = false;
                
            }
        }

        spin(1.0);
        // ROS_INFO_STREAM("\nGPS:\n\tlatitude: " <<(double) getGPS().latitude << "\n\tlongitude: " << getGPS().longitude << "\n\taltitude: " << getGPS().altitude << "\nRotation: " << getRotation().z << "\npoint \n\tx: " << getPos().x << "\n\ty: " << getPos().y << "\n\tz: " << getPos().z);
        
        updateStatus(ON_GROUND);
    }
    else{
        updateStatus(ERROR);
        state = IDLE;
    }
    
    
}

ControlleDrone::~ControlleDrone(){
}

void ControlleDrone::directionsCallback(const sar_drone::directions::ConstPtr& msg){
    //ROS_INFO_STREAM("I got " << (uint)msg->Directions << " from " << (uint)msg->ID);
    if(flying && !landing)
    { 
        switch(Status){
            case START_HUMAN_DETECTION:
            case MAPPING_ALGORITM_NEXT_STEP:{
                switch (static_cast<msgCommands>(msg->Command)){
                    case MA_MOVE_RELATIVE_GROUND_HEADLESS:{
                        StartMoveDrone(msg->x, msg->y, msg->z, true, true);
                        break;
                    }

                    case MA_MOVE_RELATIVE_GROUND:{
                        StartMoveDrone(msg->x, msg->y, msg->z, false, true);
                        break;
                    }

                    case MA_START_HUMAN_DETECT:{
                        updateStatus(START_HUMAN_DETECTION);
                        break;
                    }

                    case MA_MOVE_COORDINATES:{
                        sensor_msgs::NavSatFix tmp;
                        tmp.longitude = msg->Longitude;
                        tmp.latitude = msg->Latitude;
                        StartMoveDrone(tmp, false);
                        updateStatus(START_HUMAN_DETECTION);
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
                // ROS_INFO_STREAM("Human detection controll");
                switch (static_cast<msgCommands>(msg->Command)){
                    case HD_FORWARD:{
                        StartMoveDrone(HD_MOVE, 0, 0, true, false);
                        break;
                    }

                    case HD_BACKWARD:{
                        StartMoveDrone(-HD_MOVE, 0, 0, true, false);
                        break;
                    }

                    case HD_LEFT:{
                        StartMoveDrone(0, -HD_MOVE, 0, true, false);
                        break;
                    }

                    case HD_RIGHT:{
                        StartMoveDrone(0, HD_MOVE, 0, true, false);
                        break;
                    }

                    case HD_FORWARD_LEFT:{
                        StartMoveDrone(HD_MOVE, -HD_MOVE, 0, true, false);
                        break;
                    }

                    case HD_FORWARD_RIGHT:{
                        StartMoveDrone(HD_MOVE, HD_MOVE, 0, true, false);
                        break;
                    }

                    case HD_BACKWARD_LEFT:{
                        StartMoveDrone(-HD_MOVE, -HD_MOVE, 0, true, false);
                        break;
                    }

                    case HD_BACKWARD_RIGHT:{
                        StartMoveDrone(-HD_MOVE, HD_MOVE, 0, true, false);
                        break;
                    }

                    case HD_UP:{
                        StartMoveDrone( 0, 0, HD_MOVE, true, false);
                        break;
                    }

                    case HD_DOWN:{
                        StartMoveDrone(0, 0, -HD_MOVE, true, false);
                        break;
                    }
                }
            }
        }
        ros::Duration(0.1).sleep();
    }
}

void ControlleDrone::directionsPRIOCallback(const sar_drone::directions::ConstPtr& msg){
    switch (static_cast<msgCommands>(msg->Command)){
        case MANUAL_OVERRIDE:{
            ROS_WARN_STREAM("Taking Manual controle of drone!");

            state = IDLE;
            updateStatus(MANUAL_CONTROLL);
            break;
        }

        case TAKE_OFF:{
            sar_drone::send_mobile toMobile;
            toMobile.cmdID = TAKE_OFF;

            if(getFlightStatus() == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR){
                ROS_WARN_STREAM("drone alreay flying");
                toMobile.errorCode = DRONE_ALREADY_FLYING;
            }
            else{
                ROS_INFO_STREAM("takeoff");
                updateStatus(TAKING_OFF);
                flying = takeoff();
                
                toMobile.errorCode = flying ? NO_ERROR : ERROR_WITH_TAKEOFF;
            }
            updateStatus(MAPPING_ALGORITM_NEXT_STEP);
            send_mobile_data_pub.publish(toMobile);
            break;
        }

        case LAND:{
            sar_drone::send_mobile toMobile;
            toMobile.cmdID = LAND;
            toMobile.errorCode = START_LANDING;
            send_mobile_data_pub.publish(toMobile);
                    
            updateStatus(LANDING);

            if(!flying && getFlightStatus() == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR){
                ROS_WARN_STREAM("drone is flying");
                flying = true;
            }

            if(!landing && flying){
                ROS_INFO_STREAM("land");
                state = IDLE;
                landing = true;
                landing = land();
                if(!landing){
                    flying = false;
                    updateStatus(ON_GROUND);
                }
                toMobile.errorCode = flying ? ERROR_WITH_LANDING : NO_ERROR;
            }
            else{
                ROS_INFO_STREAM("already on the ground");
                updateStatus(ON_GROUND);
                toMobile.errorCode = DRONE_ALREADY_ON_GROUND;
            }

            send_mobile_data_pub.publish(toMobile);
            break;
        }
        
        case HD_FOUND_PERSON_I_THINK:{
            ROS_WARN_STREAM("found potential person");
            updateStatus(HUMAN_DETECTION_NEXT_STEP);
            StopMoving();
            break;
            // whoIsInControl = HUMANDETECTION;
        }

        case HD_I_WAS_WRONG:{
            ROS_WARN_STREAM("nope i'm stupide");
            updateStatus(MAPPING_ALGORITM_NEXT_STEP);
            break;
        }

        case HD_FOUND_PERSON_IM_SERTAIN:{
            ROS_WARN_STREAM("yesss i'm smarttttt");
            
            sar_drone::send_mobile toMobile;
            toMobile.cmdID = HD_FOUND_PERSON_IM_SERTAIN;
            toMobile.errorCode = NO_ERROR;
            toMobile.Latitude = getGPS().latitude;
            toMobile.Longitude = getGPS().longitude;

            send_mobile_data_pub.publish(toMobile);
            break;
        }

        case HD_START_AI:{
            
        }

        default:
            ROS_WARN_STREAM("wrong command for directions PRIO Calback");
            break;
        }
}

void ControlleDrone::step(double sleepTime){
    spin(sleepTime);
    if(Status == MANUAL_CONTROLL || Status == ERROR){
        // ROS_WARN_STREAM("manual controll");
        
        if(gotCtrlAuthority)
        {
            sar_drone::send_mobile toMobile;
            toMobile.cmdID = MANUAL_OVERRIDE;

            state = IDLE;
            updateStatus(MANUAL_CONTROLL);

            bool finished;
            uint8_t tmpcounter = 0;

            do{
                ServiceAck ack = releaseControle();
                if (ack.result){
                    ROS_INFO("released SDK control Authority successfully");
                    
                    toMobile.errorCode = MobileErrorCodes::NO_ERROR;

                    finished = true;
                    gotCtrlAuthority = false;
                }
                else{
                    finished = false;
                    if(tmpcounter < 5){
                        tmpcounter ++;
                        ROS_WARN("Failed release SDK control Authority");
                    }
                    else{
                        ROS_ERROR_STREAM("You're Fucked ;) \n trying emergency landing");
                        toMobile.errorCode = MobileErrorCodes::CANT_GIVE_MANUAL_CONTROLLE;
                        land();
                        finished = true;
                    }
                    
                    spin(0.1);
                }
            }while(!finished);
            send_mobile_data_pub.publish(toMobile);
        }
        
        spin(0.1);
    }
    else
    {
        elapsed_time = ros::Time::now() - start_time;
        elapsed_move_time = ros::Time::now() - start_move_time;
        if(elapsed_time > ros::Duration(0.02)){
            start_time = ros::Time::now();
            switch(state){
                case MOVING:{
                    geometry_msgs::Point pos = translateGPS(startGPS, getGPS());
                    geometry_msgs::Point cmd;

                    cmd.x = Target.x - pos.x;
                    cmd.y = Target.y - pos.y;
                    cmd.z = Target.z - pos.z;

                    if(elapsed_move_time > ros::Duration(1.0)){
                        //ROS_INFO_STREAM("\nx: " << cmd.x << "\t" << pos.x << "\ny: " << cmd.y << "\t" << pos.y << "\nz: " << cmd.z << "\t" << pos.z << "\nr: " << getRotation().z);
                        start_move_time = ros::Time::now();
                        if((std::abs(pos.x - lastPos.x) < 0.2) && (std::abs(pos.y - lastPos.y) < 0.2) && (std::abs(pos.z - lastPos.z) < 0.1)){
                            ROS_WARN_STREAM("drone stoped moving to early \n\tx: " << pos.x << " - " << lastPos.x << " = " << pos.x - lastPos.x << "\n\tx: " << pos.y << " - " << lastPos.y << " = " << pos.y - lastPos.y << "\n\tx: " << pos.z << " - " << lastPos.z << " = " << pos.z - lastPos.z );
                            if (fail_counter_nm < 5){
                                fail_counter_nm ++;
                            }
                            else{
                                state = IDLE;
                                ROS_ERROR_STREAM("drone stoped moving to early");
                                updateStatus(MAPPING_ALGORITM_DIDNT_FINISH_MOVE);
                            }
                        } 
                        else{
                            fail_counter_nm = 0;
                        } 

                        if((std::abs(pos.x - Target.x) > std::abs(cmd.x) + 2) || (std::abs(pos.y - Target.y) > std::abs(cmd.y) + 2) || (std::abs(pos.z - Target.z) > std::abs(cmd.z) + 1)){
                            ROS_WARN_STREAM("drone not on route \n\tx: " << pos.x << " - " << Target.x << " = " << std::abs(pos.x - Target.x) << " > " <<  std::abs(cmd.x) + 2 << "\n\tx: " << pos.y << " - " << Target.y << " = " << std::abs(pos.y - Target.y) << " > " <<  std::abs(cmd.y) + 2 << "\n\tx: " << pos.z << " - " << Target.z << " = " << std::abs(pos.z - Target.z) << " > " << std::abs(cmd.z) + 1 );
                            if (fail_counter_OOB < 5){
                                fail_counter_OOB ++;
                            }
                            else{
                                state = IDLE;
                                ROS_ERROR_STREAM("drone not on route");
                                updateStatus(MAPPING_ALGORITM_DIDNT_FINISH_MOVE);
                            }
                        } 
                        else{
                            fail_counter_OOB = 0;
                        } 
                        lastPos.x = pos.x;
                        lastPos.y = pos.y;
                        lastPos.z = pos.z;
                    }

                    if(std::abs(cmd.x) < SPEED_MIDDLE * 1.5 || std::abs(pos.x) < SPEED_MIDDLE * 1.5){
                        cmd.x = (std::abs(cmd.x) >= SPEED_SLOW) ? ((cmd.x > 0) ? SPEED_SLOW : SPEED_SLOW * -1) : cmd.x;
                    }
                    else if(std::abs(cmd.x) < SPEED_FAST * 2 || std::abs(pos.x) < SPEED_FAST * 2){
                        cmd.x = cmd.x > 0 ? SPEED_MIDDLE : SPEED_MIDDLE * -1;
                    }
                    else if(std::abs(cmd.x) < DISTANCE_SLOWER || std::abs(pos.x) < DISTANCE_SLOWER){
                        cmd.x = cmd.x > 0 ? SPEED_FAST : SPEED_FAST * -1;
                    }
                    else{
                        cmd.x = cmd.x > 0 ? SPEED_MAX : SPEED_MAX * -1;
                    }
                    
                    if(std::abs(cmd.y) < SPEED_MIDDLE * 1.5 || std::abs(pos.y) < SPEED_MIDDLE * 1.5){
                        cmd.y = (std::abs(cmd.y) >= SPEED_SLOW) ? ((cmd.y > 0) ? SPEED_SLOW : SPEED_SLOW * -1) : cmd.y;
                    }
                    else if(std::abs(cmd.y) < SPEED_FAST * 2 || std::abs(pos.y) < SPEED_FAST * 2){
                        cmd.y = cmd.y > 0 ? SPEED_MIDDLE : SPEED_MIDDLE * -1;
                    }
                    else if(std::abs(cmd.y) < DISTANCE_SLOWER || std::abs(pos.y) < DISTANCE_SLOWER){
                        cmd.y = cmd.y > 0 ? SPEED_FAST : SPEED_FAST * -1;
                    }
                    else{
                        cmd.y = cmd.y > 0 ? SPEED_MAX : SPEED_MAX * -1;
                    }

                    sensor_msgs::Joy controlVelYawRate;

                    controlVelYawRate.axes.push_back(cmd.x);
                    controlVelYawRate.axes.push_back(cmd.y);
                    controlVelYawRate.axes.push_back(cmd.z);
                    controlVelYawRate.axes.push_back(rTarget);
                    controlVelYawRate.axes.push_back(flag);
                    drone_ctrl_pos_yaw_pub.publish(controlVelYawRate);

                    if(std::abs(cmd.x) < HORIZON_THRESHOLD && std::abs(cmd.y) < HORIZON_THRESHOLD && std::abs(cmd.z) < VERTICAL_THRESHOLD && std::abs(getRotation().z - rTarget) < YAW_THRESHOLD){
                        
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
                    break;
                }

                case ROTATING:{
                    sensor_msgs::Joy controlVelYawRate;

                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(rTarget);
                    controlVelYawRate.axes.push_back(flag);
                    drone_ctrl_pos_yaw_pub.publish(controlVelYawRate);

                    if(std::abs(getRotation().z - rTarget) < YAW_THRESHOLD){
                        
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
                    break;
                    break;
                }

                case BREAKING:{
                    if(counter < 50){
                        counter ++;
                    }
                    else{
                        state = IDLE;
                        ROS_INFO_STREAM("step finished");
                        updateStatus(NEXT_STEP);
                    }

                    sensor_msgs::Joy controlVelYawRate;

                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(0);
                    controlVelYawRate.axes.push_back(breakflag);
                    drone_ctrl_pos_yaw_pub.publish(controlVelYawRate);
                    break;
                }
            }
        }
    }
}

void ControlleDrone::updateStatus(statusCodes newStatus){
    sar_drone::status pub_msg;

    switch(newStatus){
        case MOVING_SC:
            switch(Status){
                case TAKING_OFF:
                case START_HUMAN_DETECTION:
                case MAPPING_ALGORITM_NEXT_STEP:
                        pub_msg.Status = MAPPING_ALGORITM_MOVING;
                        OldStatus = Status;
                        Status = MAPPING_ALGORITM_MOVING;
                    break;
                
                case HUMAN_DETECTION_NEXT_STEP:
                        pub_msg.Status = HUMAN_DETECTION_MOVING;
                        OldStatus = Status;
                        Status = HUMAN_DETECTION_MOVING;
                    break;
                
                default:
                    ROS_WARN_STREAM("You're doing something wrong again MC: " << (int) OldStatus);
            }
            break;
        
        case NEXT_STEP:
            switch(Status){
                case MAPPING_ALGORITM_MOVING:
                        pub_msg.Status = MAPPING_ALGORITM_NEXT_STEP;
                        OldStatus = Status;
                        Status = MAPPING_ALGORITM_NEXT_STEP;
                    break;
                
                case HUMAN_DETECTION_MOVING:
                        pub_msg.Status = HUMAN_DETECTION_NEXT_STEP;
                        OldStatus = Status;
                        Status = HUMAN_DETECTION_NEXT_STEP;
                    break;
                
                default:
                    ROS_WARN_STREAM("You're doing something wrong again NS: " << (int) OldStatus);
            }
            break;
        
        default:
            pub_msg.Status = newStatus;
            OldStatus = Status;
            Status = newStatus;
            break;
    }
    
    drone_status_pub.publish(pub_msg);
}

void ControlleDrone::StartMoveDrone(float x, float y, float z, bool headless, bool relative_ground){
    if(Status != MANUAL_CONTROLL){
        ROS_INFO_STREAM("Start Move");
        updateStatus(MOVING_SC);

        state = MOVING;
        counter = 0;

        fail_counter_nm = 0;

        startGPS = getGPS();

        lastPos.x = 0;
        lastPos.y = 0;
        lastPos.z = 0;

        flag = (DJISDK::VERTICAL_POSITION   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_ANGLE           |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);

        if(relative_ground){
            rTarget = headless ? getRotation().z : getDirectionAngle(x, y);
            Target.x = x;
            Target.y = y;
            Target.z = (getGPS().altitude - home_altitude) + z;
        }
        else{
            rTarget = headless ? getRotation().z : getRotation().z + getDirectionAngle(x, y);
            std::pair<float, float> tmp = remapDirections(x, y, getRotation().z);

            // ROS_WARN_STREAM("whyyyy");
            // ROS_WARN_STREAM("x: " << x << "\t" << tmp.first);
            // ROS_WARN_STREAM("y:" << y << "\t" << tmp.second);

            Target.x = tmp.first;
            Target.y = tmp.second;
            Target.z = (getGPS().altitude - home_altitude) + z;
        }
        
        if(!gotCtrlAuthority){
            ServiceAck ack = obtainCtrlAuthority();
            if (ack.result){
                ROS_INFO("Obtain SDK control Authority successfully");
                gotCtrlAuthority = true;
            }
            else{
                if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0){
                    ROS_INFO("Obtain SDK control Authority in progess, send the cmd again");
                    obtainCtrlAuthority();
                }
                else{
                    ROS_WARN("Failed Obtain SDK control Authority");
                    updateStatus(ERROR);
                }
            }
        }

        start_time = ros::Time::now();
        start_move_time = ros::Time::now();
    }
}

void ControlleDrone::StartMoveDrone(sensor_msgs::NavSatFix dest, bool headless){
    geometry_msgs::Point tmp = translateGPS(getGPS(), dest);
    StartMoveDrone(tmp.x, tmp.y, 0, headless, true);
}

void ControlleDrone::StartRotate(float ofset, bool relative_current_rot){
    if(Status != MANUAL_CONTROLL){
        ROS_INFO_STREAM("Start Rotate");
        updateStatus(MOVING_SC);

        state = ROTATING;
        counter = 0;

        rTarget = relative_current_rot ? getRotation().z + ofset : ofset;

        flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_ANGLE           |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
        
        if(!gotCtrlAuthority){
            ServiceAck ack = obtainCtrlAuthority();
            if (ack.result){
                ROS_INFO("Obtain SDK control Authority successfully");
                gotCtrlAuthority = true;
            }
            else{
                if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0){
                    ROS_INFO("Obtain SDK control Authority in progess, send the cmd again");
                    obtainCtrlAuthority();
                }
                else{
                    ROS_WARN("Failed Obtain SDK control Authority");
                    updateStatus(ERROR);
                }
            }
        }

        start_time = ros::Time::now();
        start_move_time = ros::Time::now();
    }
}

void ControlleDrone::StopMoving(){
    if(Status != MANUAL_CONTROLL){
        ROS_INFO_STREAM("Start Breaking");
        updateStatus(MOVING_SC);

        state = BREAKING;
        
        counter = 0;

        if(!gotCtrlAuthority){
            ServiceAck ack = obtainCtrlAuthority();
            if (ack.result){
                ROS_INFO("Obtain SDK control Authority successfully");
                gotCtrlAuthority = true;
            }
            else{
                if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0){
                    ROS_INFO("Obtain SDK control Authority in progess, send the cmd again");
                    obtainCtrlAuthority();
                }
                else{
                    ROS_WARN("Failed Obtain SDK control Authority");
                    updateStatus(ERROR);
                }
            }
        }
        start_time = ros::Time::now();
        start_move_time = ros::Time::now();
    }

}

float ControlleDrone::getDirectionAngle(float x, float y){
    if(x == 0 && y == 0){
        ROS_WARN_STREAM("Im not moving horizontally!!!");
        return getRotation().z;
    }
    else if(x == 0){
        return (y > 0) ? (M_PI / 2) : (-M_PI / 2); 
    }
    else if(y == 0){
        return (x > 0) ? 0 : M_PI;
    }
    else{
        float angle = std::abs(atan(y/x));
        if(x > 0 && y > 0){
            return angle;
        }
        else if(x < 0 && y > 0){
            return M_PI - angle;
        }
        else if(x > 0 && y < 0){
            return angle * -1;
        }
        else if(x < 0 && y < 0){
            return -M_PI + angle;
        }
        else{
            ROS_ERROR_STREAM("What dit just happen????");
        }
    }
}

std::pair<float, float> ControlleDrone::remapDirections(float x, float y, float r){
    if(x == 0 && y == 0){
        ROS_WARN_STREAM("Im not moving horizontally!!!");
        return std::pair<float, float>{0,0};
    }
    else{
        float d;
        if(x == 0){
            d = std::abs(y);
        }
        else if( y == 0){
            d = std::abs(x);
        }
        else{
            d = (float)sqrt((float)pow(x, 2) * pow(y, 2));
        }
        float dr = (float)getDirectionAngle(x, y) + r;
        ROS_WARN_STREAM("d: " << d << "\tdr:" << dr);

        std::pair<float, float>tmp(sin(dr) * d, cos(dr) * d);
        
        ROS_WARN_STREAM("x: " << x << "\t" << tmp.first);
        ROS_WARN_STREAM("y:" << y << "\t" << tmp.second);
        return tmp;
    }
}

geometry_msgs::Point ControlleDrone::translateGPS(sensor_msgs::NavSatFix origin, sensor_msgs::NavSatFix offset){
    origin.longitude = origin.longitude * M_PI / 180;
    origin.latitude = origin.latitude * M_PI / 180;
    offset.longitude = offset.longitude * M_PI / 180;
    offset.latitude = offset.latitude * M_PI / 180;

    long double distance = RADIUS_EARTH * acos((sin(origin.latitude) * sin(offset.latitude)) + (cos(origin.latitude) * cos(offset.latitude) * cos(origin.longitude - offset.longitude)));

    long double phi = cos(origin.latitude) * sin(offset.latitude) - sin(origin.latitude) * cos(offset.latitude) * cos(offset.longitude - origin.longitude);
    long double lon = sin(offset.longitude - origin.longitude) * cos(offset.latitude);
    long double heading = atan2(lon, phi);
    
    geometry_msgs::Point tmp;
    tmp.x = sin(heading) * distance;
    tmp.y = cos(heading) * distance;
    tmp.z = offset.altitude - home_altitude;

    return tmp;
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

ParrentDroneClass::ServiceAck ControlleDrone::releaseControle(){
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 0;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result){
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data);
}

bool ControlleDrone::takeoff(){
    if (!set_local_position()) // We need this for height
    {
        ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
        return false;
    }

    home_altitude = getGPS().altitude;
    
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

    if(getFlightStatus() != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || getGPS().altitude - home_altitude < 0.75){
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
    if(!gotCtrlAuthority){
        ServiceAck ack = obtainCtrlAuthority();
        if (ack.result){
            ROS_INFO("Obtain SDK control Authority successfully");
            gotCtrlAuthority = true;
        }
        else{
            if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0){
                ROS_INFO("Obtain SDK control Authority in progess, send the cmd again");
                obtainCtrlAuthority();
            }
            else{
                ROS_WARN("Failed Obtain SDK control Authority");
                updateStatus(ERROR);
            }
        }
    }

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
