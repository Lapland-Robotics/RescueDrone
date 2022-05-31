#include <sar_drone/mapAlg.h>

using namespace SaR_Drone;

mapAlg::mapAlg(): msg_ID(2)
{
    if(ready){
       status_drone = ON_GROUND;
    
        local_status = IDLE;
        drone_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_TOPPIC, 10);
        drone_PRIO_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_PRIO_TOPPIC, 10);
        send_mobile_data_pub = nh.advertise<sar_drone::send_mobile>(SEND_TO_MOBILE, 10);
        
        map_commands_sub = nh.subscribe<sar_drone::directions>(MAP_TOPPIC, 10, &mapAlg::mapCommandsCalback, this);
        drone_status_sub = nh.subscribe<sar_drone::status>(STATUS_TOPPIC, 10, [this](const sar_drone::status::ConstPtr& msg){
            status_drone = static_cast<statusCodes>(msg->Status);
            ROS_WARN_STREAM("status: " << status_drone);
        });
        
        route_index = 0;
        demo = false;
        got_route = false;
        on_site = false; 
    }
    else{
        route_planner = nh.serviceClient<sar_drone::routeplanner>(ROUTE_PLANNER_TOPPIC);

        std::vector<sar_drone::coordinates> tmp;
        sar_drone::coordinates smalltmp;

        smalltmp.longitude = 25.7270694; // BL
        smalltmp.latitude = 66.4841275;
        tmp.push_back(smalltmp);
        smalltmp.longitude = 25.7299098; // TL
        smalltmp.latitude = 66.4835189;
        tmp.push_back(smalltmp);
        smalltmp.longitude = 25.7307306; // TR
        smalltmp.latitude = 66.4841160;
        tmp.push_back(smalltmp);
        smalltmp.longitude = 25.7279125; // BR
        smalltmp.latitude = 66.4847368; 
        tmp.push_back(smalltmp);

        fake_origin.longitude = 25.727553;
        fake_origin.latitude = 66.484703;
        CreateRoute(tmp);
    }
    
}

mapAlg::~mapAlg()
{

}

void mapAlg::step(double sleepTime){
    ros::spinOnce();
    if (status_drone != MANUAL_CONTROLL && status_drone != ERROR){
        switch (local_status){
            case OFF:{
                break;
            }

            case IDLE:{
                //waiting for start coordinates
                break;
            }

            case GOING_TO_START:{
                ROS_INFO_STREAM("setting home point");

                sensor_msgs::NavSatFix GPSTmp = getGPS();
                home_base.latitude = GPSTmp.latitude;
                home_base.longitude = GPSTmp.longitude;

                ROS_INFO_STREAM("Go to start");
                
                sar_drone::directions return_msg;
                sar_drone::coordinates coord_msg;

                coord_msg = start_location;
                coord_msg.altitude = SEARCH_ALTITUDE;

                return_msg.ID = msg_ID;
                return_msg.Command = MA_MOVE_COORDINATES;
                return_msg.coordinate.push_back(coord_msg);
                drone_commands_pub.publish(return_msg);

                next_local_status = WAIT_STARTING;
                local_status = MOVE_COMMAND_SEND;

                break;
            }

            case START_HUMAN:{
                sar_drone::directions return_msg;
                return_msg.ID = msg_ID;
                return_msg.Command = MA_START_HUMAN_DETECT;

                drone_commands_pub.publish(return_msg);

                next_local_status = WAIT_MOVING;
                local_status = START_HUMAN_SEND;

                break;
            }

            case NEXT_MOVE:{
                sar_drone::directions return_msg;
                return_msg.ID = msg_ID;
                return_msg.Command = MA_MOVE_RELATIVE_GROUND;

                return_msg.position = route[route_index];

                drone_commands_pub.publish(return_msg);
                
                ROS_INFO_STREAM("Next Move send");
                route_index ++;

                //ROS_INFO_STREAM("route index: " <<(int) route_index << "\troute size: " <<(int) route.size());

                next_local_status = route_index == route.size() ? RTH_MSG : WAIT_MOVING;
                local_status = MOVE_COMMAND_SEND;
                
                break;

            }

            case START_HUMAN_SEND:{
                if(status_drone != MAPPING_ALGORITM_NEXT_STEP){
                    local_status = next_local_status;
                    start_time = ros::Time::now();
                }
                break;
            }
            
            case MOVE_COMMAND_SEND:{
                if(status_drone != MAPPING_ALGORITM_NEXT_STEP && status_drone != START_HUMAN_DETECTION){
                    local_status = next_local_status;
                    start_time = ros::Time::now();
                }
                break;
            }

            case WAIT_MOVING:{
                //waiting for drone stoped moving
                switch(status_drone){
                    case START_HUMAN_DETECTION:
                    case MAPPING_ALGORITM_NEXT_STEP:
                        ROS_INFO_STREAM("next move");
                        local_status = NEXT_MOVE;
                        break;
                    
                    case TAKING_OFF:
                    case LANDING:
                    case MAPPING_ALGORITM_MOVING:
                    case HUMAN_DETECTION_NEXT_STEP:
                    case HUMAN_DETECTION_MOVING:
                    case BACK_TO_MAPPING_ALG_MOVING:
                    case BACK_TO_MAPPING_ALG_NEXT_STEP:
                        break;

                    case ON_GROUND:
                        local_status = OFF;
                        break;
                    
                    case MAPPING_ALGORITM_DIDNT_FINISH_MOVE:
                        local_status = STOPPING;
                        break;

                    default:
                        ROS_ERROR_STREAM("serious what are you doing? I don't know this command (" << (int) status_drone << ")");
                        break;
                }
                
                break;
            }

            case WAIT_STARTING:{
                //waiting for drone stoped moving
                switch(status_drone){
                    case MAPPING_ALGORITM_NEXT_STEP:
                        ROS_INFO_STREAM("go to start");
                        local_status = on_site ? (got_route ? START_HUMAN : WAIT_STARTING) : GOING_TO_START;
                        break;
                    
                    case TAKING_OFF:
                    case ON_GROUND:
                    case MAPPING_ALGORITM_MOVING:
                        break;

                    default:
                        ROS_ERROR_STREAM("serious what are you doing? I don't know this command (" << (int) status_drone << ")");
                        break;
                }
                
                break;
            }
            case RTH_MSG:{
                ROS_WARN_STREAM("i'm finished can i come home???");
                sar_drone::send_mobile toMobile;
                toMobile.cmdID = STOP_SEARCH;
                toMobile.errorCode = MobileErrorCodes::I_AM_FINISHED_CAN_I_GO_HOME;
                send_mobile_data_pub.publish(toMobile);
                local_status = WAIT_RTH;
            }

            case WAIT_RTH:{
                elapsed_time = ros::Time::now() - start_time;
                if(elapsed_time > ros::Duration(30.0)){
                   local_status = RTH;
                }
                break;
            }

            case RTH:{
                ROS_WARN_STREAM("Going home");
                
                sar_drone::directions return_msg;
                sar_drone::coordinates coord_msg;

                coord_msg = home_base;
                coord_msg.altitude = SEARCH_ALTITUDE;

                return_msg.ID = msg_ID;
                return_msg.Command = MA_MOVE_COORDINATES;
                return_msg.coordinate.push_back(coord_msg);
                drone_commands_pub.publish(return_msg);

                local_status = STOPPING;

                break;
            }

            case STOPPING:{
                switch(status_drone){
                    case ON_GROUND:
                        local_status = IDLE;
                        break;

                    case MAPPING_ALGORITM_MOVING:
                        start_time = ros::Time::now();
                        break;

                    case MAPPING_ALGORITM_NEXT_STEP:{
                        elapsed_time = ros::Time::now() - start_time;
                        if(elapsed_time > ros::Duration(15.0)){
                            ROS_INFO_STREAM("stop searching");

                            route_index = 0;
                            got_route = false;

                            sar_drone::directions return_msg;
                            return_msg.ID = msg_ID;
                            return_msg.Command = LAND;
                            drone_PRIO_commands_pub.publish(return_msg);
                            
                            local_status = IDLE;
                        }
                        break;
                    }
                    case MAPPING_ALGORITM_DIDNT_FINISH_MOVE:{
                        ROS_INFO_STREAM("stop by error");
                        route_index = 0;
                        got_route = false;

                        sar_drone::directions return_msg;
                        return_msg.ID = msg_ID;
                        return_msg.Command = LAND;
                        drone_PRIO_commands_pub.publish(return_msg);
                        
                        local_status = IDLE;
                        break;
                    }
                }
                break;
            }

            case STOP_NOW:{
                route_index = 0;
                got_route = false;

                sar_drone::directions return_msg;
                return_msg.ID = msg_ID;
                return_msg.Command = LAND;
                drone_PRIO_commands_pub.publish(return_msg);
                
                local_status = MOVE_COMMAND_SEND;
                next_local_status = WAIT_MOVING;
                break;
            }

            default:{
                ROS_ERROR_STREAM("WHAT THE HELLL DID YOU JUST DO?");
                break;
            }
        }
    }
    ros::Duration(sleepTime).sleep();
}

void mapAlg::mapCommandsCalback(const sar_drone::directions::ConstPtr& msg){
    switch (static_cast<msgCommands>(msg->Command))
    {
        case TAKE_OFF:{
            ROS_INFO_STREAM("Take off");

            local_status = WAIT_STARTING;
            
            sar_drone::directions return_msg;
            return_msg.ID = msg_ID;
            return_msg.Command = TAKE_OFF;
            drone_PRIO_commands_pub.publish(return_msg);
            break;
        }

        case STOP_SEARCH:{
            local_status = RTH;
            break;
        }

        case AREA_COORDINATES:{            
            CreateRoute(msg->coordinate);
        }

        case EMERGENCY_SHUTDOWN:
        case EMERGENCY_LAND:
            break;

        default:
            break;
    }
}

void mapAlg::CreateRoute(const std::vector<sar_drone::coordinates> &area){
    ROS_INFO_STREAM("Create route");
    
    on_site = false;
    
    route.clear();
    route_index = 0;
    
    if(demo){
        sar_drone::rel_coordinates tmp;
        tmp.x = 10; tmp.y = 0; route.push_back(tmp);
        tmp.x = 0; tmp.y = 5; route.push_back(tmp);
        tmp.x = -10; tmp.y = 0; route.push_back(tmp);
        tmp.x = 0; tmp.y = 5; route.push_back(tmp);
        tmp.x = 10; tmp.y = 0; route.push_back(tmp);
        tmp.x = 0; tmp.y = 5; route.push_back(tmp);
        tmp.x = -10; tmp.y = 0; route.push_back(tmp);
        tmp.x = 0; tmp.y = 5; route.push_back(tmp);
        tmp.x = 10; tmp.y = 0; route.push_back(tmp);

        
        sensor_msgs::NavSatFix GPSTmp = getGPS();
        start_location.latitude = GPSTmp.latitude;
        start_location.longitude = GPSTmp.longitude;
    }
    else{
        std::vector<AreaStruct> areaCorner;
        sar_drone::routeplanner msg;

        double smallestDist = -1.20301;
        for(auto &i : area){
            sensor_msgs::NavSatFix gpsTmp;
            AreaStruct areaTmp;
            gpsTmp.latitude = i.latitude;
            gpsTmp.longitude = i.longitude;
            areaTmp.Point = translateGPSArea(ready ? getGPS() : fake_origin, gpsTmp, false);
            if(smallestDist == -1.20301 || smallestDist > areaTmp.Point.z){
                for(auto &j : areaCorner){j.smallest = false;}
                areaTmp.smallest = true;
                smallestDist = areaTmp.Point.z;
                start_location.latitude = i.latitude;
                start_location.longitude = i.longitude;
            }
            areaCorner.push_back(areaTmp);
        }

        double angle = atan((areaCorner[1].Point.x - areaCorner[0].Point.x) / (areaCorner[1].Point.y - areaCorner[0].Point.y));
        
        ROS_INFO_STREAM(areaCorner << "angle: " << angle * 180.0 / M_PI);
        
        std::vector<AreaStruct> rotated;

        for(auto &i : areaCorner){
            AreaStruct areaTmp;
            areaTmp.Point = rotatePoint((i.Point.x - areaCorner[0].Point.x), (i.Point.y - areaCorner[0].Point.y), angle);
            areaTmp.smallest = i.smallest;
            rotated.push_back(areaTmp);
        }
        // ROS_INFO_STREAM(rotated);

        for (uint i = 0; i < rotated.size() - 1;  i++){
            for (uint j = i + 1; j < rotated.size(); j ++){
                if(rotated[i].Point.y > rotated[j].Point.y){
                    std::iter_swap(rotated.begin() + i, rotated.begin() + j);
                }
            }
        }

        rotated[(rotated[0].Point.x > rotated[1].Point.x)].corner = BOTTOM_LEFT;
        rotated[(rotated[0].Point.x < rotated[1].Point.x)].corner = TOP_LEFT;
        rotated[(rotated[2].Point.x > rotated[3].Point.x) + 2].corner = BOTTOM_RIGHT;
        rotated[(rotated[2].Point.x < rotated[3].Point.x) + 2].corner = TOP_RIGHT;

        for (uint i = 0; i < rotated.size() - 1;  i++){
            for (uint j = i + 1; j < rotated.size(); j ++){
                if(rotated[i].corner > rotated[j].corner){
                    std::iter_swap(rotated.begin() + i, rotated.begin() + j);
                }
            }
        }

        //ROS_INFO_STREAM(rotated);

        msg.request.BL = rotated[BOTTOM_LEFT - 1].Point;
        msg.request.BR = rotated[BOTTOM_RIGHT - 1].Point;
        msg.request.TL = rotated[TOP_LEFT - 1].Point;
        msg.request.TR = rotated[TOP_RIGHT - 1].Point;
        for(auto &i : rotated){
            if(i.smallest){
                msg.request.Closest = i.corner;
                break;
            }
        }

        //ROS_INFO_STREAM(msg);

        if(route_planner.call(msg)){
            for(auto &i : msg.response.route){
                route.push_back(rotatePoint(i, -angle));
            }
        }
        else{
            ROS_ERROR_STREAM("Failed to call route planner\n" << msg);
            return;
        }
    }

    ROS_WARN_STREAM("route created:");
    ROS_INFO_STREAM("created route:\n" << route);
    local_status = IDLE;
    got_route = true;
}


sar_drone::rel_coordinates mapAlg::rotatePoint(double x, double y, double angle){
    sar_drone::rel_coordinates tmp;
    tmp.x = cos(angle) * x - sin(angle) * y;
    tmp.y = sin(angle) * x + cos(angle) * y;
    tmp.z = sqrt(pow(x,2) + pow(y, 2));
    return tmp;
}

sar_drone::rel_coordinates mapAlg::rotatePoint(sar_drone::rel_coordinates point, double angle){
    return rotatePoint(point.x, point.y, angle);
}