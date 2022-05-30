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
        demo = true;
        got_route = false;
        on_site = false; 
    }
    else{
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

                home_base = getGPS();

                ROS_INFO_STREAM("Go to start");
                
                sar_drone::directions return_msg;
                sar_drone::coordinates coord_msg;

                coord_msg.latitude = start_location.latitude;
                coord_msg.longitude = start_location.longitude;
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

                return_msg.position.x = route.at(route_index).first;
                return_msg.position.y = route.at(route_index).second;
                return_msg.position.z = (route.at(route_index).first == 0 && route.at(route_index).second == 0) ? 5 : 0;
                return_msg.position.r = 0;

                drone_commands_pub.publish(return_msg);
                
                ROS_INFO_STREAM("Next Move send");
                route_index ++;

                //ROS_INFO_STREAM("route index: " <<(int) route_index << "\troute size: " <<(int) route.size());

                next_local_status = route_index == route.size() ? STOPPING : WAIT_MOVING;
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

                        local_status = on_site ? START_HUMAN : GOING_TO_START;
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

            case STOPPING:{
                switch(status_drone){
                    case ON_GROUND:
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
                            
                            local_status = MOVE_COMMAND_SEND;
                            next_local_status = WAIT_MOVING;
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
                        
                        local_status = MOVE_COMMAND_SEND;
                        next_local_status = WAIT_MOVING;
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

            // ROS_WARN_STREAM("don't have instructions yet");
            // sar_drone::send_mobile toMobile;
            // toMobile.cmdID = START_SEARCH;
            // toMobile.errorCode = MobileErrorCodes::NO_INSTRUCTIONS_RECEIVED;
            // send_mobile_data_pub.publish(toMobile);
            break;
        }

        case STOP_SEARCH:{
            local_status = STOP_NOW;
            break;
        }

        case AREA_COORDINATES:{
        //     start_location.first = msg->coordinate.front.;
        //     start_location.first = msg->coordinate.front().longitude;
            
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
    route.clear();
    route_index = 0;
    if(demo){
        route.emplace_back(0,0);
        route.emplace_back(10,0);
        route.emplace_back(0,5);
        route.emplace_back(-10,0);
        route.emplace_back(0,5);
        route.emplace_back(10,0);
        route.emplace_back(0,5);
        route.emplace_back(-10,0);
        route.emplace_back(0,5);
        route.emplace_back(10,0);
        route.emplace_back(-10,-20);

        // route.emplace_back(0,0);
        // route.emplace_back(10, 10);
        // route.emplace_back(10, -10);
        // route.emplace_back(-10, -10);
        // route.emplace_back(-10, 10);
        
        on_site = true;
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
            areaTmp.Point = translateGPS(ready ? getGPS() : fake_origin, gpsTmp, false, true);
            if(smallestDist == -1.20301 || smallestDist > areaTmp.Point.z){
                for(auto &j : areaCorner){j.smallest = false;}
                areaTmp.smallest = true;
                smallestDist = areaTmp.Point.z;
            }
            areaCorner.push_back(areaTmp);
        }

        double angle = atan((areaCorner[1].Point.x - areaCorner[0].Point.x) / (areaCorner[1].Point.y - areaCorner[0].Point.y));
        
        ROS_INFO_STREAM(areaCorner << "\n angle: " << angle * 180.0 / M_PI);
        
        std::vector<AreaStruct> rotated;

        for(auto &i : areaCorner){
            AreaStruct areaTmp;
            areaTmp.Point = rotatePoint((i.Point.x - areaCorner[0].Point.x), (i.Point.y - areaCorner[0].Point.y), angle);
            areaTmp.smallest = i.smallest;
            rotated.push_back(areaTmp);
        }
        ROS_INFO_STREAM(rotated);

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

        ROS_INFO_STREAM(rotated);

        // std::vector<AreaStruct> back;

        // for(auto &i : rotated){
        //     AreaStruct areaTmp;
        //     areaTmp.Point = rotatePoint((i.Point.x), (i.Point.y), -angle);
        //     back.push_back(areaTmp);
        // }
        
        // ROS_INFO_STREAM(back);

        // for (uint i = 0; i < areaCorner.size() - 1;  i++){
        //     for (uint j = i + 1; j < areaCorner.size(); j ++){
        //         if(areaCorner[i].Point.x > areaCorner[j].Point.x){
        //             std::iter_swap(areaCorner.begin() + i, areaCorner.begin() + j);
        //         }
        //     }
        // }

        // if(areaCorner[0].Point.y < areaCorner[1].Point.y){
        //     areaCorner[0].corner = 
        // }

        //create optimal route algoritm
    }

    
    ROS_INFO_STREAM("route created");
    local_status = IDLE;
    got_route = true;


}


geometry_msgs::Point mapAlg::rotatePoint(double x, double y, double angle){
    geometry_msgs::Point tmp;
    tmp.x = cos(angle) * x - sin(angle) * y;
    tmp.y = sin(angle) * x + cos(angle) * y;
    tmp.z = sqrt(pow(x,2) + pow(y, 2));
    return tmp;
}