#include <sar_drone/mapAlg.h>

using namespace SaR_Drone;

mapAlg::mapAlg(): msg_ID(2)
{
    status_drone = ON_GROUND;
    
    local_status = IDLE;
    drone_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_TOPPIC, 10);
    drone_PRIO_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_PRIO_TOPPIC, 10);
    send_mobile_data_pub = nh.advertise<sar_drone::send_mobile>(SEND_TO_MOBILE, 10);
    
    map_commands_sub = nh.subscribe<sar_drone::directions>(MAP_TOPPIC, 10, &mapAlg::mapCommandsCalback, this);
    drone_status_sub = nh.subscribe<sar_drone::status>(STATUS_TOPPIC, 10, [this](const sar_drone::status::ConstPtr& msg){
        status_drone = static_cast<statusCodes>(msg->Status);
    });
    
    route_index = 0;
    demo = true;
    got_route = false;
}

mapAlg::~mapAlg()
{

}

void mapAlg::step(double sleepTime){
    ros::spinOnce();
    switch (local_status){
        case OFF:{
            break;
        }

        case IDLE:{
            //waiting for start coordinates
            break;
        }

        case CREATE_ROUTE:{
            ROS_INFO_STREAM("Create route");
            route.clear();
            route_index = 0;
            if(demo){
                route.emplace_back(0,0);
                route.emplace_back(10,10);
                route.emplace_back(-10,10);
                route.emplace_back(10,-10);
                route.emplace_back(-10,-10);
            }
            else{
                //create optimal route algoritm
            }

            
            ROS_INFO_STREAM("route created");
            local_status = IDLE;
            got_route = true;
            break;
        }

        case GOING_TO_START:{
            ROS_INFO_STREAM("Go to start");
            
            sar_drone::directions return_msg;
            return_msg.ID = msg_ID;
            return_msg.Command = MA_MOVE_COORDINATES;
            return_msg.Latitude = start_location.first;
            return_msg.Longitude = start_location.second;
            return_msg.z = 3;
            drone_commands_pub.publish(return_msg);
            local_status = WAIT_MOVING;
            break;
        }

        case NEXT_MOVE:{
            // ROS_INFO_STREAM("Next Move");

            sar_drone::directions return_msg;
            return_msg.ID = msg_ID;
            return_msg.Command = MA_MOVE_RELATIVE_GROUND;

            return_msg.x = route.at(route_index).first;
            return_msg.y = route.at(route_index).second;
            return_msg.z = (route.at(route_index).first == 0 && route.at(route_index).second == 0) ? 3 : 0;
            return_msg.r = 0;

            drone_commands_pub.publish(return_msg);

            route_index ++;

            //ROS_INFO_STREAM("route index: " <<(int) route_index << "\troute size: " <<(int) route.size());

            next_local_status = route_index == route.size() ? STOPPING : WAIT_MOVING;
            local_status = MOVE_COMMAND_SEND;
            
            break;

        }
        
        case MOVE_COMMAND_SEND:{
            if(status_drone != MAPPING_ALGORITM_NEXT_STEP){
                local_status = next_local_status;
                start_time = ros::Time::now();
            }
            break;
        }

        case WAIT_MOVING:{
            //waiting for drone stoped moving
            switch(status_drone){
                case MAPPING_ALGORITM_NEXT_STEP:
                    ROS_INFO_STREAM("next move");
                    local_status = NEXT_MOVE;
                    break;
                
                case TAKING_OFF:
                case LANDING:
                case MAPPING_ALGORITM_MOVING:
                    break;

                case ON_GROUND:
                    local_status = OFF;
                    break;
                
                case MAPPING_ALGORITM_DIDNT_FINISH_MOVE:
                    local_status = STOPPING;
                    break;

                default:
                    ROS_ERROR_STREAM("serious what are you doing? I don't now this command (" << (int) status_drone << ")");
                    break;
            }
            
            break;
        }

        case WAIT_STARTING:{
            //waiting for drone stoped moving
            switch(status_drone){
                case MAPPING_ALGORITM_NEXT_STEP:
                    ROS_INFO_STREAM("first move");
                    local_status = NEXT_MOVE;
                    break;
                
                case TAKING_OFF:
                case ON_GROUND:
                case MAPPING_ALGORITM_MOVING:
                    break;

                default:
                    ROS_ERROR_STREAM("serious what are you doing? I don't now this command (" << (int) status_drone << ")");
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

        default:{
            ROS_ERROR_STREAM("WHAT THE HELLL DIT YOU JUST DO?");
        }
    }
    ros::Duration(sleepTime).sleep();
}

void mapAlg::mapCommandsCalback(const sar_drone::directions::ConstPtr& msg){
    switch (static_cast<msgCommands>(msg->Command))
    {
        case START_SEARCH:{
            ROS_INFO_STREAM("start searching");

            local_status = WAIT_STARTING;

            if(got_route){
                
                route_index = 0;
                sar_drone::directions return_msg;
                return_msg.ID = msg_ID;
                return_msg.Command = TAKE_OFF;
                drone_PRIO_commands_pub.publish(return_msg);
            }
            else{
                ROS_WARN_STREAM("don't have instructions yet");
                sar_drone::send_mobile toMobile;
                toMobile.cmdID = START_SEARCH;
                toMobile.errorCode = MobileErrorCodes::NO_INSTRUCTIONS_RECEIVED;
                send_mobile_data_pub.publish(toMobile);
            }

            break;
        }

        case STOP_SEARCH:{
            local_status = STOPPING;
            break;
        }

        case START_COORDINATES:{
            start_location.first = msg->Latitude;
            start_location.first = msg->Longitude;
            local_status = demo ? CREATE_ROUTE : GOING_TO_START;
        }

        case EMERGENCY_SHUTDOWN:
        case EMERGENCY_LAND:
            break;

        default:
            break;
    }
}