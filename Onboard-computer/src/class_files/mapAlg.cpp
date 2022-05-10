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
<<<<<<< HEAD
=======
        ROS_WARN_STREAM("status: " << status_drone);
>>>>>>> Combining human detection and onboard computer #55 & #56
    });
    
    route_index = 0;
    demo = true;
    got_route = false;
<<<<<<< HEAD
=======
    on_site = false;
>>>>>>> Combining human detection and onboard computer #55 & #56
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
                ROS_INFO_STREAM("Go to start");
                
                sar_drone::directions return_msg;
                return_msg.ID = msg_ID;
                return_msg.Command = MA_MOVE_COORDINATES;
                return_msg.Latitude = start_location.first;
                return_msg.Longitude = start_location.second;
                return_msg.z = 3;
                drone_commands_pub.publish(return_msg);
<<<<<<< HEAD
                local_status = WAIT_MOVING;
                break;
            }

            case NEXT_MOVE:{
                // ROS_INFO_STREAM("Next Move");

=======

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
>>>>>>> Combining human detection and onboard computer #55 & #56
                sar_drone::directions return_msg;
                return_msg.ID = msg_ID;
                return_msg.Command = MA_MOVE_RELATIVE_GROUND;

                return_msg.x = route.at(route_index).first;
                return_msg.y = route.at(route_index).second;
<<<<<<< HEAD
                return_msg.z = (route.at(route_index).first == 0 && route.at(route_index).second == 0) ? 3 : 0;
                return_msg.r = 0;

                drone_commands_pub.publish(return_msg);

=======
                return_msg.z = (route.at(route_index).first == 0 && route.at(route_index).second == 0) ? 5 : 0;
                return_msg.r = 0;

                drone_commands_pub.publish(return_msg);
                
                ROS_INFO_STREAM("Next Move send");
>>>>>>> Combining human detection and onboard computer #55 & #56
                route_index ++;

                //ROS_INFO_STREAM("route index: " <<(int) route_index << "\troute size: " <<(int) route.size());

                next_local_status = route_index == route.size() ? STOPPING : WAIT_MOVING;
                local_status = MOVE_COMMAND_SEND;
                
                break;

            }
<<<<<<< HEAD
            
            case MOVE_COMMAND_SEND:{
                if(status_drone != MAPPING_ALGORITM_NEXT_STEP){
=======

            case START_HUMAN_SEND:{
                if(status_drone != MAPPING_ALGORITM_NEXT_STEP){
                    local_status = next_local_status;
                    start_time = ros::Time::now();
                }
                break;
            }
            
            case MOVE_COMMAND_SEND:{
                if(status_drone != MAPPING_ALGORITM_NEXT_STEP && status_drone != START_HUMAN_DETECTION){
>>>>>>> Combining human detection and onboard computer #55 & #56
                    local_status = next_local_status;
                    start_time = ros::Time::now();
                }
                break;
            }

            case WAIT_MOVING:{
                //waiting for drone stoped moving
                switch(status_drone){
<<<<<<< HEAD
=======
                    case START_HUMAN_DETECTION:
>>>>>>> Combining human detection and onboard computer #55 & #56
                    case MAPPING_ALGORITM_NEXT_STEP:
                        ROS_INFO_STREAM("next move");
                        local_status = NEXT_MOVE;
                        break;
                    
                    case TAKING_OFF:
                    case LANDING:
                    case MAPPING_ALGORITM_MOVING:
<<<<<<< HEAD
=======
                    case HUMAN_DETECTION_NEXT_STEP:
                    case HUMAN_DETECTION_MOVING:
>>>>>>> Combining human detection and onboard computer #55 & #56
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
<<<<<<< HEAD
                        ROS_INFO_STREAM("first move");
                        local_status = NEXT_MOVE;
=======
                        ROS_INFO_STREAM("go to start");

                        local_status = on_site ? START_HUMAN : GOING_TO_START;
>>>>>>> Combining human detection and onboard computer #55 & #56
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
            local_status = STOP_NOW;
            break;
        }

        case START_COORDINATES:{
            start_location.first = msg->Latitude;
            start_location.first = msg->Longitude;
            CreateRoute();
        }

        case EMERGENCY_SHUTDOWN:
        case EMERGENCY_LAND:
            break;

        default:
            break;
    }
}

void mapAlg::CreateRoute(){
    ROS_INFO_STREAM("Create route");
    route.clear();
    route_index = 0;
    if(demo){
        route.emplace_back(0,0);
<<<<<<< HEAD
        route.emplace_back(10,10);
        route.emplace_back(-10,10);
        route.emplace_back(10,-10);
        route.emplace_back(-10,-10);
=======
        route.emplace_back(10,0);
        route.emplace_back(0,5);
        route.emplace_back(-10,0);
        route.emplace_back(0,5);
        route.emplace_back(10,0);
        route.emplace_back(0,5);
        route.emplace_back(-10,0);
        route.emplace_back(0,5);
        route.emplace_back(0,-20);
        
        on_site = true;
>>>>>>> Combining human detection and onboard computer #55 & #56
    }
    else{
        //create optimal route algoritm
    }

    
    ROS_INFO_STREAM("route created");
    local_status = IDLE;
    got_route = true;
}