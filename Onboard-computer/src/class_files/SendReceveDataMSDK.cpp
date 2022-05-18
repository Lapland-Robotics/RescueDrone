#include <sar_drone/SendReceveDataMSDK.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;

SendReceveDataMSDK::SendReceveDataMSDK(): msg_ID(1)
{
    if(ready){
        ready = false;
        from_mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &SendReceveDataMSDK::fromMobileDataSubscriberCallback, this);
        send_mobile_data_subscriber = nh.subscribe<sar_drone::send_mobile>(SEND_TO_MOBILE, 10, [this](const sar_drone::send_mobile::ConstPtr& msg){
            MobileReceiveCalback = false;
            dataToMobile MobileSend;
            MobileSend.cmdID = msg->cmdID;
            MobileSend.errorCode = msg->errorCode;
            MobileSend.Latitude = msg->coordinate.latitude;
            MobileSend.Longitude = msg->coordinate.longitude;
            // ROS_WARN_STREAM(MobileSend);
            ros::Time start_time = ros::Time::now();
            ros::Duration elapsed_time;
            while(!MobileReceiveCalback){
                elapsed_time = ros::Time::now() - start_time;
                if(elapsed_time > ros::Duration(0.5)){
                    sendToMobile(MobileSend);
                    start_time = ros::Time::now();
                }
                else spin(0.01);
            }
        });

        mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");
        
        drone_PRIO_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_PRIO_TOPPIC, 25);
        map_commands_pub = nh.advertise<sar_drone::directions>(MAP_TOPPIC, 10);

        while(drone_PRIO_commands_pub.getNumSubscribers() == 0 || map_commands_pub.getNumSubscribers() == 0){
            ROS_WARN_STREAM("No subscribers connected");
            ros::Duration(0.5).sleep();
        }
        ready = true;
        gotArea = true;
    }
    std::cout << std::fixed;
    std::cout << std::setprecision(10);
}

SendReceveDataMSDK::~SendReceveDataMSDK()
{

}

bool SendReceveDataMSDK::sendToMobile(dataToMobile returnAckMobile){
    //ROS_INFO_STREAM("data to mobile SDK" << returnAckMobile.cmdID << " " << returnAckMobile.errorCode);
    dji_sdk::SendMobileData mobile_data_tmp;
    mobile_data_tmp.request.data.resize(sizeof(dataToMobile));
    memcpy(&mobile_data_tmp.request.data[0], (uint8_t *)(&returnAckMobile), sizeof(dataToMobile));
    mobile_data_service.call(mobile_data_tmp);
    return mobile_data_tmp.response.result;
}

void SendReceveDataMSDK::fromMobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data){
    dji_sdk::MobileData mobile_data = *from_mobile_data;

    // ROS_INFO_STREAM("got " << mobile_data << " from mobile device");

    dataFromMobile MobileReseive;

    memcpy(&MobileReseive, &mobile_data.data[0], sizeof(dataFromMobile));

    // ROS_INFO_STREAM("got " <<(double) MobileReseive.Latitude << " from mobile device");

    msgCommands cmdID = static_cast<msgCommands>(MobileReseive.cmdID);
    switch (cmdID)
    {
        case EMERGENCY_SHUTDOWN: {
            ROS_INFO_STREAM("Got Emergency shutdown from MSDK");
            break;
        }

        case EMERGENCY_LAND:{
            ROS_INFO_STREAM("Got land from MSDK");

            sar_drone::directions msg;
            msg.ID = msg_ID;
            msg.Command = EMERGENCY_LAND;
            drone_PRIO_commands_pub.publish(msg);
            map_commands_pub.publish(msg);

            break;
        }

        case MANUAL_OVERRIDE:{
            ROS_INFO_STREAM("got manual override from MSDK");

            sar_drone::directions msg;
            msg.ID = msg_ID;
            msg.Command = MANUAL_OVERRIDE;
            drone_PRIO_commands_pub.publish(msg);

            break;
        }

        case START_SEARCH:{
            ROS_INFO_STREAM("Got start search from MSDK");

            sar_drone::directions msg;
            msg.ID = msg_ID;
            msg.Command = START_SEARCH;
            map_commands_pub.publish(msg);

            break;
        }

        case STOP_SEARCH:{
            ROS_INFO_STREAM("Got stop search from MSDK");

            sar_drone::directions msg;
            msg.ID = msg_ID;
            msg.Command = STOP_SEARCH;
            map_commands_pub.publish(msg);

            break;
        }

        case AREA_COORDINATES:{
            ROS_INFO_STREAM("Got area coordinates from MSDK");

            ROS_WARN_STREAM("Latitude: " << std::fixed << std::setprecision(10) << MobileReseive.Latitude << "\tLongitude: " << MobileReseive.Longitude);
            
            if(gotArea){
                area.clear();
                gotArea = false;
            }
            sar_drone::coordinates tmp;
            tmp.latitude = MobileReseive.Latitude;
            tmp.longitude = MobileReseive.Longitude;
            area.emplace_back(tmp);
            
            dataToMobile MobileSend;
            MobileSend.cmdID = AREA_COORDINATES;
            MobileSend.errorCode = NO_ERROR;
            MobileSend.Latitude = area.back().latitude;
            MobileSend.Longitude = area.back().longitude;
            sendToMobile(MobileSend);
            break;
        }

        case AREA_FINISHED:{
            ROS_WARN_STREAM("Got all area coordinates from MSDK");
            sar_drone::directions msg;
            msg.ID = msg_ID;
            msg.Command = AREA_COORDINATES;
            msg.coordinate = area;
            map_commands_pub.publish(msg);
            ROS_WARN_STREAM("this is the area with size: " << area.size());
            for(auto &i : area){
                ROS_WARN_STREAM(i);
            }
            gotArea = true;
        }

        case ARE_YOU_ALIVE:{
            ROS_INFO_STREAM("Got alive request from MSDK");
            dataToMobile MobileSend;
            MobileSend.cmdID = ARE_YOU_ALIVE;
            MobileSend.errorCode = ready ? NO_ERROR : NOT_READY_YET;
            sendToMobile(MobileSend);
            break;
        }

        case MOBILE_RECEIVE_CALLBACK:{
            MobileReceiveCalback = true;
        }

        default:{
            ROS_INFO_STREAM("Does not know this command (" <<(int) cmdID << ")");
            break;
        }
    }
}