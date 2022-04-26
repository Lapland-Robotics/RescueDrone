#include <sar_drone/SendReceveDataMSDK.h>

using namespace SaR_Drone;
using namespace DJI::OSDK;

SendReceveDataMSDK::SendReceveDataMSDK(): msg_ID(1)
{
    if(ready){
        from_mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &SendReceveDataMSDK::fromMobileDataSubscriberCallback, this);
        send_mobile_data_subscriber = nh.subscribe<sar_drone::send_mobile>(SEND_TO_MOBILE, 10, [this](const sar_drone::send_mobile::ConstPtr& msg){
            dataToMobile MobileSend;
            MobileSend.cmdID = msg->cmdID;
            MobileSend.errorCode = msg->errorCode;
            MobileSend.Latitude = msg->Latitude;
            MobileSend.Longitude = msg->Longitude;
            sendToMobile(MobileSend);
        });

        mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");
        
        drone_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_TOPPIC, 25);
        drone_PRIO_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_PRIO_TOPPIC, 25);
        map_commands_pub = nh.advertise<sar_drone::directions>(MAP_TOPPIC, 10);
        
        sar_drone::directions msg;
        msg.ID = 1;
        msg.Command = 255;

        while(drone_commands_pub.getNumSubscribers() == 0 || drone_PRIO_commands_pub.getNumSubscribers() == 0){
            ROS_WARN_STREAM("No subscribers connected");
            ros::Duration(0.5).sleep();
        }
        
        drone_commands_pub.publish(msg);
    }
    else{ //debugging and testing

        ROS_WARN_STREAM("entering debug and testing mode. Not connected to drone");
        drone_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_TOPPIC, 25);
        drone_PRIO_commands_pub = nh.advertise<sar_drone::directions>(DIRECTIONS_PRIO_TOPPIC, 25);
        map_commands_pub = nh.advertise<sar_drone::directions>(MAP_TOPPIC, 10);
        
        sar_drone::directions msg;

        while(drone_commands_pub.getNumSubscribers() == 0 || drone_PRIO_commands_pub.getNumSubscribers() == 0){
            ROS_WARN_STREAM("No subscribers connected");
            ros::Duration(0.5).sleep();
        }

        msg.Command = 10;
        drone_PRIO_commands_pub.publish(msg);
        ros::Duration(0.5).sleep();

        msg.ID = 0;
        msg.Command = 20;
        for(char i = 0; i < 20; i++){
            msg.ID += 1;
            drone_commands_pub.publish(msg);
        }
        ros::Duration(0.5).sleep();

        msg.Command = 11;
        drone_PRIO_commands_pub.publish(msg);
            
        msg.Command = 20;
        for(char i = 0; i < 20; i++){
            msg.ID += 1;
            drone_commands_pub.publish(msg);
        }

        msg.Command = 10;
        drone_PRIO_commands_pub.publish(msg);

        ros::Duration(1).sleep();

        msg.Command = 20;
        for(char i = 0; i < 20; i++){
            msg.ID += 1;
            drone_commands_pub.publish(msg);
        }

        ROS_INFO_STREAM("data send succesfuly");
    }
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

        case START_COORDINATES:{
            ROS_INFO_STREAM("Got start coordinates from MSDK");

            ROS_INFO_STREAM("Latitude: " << MobileReseive.Latitude << "\tLongitude: " << MobileReseive.Longitude);

            sar_drone::directions msg;
            msg.ID = msg_ID;
            msg.Command = START_COORDINATES;
            msg.Latitude = MobileReseive.Latitude;
            msg.Longitude = MobileReseive.Longitude;
            map_commands_pub.publish(msg);
            break;
        }

        default:{
            ROS_INFO_STREAM("Does not know this command (" <<(int) cmdID << ")");
            break;
        }
    }
}