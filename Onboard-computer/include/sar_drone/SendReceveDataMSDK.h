#ifndef SENDRECEVEDATAMSDK_H
#define SENDRECEVEDATAMSDK_H

//parrent class
#include <sar_drone/ParrentDroneClass.h>

//class specific includes
#include <dji_sdk/MobileData.h>
#include <dji_sdk/SendMobileData.h>

//system includes
#include <sar_drone/directions.h>
#include <sar_drone/send_mobile.h>

namespace SaR_Drone{
    class SendReceveDataMSDK: public ParrentDroneClass
    {
    public:

        SendReceveDataMSDK();
        ~SendReceveDataMSDK();
#pragma pack(1)
        struct dataFromMobile{
            uint8_t cmdID;
            uint8_t index;
            float64_t Latitude;
            float64_t Longitude;
        };
#pragma pack()

#pragma pack(1)
        struct dataToMobile{
            uint8_t cmdID;
            uint8_t errorCode;
            float64_t Latitude;
            float64_t Longitude;
        };
#pragma pack()

    bool sendToMobile(dataToMobile returnAckMobile);
    void fromMobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data);

    private:
        const uint8_t msg_ID; 

        ros::Subscriber from_mobile_data_subscriber;
        ros::Subscriber send_mobile_data_subscriber;

        ros::ServiceClient mobile_data_service;
        
        ros::Publisher drone_PRIO_commands_pub;
        ros::Publisher map_commands_pub;
    };
}

#endif