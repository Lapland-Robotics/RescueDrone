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

#include <vector>
#include <iostream>

namespace SaR_Drone{

    class SendReceveDataMSDK: public ParrentDroneClass
    {
    public:

        SendReceveDataMSDK();
        ~SendReceveDataMSDK();
#pragma pack(1)
        struct dataFromMobile{
            uint8_t cmdID;
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
        bool gotArea;
        bool MobileReceiveCalback;
        const uint8_t msg_ID; 

        std::vector<sar_drone::coordinates> area; //first = lat, second = long

        ros::Subscriber from_mobile_data_subscriber;
        ros::Subscriber send_mobile_data_subscriber;

        ros::ServiceClient mobile_data_service;
        
        ros::Publisher drone_PRIO_commands_pub;
        ros::Publisher map_commands_pub;
        
    };

    std::ostream& operator<<(std::ostream& os, const SendReceveDataMSDK::dataToMobile& data)
    {
        return os << "ID: " << (int)data.cmdID << " \terror: " << (int)data.errorCode << "\nlat: " << std::setprecision(10) << data.Latitude << "\tlong: " << data.Longitude << "\n";
    }
}

#endif