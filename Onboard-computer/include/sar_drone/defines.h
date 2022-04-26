#ifndef DEFINES_H
#define DEFINES_H

#define DIRECTIONS_TOPPIC "directions"
#define MAP_TOPPIC "mapAlg"
#define DIRECTIONS_PRIO_TOPPIC "directionsPRIO"
#define STATUS_TOPPIC "status"
#define SEND_TO_MOBILE "mobile_messages"

#define DISTANCE_SLOWER 50.0
#define SPEED_MAX 30.0
#define SPEED_FAST 5.0
#define SPEED_MIDDLE 2.0
#define SPEED_SLOW 1.0

#define PI 3.14159265

namespace SaR_Drone{
    enum msgCommands{
        //PRIO Commands
        EMERGENCY_SHUTDOWN = 0,
        EMERGENCY_LAND = 1,
        MANUAL_OVERRIDE = 2,
        
        //Primary function commands
        TAKE_OFF = 10,
        LAND = 11,

        //MSDK -> mapping alg commands
        START_SEARCH = 20,
        STOP_SEARCH = 21,
        START_COORDINATES = 22,
        AREA_COORDINATES = 23,
        AREA_FINISHED = 24,

        DEMO = 30,

        //mapping alg -> controll drone commands
        
        MA_MOVE_COORDINATES = 45,
        MA_MOVE_RELATIVE_GROUND_HEADLESS = 46,
        MA_MOVE_RELATIVE_GROUND = 47,
        MA_MOVE_RELATIVE_BODY = 48,

        //human detection commands
        HD_FOUND_PERSON_I_THINK = 110,
        HD_I_WAS_WRONG = 111,
        HD_FOUND_PERSON_IM_SERTAIN = 112,
        HD_FORWARD = 113,
        HD_BACKWARD = 114,
        HD_LEFT = 115,
        HD_RIGHT = 116,
        HD_TURN_LEFT = 117,
        HD_TURN_RIGHT = 118,
        HD_UP = 119,
        HD_DOWN = 120
    };

    enum statusCodes{
        ON_GROUND = 0,
        TAKING_OFF = 1,
        MAPPING_ALGORITM_NEXT_STEP = 3,
        MAPPING_ALGORITM_MOVING = 4,
        HUMAN_DETECTION_NEXT_STEP = 5,
        HUMAN_DETECTION_MOVING = 6,
        FOUND_HUMAN = 7,
        BACK_TO_BASE = 8,
        LANDING = 8,

        MAPPING_ALGORITM_DIDNT_FINISH_MOVE = 40,

        MANUAL_CONTROLL = 112,


        STARTING_UP = 255
    };

    enum MobileErrorCodes{
        NO_ERROR = 0,

        ERROR_WITH_TAKEOFF = 10,
        ERROR_WITH_LANDING = 11,

        NO_INSTRUCTIONS_RECEIVED = 20
    };
}

#endif //DEFINES_H