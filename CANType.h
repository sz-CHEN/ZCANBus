#ifndef __CAN_TYPE_H
#define __CAN_TYPE_H
#include <cstdint>
namespace ZCANBus {

enum class CANMSGType : unsigned int {
    STANDARD = 0,
    RTR = 1,
    EXTENDED = 1 << 1,
    FD = 1 << 2,
    BRS = 1 << 3,
    ESI = 1 << 4,
    ERRFRAME = 1 << 6,
    UNKNOWN = 1 << 7
};

/**
 * The struct contains CAN message, CAN ID, message length, message type and
 * timestamp
 */
typedef struct {
    unsigned long timestamp;
    long id;
    unsigned int length;
    unsigned int type;
    unsigned char msg[8];
} CANMessage;

/**
 * The enum for CANRate which means the speed for the communication.
 * Every device should translate it to its definition while configure the CAN
 * channel.
 * */
enum class CANRate : int {
    CAN_RATE_5K = 5,
    CAN_RATE_10K = 10,
    CAN_RATE_20K = 20,
    CAN_RATE_33K = 33,
    CAN_RATE_40K = 40,
    CAN_RATE_47K = 47,
    CAN_RATE_50K = 50,
    CAN_RATE_62K = 62,
    CAN_RATE_80K = 80,
    CAN_RATE_83K = 83,
    CAN_RATE_95K = 95,
    CAN_RATE_100K = 100,
    CAN_RATE_125K = 125,
    CAN_RATE_250K = 250,
    CAN_RATE_400K = 400,
    CAN_RATE_500K = 500,
    CAN_RATE_666K = 666,
    CAN_RATE_800K = 800,
    CAN_RATE_1M = 1000
};

/**
 * The error code. Generally, 0 means OK
 */
typedef int CANStatus;

}  // namespace ZCANBus
#endif