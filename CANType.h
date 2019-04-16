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
    CAN_RATE_5K = 0,
    CAN_RATE_10K,
    CAN_RATE_20K,
    CAN_RATE_33K,
    CAN_RATE_40K,
    CAN_RATE_47K,
    CAN_RATE_50K,
    CAN_RATE_62K,
    CAN_RATE_80K,
    CAN_RATE_83K,
    CAN_RATE_95K,
    CAN_RATE_100K,
    CAN_RATE_125K,
    CAN_RATE_250K,
    CAN_RATE_400K,
    CAN_RATE_500K,
    CAN_RATE_666K,
    CAN_RATE_800K,
    CAN_RATE_1M
};

/**
 * The error code. Generally, 0 means OK
 */
typedef int CANStatus;

}  // namespace ZCANBus
#endif