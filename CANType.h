#ifndef __CAN_TYPE_H
#define __CAN_TYPE_H
namespace ZCANBus {
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
    CAN_RATE_47K,
    CAN_RATE_50K,
    CAN_RATE_62K,
    CAN_RATE_83K,
    CAN_RATE_95K,
    CAN_RATE_100K,
    CAN_RATE_125K,
    CAN_RATE_250K,
    CAN_RATE_500K,
    CAN_RATE_800K,
    CAN_RATE_1M
};

/**
 * The error code. Generally, 0 means OK
 */
typedef int CANStatus;

}  // namespace ZCANBus
#endif