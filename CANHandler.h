//
// Created by ShizengChen on 2017/12/24.
//

#ifndef __CANBase_Handler_H
#define __CANBase_Handler_H

#include <functional>
#include "CANType.h"
#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

namespace ZCANBus {
/**
 * @see <PCANBasic.h>
 * @see <canlib.h>
 */
enum class CANType { PEAK_CAN, KVASER_CAN, ZLG_CAN, ZLG2_CAN };
class CANBase;
class DLLEXPORT CANHandler {
   private:
    CANBase* baseCan;

   public:
    CANHandler(CANType canType);
    ~CANHandler();

    /**
     * @brief Open a specific channel
     * @param channel the specific number for the CAN device
     * @param baudRate the speed for the communication
     * @param type different CAN devices has differents meanings
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int type);

    /**
     * @brief Open a specific channel
     * @param channel the specific number for the CAN device
     * @param baudRate the speed for the communication
     * @param argc size of argv
     * @param argv the specific params for specific device
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int argc,
                          void* argv[]);

    /**
     * @brief Read CAN message continuously with async mode.
     * @param callback the function will be called while received new CAN
     * message the param msg is the received msg and status is the error code
     * which generally 0 means OK
     * @param interval the max interval in milliseconds between two message
     * received
     */
    void ReadLoop(
        std::function<void(const CANMessage* msg, CANStatus status)> callback,
        uint64_t interval);

    /**@brief End read CAN message continuously with async mode.*/
    void EndReadLoop();

    /**
     * @brief Read CAN message once
     * @param msg Modified by received CAN message
     * @param timeout Read CAN message with timeout in milliseconds
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus ReadOnce(CANMessage& msg, uint64_t timeout = 0);

    /**
     * @brief Write CAN message once
     * @param msg the CAN message to be wrote
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus Write(const CANMessage& msg);

    /**
     * @brief Write CAN message once
     * @param msg the pointer of the CAN messages array to be wrote
     * @param count the msg count
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus Write(CANMessage* msg, int count);

    /**
     * @brief Close the channel.
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus CloseChannel();

    /**
     * @brief Flush the data in queue.
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    CANStatus FlushQueue();

    /**
     * @brief Get error message in detail.
     * @param status the error code. Modified by the new error code returned by
     * this operation
     * @return error message
     */
    std::string GetErrorText(CANStatus& status);
};
}  // namespace ZCANBus
#endif  //__CANBase_Handler_H
