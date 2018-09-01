#pragma once
#ifndef __CANBase_H
#define __CANBase_H
#include <functional>
#include <string>
#include "CANType.h"
namespace ZCANBus {

class CANBase {
   public:
    CANBase(){};

    virtual ~CANBase(){};

    /**
     * @brief Open a specific channel
     * @param channel the specific number for the CAN device
     * @param baudRate the speed for the communication
     * @param type different CAN devices has differents meanings
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    virtual CANStatus OpenChannel(int channel, CANRate baudRate, int type) = 0;

    /**
     * @brief Read CAN message continuously with async mode.
     * @param callback the function will be called while received new CAN
     * message the param msg is the received msg and status is the error code
     * which generally 0 means OK
     * @param interval the max interval in milliseconds between two message
     * received
     */
    virtual void ReadLoop(
        std::function<void(const CANMessage* msg, CANStatus status)> callback,
        uint64_t interval) = 0;

    /**@brief End read CAN message continuously with async mode.*/
    virtual void EndReadLoop() = 0;

    /**
     * @brief Read CAN message once
     * @param msg Modified by received CAN message
     * @param timeout Read CAN message with timeout in milliseconds
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    virtual CANStatus ReadOnce(CANMessage& msg, uint64_t timeout = 0) = 0;

    /**
     * @brief Write CAN message once
     * @param msg the CAN message to be wrote
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    virtual CANStatus Write(CANMessage* msg, int count) = 0;

    /**
     * @brief Close the channel.
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    virtual CANStatus CloseChannel() = 0;

    /**
     * @brief Flush the data in queue.
     * @return a error code. Generally, 0 means OK. @see CanStatus
     */
    virtual CANStatus FlushQueue() = 0;

    /**
     * @brief Get error message in detail.
     * @param status the error code. Modified by the new error code returned by
     * this operation
     * @return error message
     */
    virtual std::string GetErrorText(CANStatus& status) = 0;
};
}  // namespace ZCANBus
#endif