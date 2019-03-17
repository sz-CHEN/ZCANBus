#pragma once
#ifndef __CANBase_Kvaser_H
#define __CANBase_Kvaser_H
#include <canlib.h>
#include <thread>
#include "CANBase.h"
namespace ZCANBus {
class CANKvaser : public CANBase {
   private:
    std::thread* th;
    int handle;
    bool loopOn;

   public:
    CANKvaser();
    ~CANKvaser();
    /**
     * @param channel @see <canlib.h> canOpenChannel
     * @param type Indicates flags. @see <canlib.h> canOPEN_XXX
     */
    CANStatus OpenChannel(int channel, CANRate baudRate,
                          int type = canOPEN_ACCEPT_VIRTUAL) override;

    /**
     * @param channel Indicates @param channel in canOpenChannel
     * @param argv Indicates @param flags in canOpenChannel and  tseg1, tseg2,
     * sjw, noSamp, syncmode in canSetBusParams. Default 0.
     * @see <canlib.h>
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int argc,
                          char* argv[]) override;
    void ReadLoop(
        std::function<void(const CANMessage* msg, CANStatus status)> callback,
        uint64_t interval) override;
    void EndReadLoop() override;
    CANStatus ReadOnce(CANMessage& msg, uint64_t timeout) override;
    CANStatus Write(const CANMessage& msg) override;
    CANStatus Write(CANMessage* msg, int count) override;
    CANStatus CloseChannel() override;
    CANStatus FlushQueue() override;
    std::string GetErrorText(CANStatus& status) override;
};
}  // namespace ZCANBus
#endif