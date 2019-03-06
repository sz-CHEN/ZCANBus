#pragma once
#ifndef __CANBase_Peak_H
#define __CANBase_Peak_H
#include "CANBase.h"
#ifdef __APPLE__
#include <PCBUSB.h>
#else
#include <PCANBasic.h>
#endif
#include <thread>
namespace ZCANBus {
class CANPeak : public CANBase {
   private:
    std::thread* th;
    int channel;
    bool loopOn;

   public:
    CANPeak();
    ~CANPeak();
    /**
     * @param channel
     * @param type Indicates HwType
     * @see <PCANBasic.h> CAN_Initialize
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int type = 0) override;

    /**
     * @param channel
     * @param argv Indicates HwType, IOPort, Interrupt in CAN_Initialize.
     * Default 0.
     * @see <PCANBasic.h> CAN_Initialize
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int argc,
                          char* argv[]) override;
    void ReadLoop(
        std::function<void(const CANMessage* msg, CANStatus status)> callback,
        uint64_t interval) override;
    void EndReadLoop() override;
    CANStatus ReadOnce(CANMessage& msg, uint64_t timeout) override;
    CANStatus Write(CANMessage* msg, int count) override;
    CANStatus CloseChannel() override;
    CANStatus FlushQueue() override;
    std::string GetErrorText(CANStatus& status) override;
};
}  // namespace ZCANBus
#endif