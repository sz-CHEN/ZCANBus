#pragma once
#ifndef __CANBase_ZLG2_H
#define __CANBase_ZLG2_H
#include "CANBase.h"
#include <thread>
#include <zlgcan/zlgcan.h>
namespace ZCANBus {
class CANZLG2 : public CANBase {
   private:
    std::thread* th;
    bool loopOn;
    DEVICE_HANDLE dhandle;
    CHANNEL_HANDLE chandle;

   public:
    CANZLG2();
    ~CANZLG2();
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
    CANStatus Write(const CANMessage& msg) override;
    CANStatus Write(CANMessage* msg, int count) override;
    CANStatus CloseChannel() override;
    CANStatus FlushQueue() override;
    std::string GetErrorText(CANStatus& status) override;
};
}  // namespace ZCANBus
#endif