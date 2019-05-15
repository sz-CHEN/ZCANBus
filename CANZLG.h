#pragma once
#ifndef __CANBase_ZLG_H
#define __CANBase_ZLG_H
#include <controlcan.h>
#include <map>
#include <thread>
#include <vector>
#include "CANBase.h"

namespace ZCANBus {
class CANZLG : public CANBase {
   private:
    std::thread* th;
    bool loopOn;
    UINT can_index = 0;
    UINT device_index = 0;
    UINT device_type = 0;
    using device_index_t = UINT;
    using device_type_t = UINT;
    static std::map<std::pair<device_type_t, device_index_t>, UINT>
        openedCount;
    bool opened;

   public:
    CANZLG();
    ~CANZLG();
    /**
     * @param channel Indicates can_index
     * @param type Indicates device_type
     * @see <controlcan.h>
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int type = 0) override;

    /**
     * @param channel Indicates can_index
     * @param argv Indicates device_type, device_index.
     * Default 0.
     * @see <controlcan.h>
     */
    CANStatus OpenChannel(int channel, CANRate baudRate, int argc,
                          void* argv[]) override;
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