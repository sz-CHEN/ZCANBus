//
// Created by ShizengChen on 2017/12/24.
//

#include "CANHandler.h"
#include "CANBase.h"
#ifdef USE_PEAK
#include "CANPeak.h"
#endif
#ifdef USE_KVASER
#include "CANKvaser.h"
#endif

namespace ZCANBus {
CANStatus CANHandler::OpenChannel(int channel, CANRate baudRate, int type) {
    return baseCan->OpenChannel(channel, baudRate, type);
}

void CANHandler::ReadLoop(
    std::function<void(const CANMessage *msg, CANStatus status)> callback,
    uint64_t interval) {
    baseCan->ReadLoop(callback, interval);
}

void CANHandler::EndReadLoop() { baseCan->EndReadLoop(); }

CANStatus CANHandler::ReadOnce(CANMessage &msg, uint64_t timeout) {
    return baseCan->ReadOnce(msg);
}

CANStatus CANHandler::Write(CANMessage *msg, int count) {
    return baseCan->Write(msg, count);
}

CANStatus CANHandler::CloseChannel() { return baseCan->CloseChannel(); }

CANStatus CANHandler::FlushQueue() { return baseCan->FlushQueue(); }

std::string CANHandler::GetErrorText(CANStatus &status) {
    return baseCan->GetErrorText(status);
}

CANHandler::CANHandler(CANType canType) {
    switch (canType) {
#ifdef USE_PEAK
        case CANType::PEAK_CAN:
            baseCan = new CANPeak();
            break;
#endif
#ifdef USE_KVASER
        case CANType::KVASER_CAN:
            baseCan = new CANKvaser();
            break;
#endif
        default:
            throw;
    }
}

CANHandler::~CANHandler() { delete baseCan; }
}  // namespace ZCANBus