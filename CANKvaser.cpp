#include "CANKvaser.h"
namespace ZCANBus {
CANKvaser::CANKvaser() : loopOn(false) { canInitializeLibrary(); }

CANKvaser::~CANKvaser() {}

CANStatus CANKvaser::OpenChannel(int channel, CANRate baudRate, int type) {
        char* argv[]={(char*)&type};
        return OpenChannel(channel, baudRate, 1, argv);
}

CANStatus CANKvaser::OpenChannel(int channel, CANRate baudRate, int argc,
                                 char* argv[]) {
    long freq = 0;
    switch (baudRate) {
        case CANRate::CAN_RATE_10K:
            freq = canBITRATE_10K;
            break;
        case CANRate::CAN_RATE_50K:
            freq = canBITRATE_50K;
            break;
        case CANRate::CAN_RATE_62K:
            freq = canBITRATE_62K;
            break;
        case CANRate::CAN_RATE_83K:
            freq = canBITRATE_83K;
            break;
        case CANRate::CAN_RATE_100K:
            freq = canBITRATE_100K;
            break;
        case CANRate::CAN_RATE_125K:
            freq = canBITRATE_125K;
            break;
        case CANRate::CAN_RATE_250K:
            freq = canBITRATE_250K;
            break;
        case CANRate::CAN_RATE_500K:
            freq = canBITRATE_500K;
            break;
        case CANRate::CAN_RATE_1M:
            freq = canBITRATE_1M;
            break;
        default:
            return canERR_PARAM;
    }
    int flags = 0;
    unsigned int tseg1 = 0;
    unsigned int tseg2 = 0;
    unsigned int sjw = 0;
    unsigned int noSamp = 0;
    unsigned int syncmode = 0;
    if (argc > 5) {
        syncmode = *(unsigned int*)argv[5];
    }
    if (argc > 4) {
        noSamp = *(unsigned int*)argv[4];
    }
    if (argc > 3) {
        sjw = *(unsigned int*)argv[3];
    }
    if (argc > 2) {
        tseg2 = *(unsigned int*)argv[2];
    }
    if (argc > 1) {
        tseg1 = *(unsigned int*)argv[1];
    }
    if (argc > 0) {
        flags = *(int*)argv[0];
    }
    handle = canOpenChannel(channel, flags);
    if (handle < 0) return handle;
    CANStatus stat = 0;
    do {
        stat |=
            canSetBusParams(handle, freq, tseg1, tseg2, sjw, noSamp, syncmode);
        stat |= canBusOn(handle);
    } while (false);
    return stat;
}

void CANKvaser::ReadLoop(
    std::function<void(const CANMessage* msg, CANStatus status)> callback,
    uint64_t interval) {
    th = new std::thread([&, callback, interval]() -> void {
        loopOn = true;
        CANMessage msg;
        CANStatus stat;
        while (loopOn) {
            stat = canReadWait(handle, &msg.id, msg.msg, &msg.length, &msg.type,
                               &msg.timestamp, interval);
            callback(&msg, stat);
        }
    });
}

void CANKvaser::EndReadLoop() {
    loopOn = false;
    th->join();
    delete th;
}

CANStatus CANKvaser::ReadOnce(CANMessage& msg, uint64_t timeout) {
    return canReadWait(handle, &msg.id, msg.msg, &msg.length, &msg.type,
                       &msg.timestamp, timeout);
}

CANStatus CANKvaser::Write(const CANMessage& msg) {
    return canWriteWait(handle, msg.id, (void*)msg.msg, msg.length, msg.type,
                        0xFF);
    ;
};

CANStatus CANKvaser::Write(CANMessage* msg, int count) {
    CANStatus status = 0;
    for (int i = 0; i < count; ++i) {
        status |= canWriteWait(handle, msg[i].id, msg[i].msg, msg[i].length,
                               msg[i].type, 0xFF);
    }
    return status;
}

CANStatus CANKvaser::CloseChannel() {
    if (loopOn) {
        EndReadLoop();
    }
    CANStatus stat = canWriteSync(handle, 0xFF);
    stat |= canBusOff(handle);
    return stat | canClose(handle);
}

CANStatus CANKvaser::FlushQueue() {
    return canFlushReceiveQueue(handle) | canFlushTransmitQueue(handle);
}

std::string CANKvaser::GetErrorText(CANStatus& status) {
    char buf[256];
    status = canGetErrorText((canStatus)status, buf, sizeof(buf));
    return std::string(buf);
}
}  // namespace ZCANBus