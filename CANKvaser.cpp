#include "CANKvaser.h"
namespace ZCANBus {
CANKvaser::CANKvaser() : loopOn(false) { canInitializeLibrary(); }

CANKvaser::~CANKvaser() {}

CANStatus CANKvaser::OpenChannel(int channel, CANRate baudRate, int type) {
    void* argv[] = {&type};
    return OpenChannel(channel, baudRate, 1, argv);
}

CANStatus CANKvaser::OpenChannel(int channel, CANRate baudRate, int argc,
                                 void* argv[]) {
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
            long id;
            unsigned int length;
            unsigned int flag;
            unsigned long time;
            stat = canReadWait(handle, &id, msg.msg, &length, &flag, &time,
                               interval);
            msg.id = (unsigned long)id;
            msg.length = length;
            msg.timestamp = time;
            uint32_t type = (uint32_t)CANMSGType::STANDARD;
            if (flag & canMSG_EXT) {
                type |= (uint32_t)CANMSGType::EXTENDED;
                flag &= ~canMSG_EXT;
            }
            if (flag & canMSG_RTR) {
                type |= (uint32_t)CANMSGType::RTR;
                flag &= ~canMSG_RTR;
            }
            if (flag & canMSG_ERROR_FRAME) {
                type |= (uint32_t)CANMSGType::ERRFRAME;
                flag &= ~canMSG_ERROR_FRAME;
            }
            if (flag & canFDMSG_FDF) {
                type |= (uint32_t)CANMSGType::FD;
                flag &= ~canFDMSG_FDF;
            }
            if (flag & canFDMSG_BRS) {
                type |= (uint32_t)CANMSGType::BRS;
                flag &= ~canFDMSG_BRS;
            }
            if (flag & canFDMSG_ESI) {
                type |= (uint32_t)CANMSGType::ESI;
                flag &= ~canFDMSG_ESI;
            }
            if (flag & canMSG_STD) {
                flag &= ~canMSG_STD;
            }
            if (flag) {
                msg.type =
                    ((uint32_t)CANMSGType::HARDWAREDEF) | ((uint32_t)flag);
            } else {
                msg.type = type;
            }
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
    long id;
    unsigned int length;
    unsigned int flag;
    unsigned long time;
    auto&& status =
        canReadWait(handle, &id, msg.msg, &length, &flag, &time, timeout);
    msg.id = (unsigned long)id;
    msg.length = length;
    msg.timestamp = time;
    uint32_t type = (uint32_t)CANMSGType::STANDARD;
    if (flag & canMSG_EXT) {
        type |= (uint32_t)CANMSGType::EXTENDED;
        flag &= ~canMSG_EXT;
    }
    if (flag & canMSG_RTR) {
        type |= (uint32_t)CANMSGType::RTR;
        flag &= ~canMSG_RTR;
    }
    if (flag & canMSG_ERROR_FRAME) {
        type |= (uint32_t)CANMSGType::ERRFRAME;
        flag &= ~canMSG_ERROR_FRAME;
    }
    if (flag & canFDMSG_FDF) {
        type |= (uint32_t)CANMSGType::FD;
        flag &= ~canFDMSG_FDF;
    }
    if (flag & canFDMSG_BRS) {
        type |= (uint32_t)CANMSGType::BRS;
        flag &= ~canFDMSG_BRS;
    }
    if (flag & canFDMSG_ESI) {
        type |= (uint32_t)CANMSGType::ESI;
        flag &= ~canFDMSG_ESI;
    }
    if (flag & canMSG_STD) {
        flag &= ~canMSG_STD;
    }
    if (flag) {
        msg.type = ((uint32_t)CANMSGType::HARDWAREDEF) | ((uint32_t)flag);
    } else {
        msg.type = type;
    }
    return status;
}

CANStatus CANKvaser::Write(const CANMessage& msg) {
    uint32_t flag = canMSG_STD;
    if (msg.type & (uint32_t)CANMSGType::HARDWAREDEF) {
        flag = msg.type & (~(uint32_t)CANMSGType::HARDWAREDEF);
    } else {
        if ((uint32_t)msg.type & (uint32_t)CANMSGType::EXTENDED) {
            flag |= canMSG_EXT;
            flag &= ~canMSG_STD;
        }
        if ((uint32_t)msg.type & (uint32_t)CANMSGType::RTR) {
            flag |= canMSG_RTR;
        }
        if ((uint32_t)msg.type & (uint32_t)CANMSGType::ERRFRAME) {
            flag |= canMSG_ERROR_FRAME;
        }
        if ((uint32_t)msg.type & (uint32_t)CANMSGType::FD) {
            flag |= canFDMSG_FDF;
        }
        if ((uint32_t)msg.type & (uint32_t)CANMSGType::BRS) {
            flag |= canFDMSG_BRS;
        }
        if ((uint32_t)msg.type & (uint32_t)CANMSGType::ESI) {
            flag |= canFDMSG_ESI;
        }
    }
    return canWriteWait(handle, (int32_t)msg.id, (void*)msg.msg, msg.length,
                        flag, 0xFF);
};

CANStatus CANKvaser::Write(CANMessage* msg, int count) {
    CANStatus status = 0;
    for (int i = 0; i < count; ++i) {
        status |= Write(msg[i]);
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