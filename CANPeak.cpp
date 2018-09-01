#include "CANPeak.h"
#include <cstring>
namespace ZCANBus {
CANPeak::CANPeak() {}

CANPeak::~CANPeak() {}

CANStatus CANPeak::OpenChannel(int channel, CANRate baudRate, int type) {
    TPCANBaudrate freq = 0;
    switch (baudRate) {
        case CANRate::CAN_RATE_5K:
            freq = PCAN_BAUD_5K;
            break;
        case CANRate::CAN_RATE_10K:
            freq = PCAN_BAUD_10K;
            break;
        case CANRate::CAN_RATE_20K:
            freq = PCAN_BAUD_20K;
            break;
        case CANRate::CAN_RATE_33K:
            freq = PCAN_BAUD_33K;
            break;
        case CANRate::CAN_RATE_47K:
            freq = PCAN_BAUD_47K;
            break;
        case CANRate::CAN_RATE_50K:
            freq = PCAN_BAUD_50K;
            break;
        case CANRate::CAN_RATE_83K:
            freq = PCAN_BAUD_83K;
            break;
        case CANRate::CAN_RATE_95K:
            freq = PCAN_BAUD_95K;
            break;
        case CANRate::CAN_RATE_100K:
            freq = PCAN_BAUD_100K;
            break;
        case CANRate::CAN_RATE_125K:
            freq = PCAN_BAUD_125K;
            break;
        case CANRate::CAN_RATE_250K:
            freq = PCAN_BAUD_250K;
            break;
        case CANRate::CAN_RATE_500K:
            freq = PCAN_BAUD_500K;
            break;
        case CANRate::CAN_RATE_800K:
            freq = PCAN_BAUD_800K;
            break;
        case CANRate::CAN_RATE_1M:
            freq = PCAN_BAUD_1M;
            break;
        default:
            return PCAN_ERROR_UNKNOWN;
    }
    this->channel = channel;
    unsigned int channelStatus;
    CANStatus stat = CAN_GetValue(channel, PCAN_CHANNEL_CONDITION,
                                  &channelStatus, sizeof(channelStatus));
    if (!(channelStatus & PCAN_CHANNEL_AVAILABLE)) {
        return PCAN_ERROR_INITIALIZE;
    }
    return CAN_Initialize(channel, freq, type);
}

void CANPeak::ReadLoop(
    std::function<void(const CANMessage* msg, CANStatus status)> callback,
    uint64_t interval) {
    th = new std::thread([&, callback, interval]() -> void {
        loopOn = true;
        CANMessage msg;
        CANStatus stat;
#ifdef _WIN32
        HANDLE evRecv;
#elif defined(__linux__) || defined(__APPLE__)
        int evRecv = 1;
        fd_set Fds;
#endif  // WIN32
#ifdef _WIN32
        evRecv = CreateEvent(NULL, FALSE, FALSE, NULL);
// #endif // WIN32
#elif defined(__linux__) || defined(__APPLE__)
        FD_ZERO(&Fds);
        FD_SET(evRecv, &Fds);
#endif
        stat =
            CAN_SetValue(channel, PCAN_RECEIVE_EVENT, &evRecv, sizeof(evRecv));
        while (loopOn) {
#ifdef _WIN32
            if (WaitForSingleObject(evRecv, interval) == WAIT_OBJECT_0)
#elif defined(__linux__) || defined(__APPLE__)
            timeval time;
            time.tv_sec = 0;
            time.tv_usec = interval * 1000;
            if (select(evRecv + 1, &Fds, NULL, NULL, &time) > 0)
#endif
            {
                TPCANMsg buf;
                TPCANTimestamp timeStamp;
                TPCANStatus status;
                status = CAN_Read(channel, &buf, &timeStamp);
                while (!(status &
                         (PCAN_ERROR_QRCVEMPTY | PCAN_ERROR_ILLOPERATION)) &&
                       loopOn) {
                    msg.id = buf.ID;
                    msg.length = buf.LEN;
                    msg.type = buf.MSGTYPE;
                    msg.timestamp = timeStamp.millis;
                    memcpy(msg.msg, buf.DATA, msg.length);
                    callback(&msg, status);
                    status = CAN_Read(channel, &buf, &timeStamp);
                }
            }
        }
    });
}

void CANPeak::EndReadLoop() {
    loopOn = false;
    th->join();
    delete th;
}

CANStatus CANPeak::ReadOnce(CANMessage& msg, uint64_t timeout) {
#ifdef WIN32
    HANDLE evRecv;
#elif defined(__linux__) || defined(__APPLE__)
    int evRecv = 1;
    fd_set Fds;
#endif  // WIN32
#ifdef WIN32
    evRecv = CreateEvent(NULL, FALSE, FALSE, NULL);
// #endif // WIN32
#elif defined(__linux__) || defined(__APPLE__)
    FD_ZERO(&Fds);
    FD_SET(evRecv, &Fds);
#endif
    CANStatus status =
        CAN_SetValue(channel, PCAN_RECEIVE_EVENT, &evRecv, sizeof(evRecv));
#ifdef _WIN32
    if (WaitForSingleObject(evRecv, timeout) == WAIT_OBJECT_0)
#elif defined(__linux__) || defined(__APPLE__)
    timeval time;
    time.tv_sec = 0;
    time.tv_usec = timeout * 1000;
    if (select(evRecv + 1, &Fds, NULL, NULL, &time) > 0)
#endif
    {
        TPCANMsg buf;
        TPCANTimestamp timeStamp;
        status = CAN_Read(channel, &buf, &timeStamp);
        if (status == PCAN_ERROR_OK) {
            msg.id = buf.ID;
            msg.length = buf.LEN;
            msg.type = buf.MSGTYPE;
            msg.timestamp = timeStamp.millis;
            memcpy(msg.msg, buf.DATA, msg.length);
        }
        return status;
    }
    return PCAN_ERROR_QRCVEMPTY;
}

CANStatus CANPeak::Write(CANMessage* msg, int count) {
    CANStatus status = 0;
    for (int i = 0; i < count; ++i) {
        TPCANMsg message;
        message.ID = msg[i].id;
        message.LEN = msg[i].length;
        message.MSGTYPE = msg[i].type;
        memcpy(message.DATA, msg[i].msg, msg[i].length);
        status |= CAN_Write(channel, &message);
    }
    return status;
}

CANStatus CANPeak::CloseChannel() {
    if (loopOn) {
        EndReadLoop();
    }
    // CAN_Reset(channel);
    return CAN_Uninitialize(channel);
}

CANStatus CANPeak::FlushQueue() { return CAN_Reset(channel); }

std::string CANPeak::GetErrorText(CANStatus& status) {
    char strMsg[256];
    status = CAN_GetErrorText(status, 0, strMsg);
    return std::string(strMsg);
}
}  // namespace ZCANBus