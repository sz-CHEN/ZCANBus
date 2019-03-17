#include "CANPeak.h"
#include <cstring>
#ifdef __APPLE__
#include <sys/select.h>
#endif
namespace ZCANBus {
CANPeak::CANPeak() {}

CANPeak::~CANPeak() {}

CANStatus CANPeak::OpenChannel(int channel, CANRate baudRate, int type) {
        char* argv[]={(char*)&type};
        return OpenChannel(channel, baudRate, 1, argv);
}

CANStatus CANPeak::OpenChannel(int channel, CANRate baudRate, int argc,
                               char* argv[]) {
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
            return PCAN_ERROR_ILLOPERATION;
    }
    this->channel = channel;
    unsigned int channelStatus;
    CANStatus stat = CAN_GetValue(channel, PCAN_CHANNEL_CONDITION,
                                  &channelStatus, sizeof(channelStatus));
    if (!(channelStatus & PCAN_CHANNEL_AVAILABLE)) {
        return PCAN_ERROR_HWINUSE;
    }
    TPCANType hwType = 0;
    DWORD IOPort = 0;
    WORD interrupt = 0;
    if (argc > 2) {
        interrupt = *(WORD*)argv[2];
    }
    if (argc > 1) {
        IOPort = *(DWORD*)argv[1];
    }
    if (argc > 0) {
        hwType = *(TPCANType*)argv[0];
    }
    return CAN_Initialize(channel, freq, hwType, IOPort, interrupt);
}

void CANPeak::ReadLoop(
    std::function<void(const CANMessage* msg, CANStatus status)> callback,
    uint64_t interval) {
    th = new std::thread([&, callback, interval]() -> void {
        loopOn = true;
        CANMessage msg;
        CANStatus stat = 0;
#ifdef _WIN32
        HANDLE evRecv = CreateEvent(NULL, FALSE, FALSE, NULL);
        stat =
            CAN_SetValue(channel, PCAN_RECEIVE_EVENT, &evRecv, sizeof(evRecv));
#elif __linux__ || __APPLE__ || __unix__
        int evRecv = 0;
        stat =
            CAN_GetValue(channel, PCAN_RECEIVE_EVENT, &evRecv, sizeof(evRecv));
        fd_set fds;
#else
#warning Unsupported OS
#endif  // WIN32
        while (loopOn) {
#ifdef _WIN32
            if (WaitForSingleObject(evRecv, interval) == WAIT_OBJECT_0)
#elif __linux__ || __APPLE__ || __unix__
            struct timeval time;
            time.tv_sec = 0;
            time.tv_usec = interval * 1000;
            FD_ZERO(&fds);
            FD_SET(evRecv, &fds);
            if (select(evRecv + 1, &fds, NULL, NULL, &time) > 0 &&
                FD_ISSET(evRecv, &fds))
#else
#warning Unsupported OS
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
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
    CANStatus stat = 0;
#ifdef _WIN32
    HANDLE evRecv = CreateEvent(NULL, FALSE, FALSE, NULL);
    stat = CAN_SetValue(channel, PCAN_RECEIVE_EVENT, &evRecv, sizeof(evRecv));
    if (WaitForSingleObject(evRecv, interval) == WAIT_OBJECT_0)
#elif __linux__ || __APPLE__ || __unix__
    int evRecv = 0;
    stat = CAN_GetValue(channel, PCAN_RECEIVE_EVENT, &evRecv, sizeof(evRecv));
    fd_set fds;
    struct timeval time;
    time.tv_sec = 0;
    time.tv_usec = timeout * 1000;
    FD_ZERO(&fds);
    FD_SET(evRecv, &fds);
    // std::this_thread::sleep_for(std::chrono::milliseconds(interval));
    if (select(evRecv + 1, &fds, NULL, NULL, &time) > 0 &&
        FD_ISSET(evRecv, &fds))
#else
#warning Unsupported OS
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
#endif
    {
        TPCANMsg buf;
        TPCANTimestamp timeStamp;
        stat = CAN_Read(channel, &buf, &timeStamp);
        if (stat == PCAN_ERROR_OK) {
            msg.id = buf.ID;
            msg.length = buf.LEN;
            msg.type = buf.MSGTYPE;
            msg.timestamp = timeStamp.millis;
            memcpy(msg.msg, buf.DATA, msg.length);
        }
        return stat;
    }
    return PCAN_ERROR_QRCVEMPTY;
}
CANStatus CANPeak::Write(const CANMessage& msg) {
    TPCANMsg message;
    message.ID = msg.id;
    message.LEN = msg.length;
    message.MSGTYPE = msg.type;
    memcpy(message.DATA, msg.msg, msg.length);
    return CAN_Write(channel, &message);
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