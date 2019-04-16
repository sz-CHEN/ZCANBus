#include "CANZLG2.h"
#include <cstring>
namespace ZCANBus {
CANZLG2::CANZLG2() : loopOn(false) {}

CANZLG2::~CANZLG2() {}

CANStatus CANZLG2::OpenChannel(int channel, CANRate baudRate, int type) {
    char* argv[] = {(char*)&type};
    return OpenChannel(channel, baudRate, 1, argv);
}

CANStatus CANZLG2::OpenChannel(int channel, CANRate baudRate, int argc,
                               char* argv[]) {
    UINT can_index = channel;
    UINT device_index = 0;
    UINT device_type = 0;
    if (argc > 1) {
        device_index = *(UINT*)argv[1];
    }
    if (argc > 0) {
        device_type = *(UINT*)argv[0];
    }
    dhandle = ZCAN_OpenDevice(device_type, device_index, 0);
    if (dhandle == INVALID_DEVICE_HANDLE) {
        return ERR_DEVICEOPEN;
    }
    std::string baudrate;
    ZCAN_CHANNEL_INIT_CONFIG config;
    config.can_type = 0;
    config.can.acc_code = 0;
    config.can.acc_mask = 0xFFFFFFFF;
    config.can.reserved = 0;
    config.can.filter = 1;
    config.can.mode = 0;
    switch (baudRate) {
        case CANRate::CAN_RATE_10K:
            config.can.timing0 = 0x31;
            config.can.timing1 = 0x1C;
            baudrate = "10000";
            break;

        case CANRate::CAN_RATE_20K:
            config.can.timing0 = 0x18;
            config.can.timing1 = 0x1C;
            baudrate = "20000";
            break;

        case CANRate::CAN_RATE_40K:
            config.can.timing0 = 0x87;
            config.can.timing1 = 0xFF;
            baudrate = "40000";
            break;

        case CANRate::CAN_RATE_50K:
            config.can.timing0 = 0x09;
            config.can.timing1 = 0x1C;
            baudrate = "50000";
            break;

        case CANRate::CAN_RATE_80K:
            config.can.timing0 = 0x83;
            config.can.timing1 = 0xFF;
            baudrate = "80000";
            break;

        case CANRate::CAN_RATE_100K:
            config.can.timing0 = 0x04;
            config.can.timing1 = 0x1C;
            baudrate = "100000";
            break;

        case CANRate::CAN_RATE_125K:
            config.can.timing0 = 0x03;
            config.can.timing1 = 0x1C;
            baudrate = "125000";
            break;

        case CANRate::CAN_RATE_250K:
            config.can.timing0 = 0x01;
            config.can.timing1 = 0x1C;
            baudrate = "250000";
            break;

        case CANRate::CAN_RATE_400K:
            config.can.timing0 = 0x80;
            config.can.timing1 = 0xFA;
            baudrate = "400000";
            break;

        case CANRate::CAN_RATE_500K:
            config.can.timing0 = 0x00;
            config.can.timing1 = 0x1C;
            baudrate = "500000";
            break;

        case CANRate::CAN_RATE_666K:
            config.can.timing0 = 0x80;
            config.can.timing1 = 0xB6;
            baudrate = "666000";
            break;

        case CANRate::CAN_RATE_800K:
            config.can.timing0 = 0x00;
            config.can.timing1 = 0x16;
            baudrate = "800000";
            break;

        case CANRate::CAN_RATE_1M:
            config.can.timing0 = 0x00;
            config.can.timing1 = 0x14;
            baudrate = "1000000";
            break;

        default:
            return ERR_DEVICEOPEN;
            break;
    }
    chandle = ZCAN_InitCAN(dhandle, can_index, &config);
    if (INVALID_CHANNEL_HANDLE == chandle) {
        ZCAN_CloseDevice(dhandle);
        return ERR_DEVICEOPEN;
    }
    switch (device_type) {
        case ZCAN_USBCAN_E_U:
        case ZCAN_USBCAN_2E_U:
        case ZCAN_USBCAN_4E_U:
        case ZCAN_PCI5010U:
        case ZCAN_PCI5020U:
        case ZCAN_CANDTU: {
            auto* p = GetIProperty(dhandle);
            auto path = std::to_string(can_index) + "/baud_rate";
            if (0 == p->SetValue(path.c_str(), baudrate.c_str())) {
                ZCAN_CHANNEL_ERR_INFO errInfo;
                if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
                    ZCAN_CloseDevice(dhandle);
                    if (0 < errInfo.error_code) {
                        return errInfo.error_code;
                    }
                }
                ZCAN_CloseDevice(dhandle);
                return -1;
            }
            break;
        }
        default:
            break;
    }
    if (STATUS_OK != ZCAN_ResetCAN(chandle)) {
        ZCAN_CHANNEL_ERR_INFO errInfo;
        if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
            ZCAN_CloseDevice(dhandle);
            if (0 < errInfo.error_code) {
                return errInfo.error_code;
            }
        }
        ZCAN_CloseDevice(dhandle);
        return -1;
    }
    if (STATUS_OK != ZCAN_StartCAN(chandle)) {
        ZCAN_CHANNEL_ERR_INFO errInfo;
        if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
            ZCAN_CloseDevice(dhandle);
            if (0 < errInfo.error_code) {
                return errInfo.error_code;
            }
        }
        ZCAN_CloseDevice(dhandle);
        return -1;
    }
    return 0;
}

void CANZLG2::ReadLoop(
    std::function<void(const CANMessage* msg, CANStatus status)> callback,
    uint64_t interval) {
    th = new std::thread([&, callback, interval]() -> void {
        loopOn = true;
        CANMessage msg;
        CANStatus stat;
        ZCAN_Receive_Data data;
        ZCAN_CHANNEL_ERR_INFO errInfo;
        while (loopOn) {
            stat = ZCAN_Receive(chandle, &data, 1, interval);
            if (stat > 0) {
                msg.timestamp = data.timestamp;
                msg.length = data.frame.can_dlc;
                msg.id = data.frame.can_id & 0x1FFFFFFF;
                msg.type = (data.frame.can_id >> 29) & 0x07;
                unsigned int type = (unsigned int)CANMSGType::STANDARD;
                if (msg.type & 1) {
                    type |= (unsigned int)CANMSGType::ERRFRAME;
                }
                if (msg.type & (1 << 1)) {
                    type |= (unsigned int)CANMSGType::RTR;
                }
                if (msg.type & (1 << 2)) {
                    type |= (unsigned int)CANMSGType::EXTENDED;
                }
                msg.type = type;
                memcpy(msg.msg, data.frame.data, msg.length);
                callback(&msg, 0);
            } else if (stat < 0) {
                if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
                    if (0 < errInfo.error_code) {
                        callback(&msg, errInfo.error_code);
                    }
                }
            }
        }
    });
}

void CANZLG2::EndReadLoop() {
    loopOn = false;
    th->join();
    delete th;
}

CANStatus CANZLG2::ReadOnce(CANMessage& msg, uint64_t timeout) {
    CANStatus stat;
    ZCAN_Receive_Data data;
    stat = ZCAN_Receive(chandle, &data, 1, timeout);
    if (stat > 0) {
        msg.timestamp = data.timestamp;
        msg.length = data.frame.can_dlc;
        msg.id = data.frame.can_id & 0x1FFFFFFF;
        msg.type = (data.frame.can_id >> 29) & 0x07;
        unsigned int type = (unsigned int)CANMSGType::STANDARD;
        if (msg.type & 1) {
            type |= (unsigned int)CANMSGType::ERRFRAME;
        }
        if (msg.type & (1 << 1)) {
            type |= (unsigned int)CANMSGType::RTR;
        }
        if (msg.type & (1 << 2)) {
            type |= (unsigned int)CANMSGType::EXTENDED;
        }
        msg.type = type;
        memcpy(msg.msg, data.frame.data, msg.length);
        return 0;
    } else if (stat < 0) {
        ZCAN_CHANNEL_ERR_INFO errInfo;
        if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
            if (0 < errInfo.error_code) {
                return errInfo.error_code;
            }
        }
    }
    return -1;
}

CANStatus CANZLG2::Write(const CANMessage& msg) {
    uint8_t flag = 0;
    if (msg.type & (unsigned int)CANMSGType::EXTENDED) {
        flag |= 1 << 2;
    }
    if (msg.type & (unsigned int)CANMSGType::RTR) {
        flag |= 1 << 1;
    }
    if (msg.type & (unsigned int)CANMSGType::ERRFRAME) {
        flag |= 1;
    }
    ZCAN_Transmit_Data data;
    data.transmit_type = 0;
    data.frame.can_id = (msg.id & 0x1FFFFFFF) | ((flag & (0x07)) << 29);
    data.frame.can_dlc = msg.length;
    memcpy(data.frame.data, msg.msg, msg.length);
    auto stat = ZCAN_Transmit(chandle, &data, 1);
    if (stat > 0) {
        return 0;
    } else if (stat < 0) {
        ZCAN_CHANNEL_ERR_INFO errInfo;
        if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
            if (0 < errInfo.error_code) {
                return errInfo.error_code;
            }
        }
    }
    return -1;
};

CANStatus CANZLG2::Write(CANMessage* msg, int count) {
    ZCAN_Transmit_Data* data = new ZCAN_Transmit_Data[count];
    for (int i = 0; i < count; i++) {
        uint8_t flag = 0;
        if (msg.type & (unsigned int)CANMSGType::EXTENDED) {
            flag |= 1 << 2;
        }
        if (msg.type & (unsigned int)CANMSGType::RTR) {
            flag |= 1 << 1;
        }
        if (msg.type & (unsigned int)CANMSGType::ERRFRAME) {
            flag |= 1;
        }
        data[i].transmit_type = 0;
        data[i].frame.can_id = (msg.id & 0x1FFFFFFF) | ((flag & (0x07)) << 29);
        data[i].frame.can_dlc = msg[i].length;
        memcpy(data[i].frame.data, msg[i].msg, msg[i].length);
    }
    auto stat = ZCAN_Transmit(chandle, data, count);
    delete[] data;
    if (stat < 0) {
        ZCAN_CHANNEL_ERR_INFO errInfo;
        if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
            if (0 < errInfo.error_code) {
                return errInfo.error_code;
            }
        }
    }
    return stat - count;
}

CANStatus CANZLG2::CloseChannel() {
    if (loopOn) {
        EndReadLoop();
    }
    if (STATUS_OK != ZCAN_ResetCAN(chandle) ||
        STATUS_OK != ZCAN_CloseDevice(dhandle)) {
        return -1;
    }
    return 0;
}

CANStatus CANZLG2::FlushQueue() {
    if (STATUS_OK != ZCAN_ClearBuffer(chandle)) {
        ZCAN_CHANNEL_ERR_INFO errInfo;
        if (STATUS_OK == ZCAN_ReadChannelErrInfo(chandle, &errInfo)) {
            if (0 < errInfo.error_code) {
                return errInfo.error_code;
            }
        }
        return -1;
    } else {
        return 0;
    }
}

std::string CANZLG2::GetErrorText(CANStatus& status) {
    if (status == 0) {
        return "OK";
    } else if (status > 0) {
        switch (status) {
            case ERR_CAN_OVERFLOW:
                return "CAN控制器内部FIFO溢出";
                break;

            case ERR_CAN_ERRALARM:
                return "CAN控制器错误报警";
                break;

            case ERR_CAN_PASSIVE:
                return "CAN控制器消极错误";
                break;

            case ERR_CAN_LOSE:
                return "CAN控制器仲裁丢失";
                break;

            case ERR_CAN_BUSERR:
                return "CAN控制器总线错误";
                break;

            case ERR_CAN_BUFFER_OVERFLOW:
                return "CAN控制器内部BUFFER溢出";
                break;

            case ERR_DEVICEOPENED:
                return "设备已经打开";
                break;

            case ERR_DEVICEOPEN:
                return "打开设备错误";
                break;

            case ERR_DEVICENOTOPEN:
                return "设备没有打开";
                break;

            case ERR_BUFFEROVERFLOW:
                return "缓冲区溢出";
                break;
            case ERR_DEVICENOTEXIST:
                return "此设备不存在";
                break;
            case ERR_LOADKERNELDLL:
                return "装载动态库失败";
                break;
            case ERR_CMDFAILED:
                return "执行命令失败错误码";
                break;
            case ERR_BUFFERCREATE:
                return "内存不足";
                break;
            case ERR_CANETE_PORTOPENED:
                return "端口已经被打开";
                break;
            case ERR_CANETE_INDEXUSED:
                return "设备索引号已经被占用";
                break;
            case ERR_REF_TYPE_ID:
                return "SetReference或GetReference传递的RefType不存在";
                break;
            case ERR_CREATE_SOCKET:
                return "创建Socket失败";
                break;
            case ERR_OPEN_CONNECT:
                return "打开Socket的连接时失败，可能设备连接已经存在";
                break;
            case ERR_NO_STARTUP:
                return "设备没启动";
                break;
            case ERR_NO_CONNECTED:
                return "设备无连接";
                break;
            case ERR_SEND_PARTIAL:
                return "只发送了部分的CAN帧";
                break;
            case ERR_SEND_TOO_FAST:
                return "数据发得太快，Socket缓冲区满了";
                break;
            default:
                return "未知错误";
                break;
        }
    } else {
        return "未知错误";
    }
}
}  // namespace ZCANBus