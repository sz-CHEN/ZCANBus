#include "CANZLG.h"
#include <cstring>
namespace ZCANBus {
CANZLG::CANZLG() : loopOn(false) {}

CANZLG::~CANZLG() {}

CANStatus CANZLG::OpenChannel(int channel, CANRate baudRate, int type) {
    char* argv[] = {(char*)&type};
    return OpenChannel(channel, baudRate, 1, argv);
}

CANStatus CANZLG::OpenChannel(int channel, CANRate baudRate, int argc,
                              char* argv[]) {
    can_index = channel;
    device_index = 0;
    device_type = 0;
    if (argc > 1) {
        device_index = *(UINT*)argv[1];
    }
    if (argc > 0) {
        device_type = *(UINT*)argv[0];
    }
    if (STATUS_OK != VCI_OpenDevice(device_type, device_index, 0)) {
        return ERR_DEVICEOPEN;
    }
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Reserved = 0;
    config.Filter = 1;
    config.Mode = 0;
    switch (baudRate) {
        case CANRate::CAN_RATE_10K:
            config.Timing0 = 0x31;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_20K:
            config.Timing0 = 0x18;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_40K:
            config.Timing0 = 0x87;
            config.Timing1 = 0xFF;
            break;

        case CANRate::CAN_RATE_50K:
            config.Timing0 = 0x09;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_80K:
            config.Timing0 = 0x83;
            config.Timing1 = 0xFF;
            break;

        case CANRate::CAN_RATE_100K:
            config.Timing0 = 0x04;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_125K:
            config.Timing0 = 0x03;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_250K:
            config.Timing0 = 0x01;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_400K:
            config.Timing0 = 0x80;
            config.Timing1 = 0xFA;
            break;

        case CANRate::CAN_RATE_500K:
            config.Timing0 = 0x00;
            config.Timing1 = 0x1C;
            break;

        case CANRate::CAN_RATE_666K:
            config.Timing0 = 0x80;
            config.Timing1 = 0xB6;
            break;

        case CANRate::CAN_RATE_800K:
            config.Timing0 = 0x00;
            config.Timing1 = 0x16;
            break;

        case CANRate::CAN_RATE_1M:
            config.Timing0 = 0x00;
            config.Timing1 = 0x14;
            break;

        default:
            return ERR_DEVICEOPEN;
            break;
    }
    if (STATUS_OK !=
        VCI_InitCAN(device_type, device_index, can_index, &config)) {
        VCI_CloseDevice(device_type, device_index);
        return ERR_DEVICEOPEN;
    }
    if (STATUS_OK != VCI_ResetCAN(device_type, device_index, can_index)) {
        VCI_ERR_INFO errInfo;
        if (STATUS_OK ==
            VCI_ReadErrInfo(device_type, device_index, can_index, &errInfo)) {
            VCI_CloseDevice(device_type, device_index);
            if (0 < errInfo.ErrCode) {
                return errInfo.ErrCode;
            }
        }
        VCI_CloseDevice(device_type, device_index);
        return -1;
    }
    if (STATUS_OK != VCI_StartCAN(device_type, device_index, can_index)) {
        VCI_ERR_INFO errInfo;
        if (STATUS_OK ==
            VCI_ReadErrInfo(device_type, device_index, can_index, &errInfo)) {
            VCI_CloseDevice(device_type, device_index);
            if (0 < errInfo.ErrCode) {
                return errInfo.ErrCode;
            }
        }
        VCI_CloseDevice(device_type, device_index);
        return -1;
    }
    return 0;
}

void CANZLG::ReadLoop(
    std::function<void(const CANMessage* msg, CANStatus status)> callback,
    uint64_t interval) {
    th = new std::thread([&, callback, interval]() -> void {
        loopOn = true;
        CANMessage msg;
        CANStatus stat;
        VCI_CAN_OBJ data;
        VCI_ERR_INFO errInfo;
        while (loopOn) {
            stat = VCI_Receive(device_type, device_index, can_index, &data, 1,
                               interval);
            if (stat > 0) {
                msg.timestamp = data.TimeStamp;
                msg.length = data.DataLen;
                msg.id = data.ID;
                msg.type = (unsigned int)CANMSGType::STANDARD;
                if (data.ExternFlag) {
                    msg.type |= (unsigned int)CANMSGType::EXTENDED;
                }
                if (data.RemoteFlag) {
                    msg.type |= (unsigned int)CANMSGType::RTR;
                }
                memcpy(msg.msg, data.Data, msg.length);
                callback(&msg, 0);
            } else if (stat < 0) {
                if (STATUS_OK == VCI_ReadErrInfo(device_type, device_index,
                                                 can_index, &errInfo)) {
                    if (0 < errInfo.ErrCode) {
                        callback(&msg, errInfo.ErrCode);
                    }
                }
            }
        }
    });
}

void CANZLG::EndReadLoop() {
    loopOn = false;
    th->join();
    delete th;
}

CANStatus CANZLG::ReadOnce(CANMessage& msg, uint64_t timeout) {
    CANStatus stat;
    VCI_CAN_OBJ data;
    stat = VCI_Receive(device_type, device_index, can_index, &data, 1, timeout);
    if (stat > 0) {
        msg.timestamp = data.TimeStamp;
        msg.length = data.DataLen;
        msg.id = data.ID;
        msg.type = (unsigned int)CANMSGType::STANDARD;
        if (data.ExternFlag) {
            msg.type |= (unsigned int)CANMSGType::EXTENDED;
        }
        if (data.RemoteFlag) {
            msg.type |= (unsigned int)CANMSGType::RTR;
        }
        memcpy(msg.msg, data.Data, msg.length);
        return 0;
    } else if (stat < 0) {
        VCI_ERR_INFO errInfo;
        if (STATUS_OK ==
            VCI_ReadErrInfo(device_type, device_index, can_index, &errInfo)) {
            if (0 < errInfo.ErrCode) {
                return errInfo.ErrCode;
            }
        }
    }
    return -1;
}

CANStatus CANZLG::Write(const CANMessage& msg) {
    VCI_CAN_OBJ data;
    data.SendType = 0;
    data.ID = msg.id;
    data.DataLen = msg.length;
    if (msg.type & (unsigned int)CANMSGType::RTR) {
        data.RemoteFlag = 1;
    } else {
        data.RemoteFlag = 0;
    }
    if (msg.type & (unsigned int)CANMSGType::EXTENDED) {
        data.ExternFlag = 1;
    } else {
        data.ExternFlag = 0;
    }
    memcpy(data.Data, msg.msg, msg.length);
    CANStatus stat =
        VCI_Transmit(device_type, device_index, can_index, &data, 1);
    if (stat > 0) {
        return 0;
    } else if (stat < 0) {
        VCI_ERR_INFO errInfo;
        if (STATUS_OK ==
            VCI_ReadErrInfo(device_type, device_index, can_index, &errInfo)) {
            if (0 < errInfo.ErrCode) {
                return errInfo.ErrCode;
            }
        }
    }
    return -1;
};

CANStatus CANZLG::Write(CANMessage* msg, int count) {
    VCI_CAN_OBJ* data = new VCI_CAN_OBJ[count];
    for (int i = 0; i < count; i++) {
        data[i].SendType = 0;
        data[i].ID = msg[i].id;
        data[i].DataLen = msg[i].length;
        if (msg.type & (unsigned int)CANMSGType::RTR) {
            data.RemoteFlag = 1;
        } else {
            data.RemoteFlag = 0;
        }
        if (msg.type & (unsigned int)CANMSGType::EXTENDED) {
            data.ExternFlag = 1;
        } else {
            data.ExternFlag = 0;
        }
        memcpy(data[i].Data, msg[i].msg, msg[i].length);
    }
    CANStatus stat =
        VCI_Transmit(device_type, device_index, can_index, data, count);
    delete[] data;
    if (stat < 0) {
        VCI_ERR_INFO errInfo;
        if (STATUS_OK ==
            VCI_ReadErrInfo(device_type, device_index, can_index, &errInfo)) {
            if (0 < errInfo.ErrCode) {
                return errInfo.ErrCode;
            }
        }
    }
    return stat - count;
}

CANStatus CANZLG::CloseChannel() {
    if (loopOn) {
        EndReadLoop();
    }
    if (STATUS_OK != VCI_ResetCAN(device_type, device_index, can_index) ||
        STATUS_OK != VCI_CloseDevice(device_type, device_index)) {
        return -1;
    }
    return 0;
}

CANStatus CANZLG::FlushQueue() {
    if (STATUS_OK != VCI_ClearBuffer(device_type, device_index, can_index)) {
        VCI_ERR_INFO errInfo;
        if (STATUS_OK ==
            VCI_ReadErrInfo(device_type, device_index, can_index, &errInfo)) {
            if (0 < errInfo.ErrCode) {
                return errInfo.ErrCode;
            }
        }
        return -1;
    } else {
        return 0;
    }
}

std::string CANZLG::GetErrorText(CANStatus& status) {
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
            // case ERR_CANETE_PORTOPENED:
            //     return "端口已经被打开";
            //     break;
            // case ERR_CANETE_INDEXUSED:
            //     return "设备索引号已经被占用";
            //     break;
            // case ERR_REF_TYPE_ID:
            //     return "SetReference或GetReference传递的RefType不存在";
            //     break;
            // case ERR_CREATE_SOCKET:
            //     return "创建Socket失败";
            //     break;
            // case ERR_OPEN_CONNECT:
            //     return "打开Socket的连接时失败，可能设备连接已经存在";
            //     break;
            // case ERR_NO_STARTUP:
            //     return "设备没启动";
            //     break;
            // case ERR_NO_CONNECTED:
            //     return "设备无连接";
            //     break;
            // case ERR_SEND_PARTIAL:
            //     return "只发送了部分的CAN帧";
            //     break;
            // case ERR_SEND_TOO_FAST:
            //     return "数据发得太快，Socket缓冲区满了";
            //     break;
            default:
                return "未知错误";
                break;
        }
    } else {
        return "未知错误";
    }
}
}  // namespace ZCANBus