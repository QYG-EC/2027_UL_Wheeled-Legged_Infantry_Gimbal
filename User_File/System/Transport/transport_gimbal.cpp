#include "board_transport.h"

#include <cmath>

#include "bsp_can.h"
#include "daemon.h"
#include "message_center.h"
#include "stm32h7xx.h"
#include "sys_timestamp.h"
#include "transport_config.h"
#include "transport_protocol.h"

namespace
{
// FDCAN RX ISR 写最新帧和实际接收时间；ControlTask 在 PRIMASK 临界区复制后解码。
volatile uint8_t pending_bytes[TransportProtocol::kPayloadSize];
volatile uint64_t pending_rx_us;
volatile bool pending;
bool initialized;
bool has_sequence;
uint8_t last_sequence;
uint64_t last_valid_rx_us;
Daemon transport_daemon{TransportProtocol::kCommandMaxAgeUs / 1000U};

// 两种消息独立保存接收状态与序号，底盘反馈不能覆盖遥控云台输入。
volatile uint8_t rate_bytes[TransportProtocol::kPayloadSize];
volatile uint64_t rate_rx_us;
volatile bool rate_pending;
bool rate_has_sequence;
uint8_t rate_last_sequence;
uint64_t rate_last_valid_rx_us;

// 与现有底盘反馈保持相同的实际接收时间、序号和链路活性语义。
void PollRate()
{
    uint8_t bytes[TransportProtocol::kPayloadSize];
    uint64_t received_us = 0U;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool available = rate_pending;
    if (available)
    {
        for (uint8_t index = 0U; index < TransportProtocol::kPayloadSize; ++index)
        {
            bytes[index] = rate_bytes[index];
        }
        received_us = rate_rx_us;
        rate_pending = false;
    }
    __DMB();
    __set_PRIMASK(primask);
    if (!available) { return; }
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (now_us < received_us || now_us - received_us > TransportProtocol::kCommandMaxAgeUs)
    {
        return;
    }
    GimbalRateInput input{};
    uint8_t sequence = 0U;
    if (!TransportProtocol::DecodeGimbalRateInput(bytes, sizeof(bytes), input, sequence))
    {
        return;
    }
    transport_daemon.Feed();
    if (rate_has_sequence && received_us - rate_last_valid_rx_us > TransportProtocol::kCommandMaxAgeUs)
    {
        rate_has_sequence = false;
    }
    rate_last_valid_rx_us = received_us;
    if (rate_has_sequence && !TransportProtocol::SequenceNewer(sequence, rate_last_sequence))
    {
        return;
    }
    MessageCenter::Remote_Gimbal_Input_Topic.PublishAt(input, received_us);
    rate_last_sequence = sequence;
    rate_has_sequence = true;
}

void Receive(FDCAN_HandleTypeDef* bus, uint32_t id, uint8_t* data,
             uint32_t size, void*)
{
    uint8_t type = 0U;
    if (bus != TransportConfig_Bus() || id != TransportConfig::kUplinkCanId ||
        !GimbalLink_GetMessageType(data, size, &type))
    {
        return;
    }
    // 同一0x80仅注册一个回调；按类型分开缓存，连续到达的两类帧不能互相覆盖。
    if (type == GIMBAL_LINK_RATE_INPUT)
    {
        for (uint8_t index = 0U; index < TransportProtocol::kPayloadSize; ++index)
        {
            rate_bytes[index] = data[index];
        }
        rate_rx_us = SYS_Timestamp_Get_Microsecond();
        rate_pending = true;
    }
    else if (type == GIMBAL_LINK_CHASSIS_FEEDBACK)
    {
        for (uint8_t index = 0U; index < TransportProtocol::kPayloadSize; ++index)
        {
            pending_bytes[index] = data[index];
        }
        pending_rx_us = SYS_Timestamp_Get_Microsecond();
        pending = true;
    }
}
}

FDCAN_HandleTypeDef *TransportConfig_Bus(void)
{
    return &hfdcan2;
}

bool BoardTransport_Init(void)
{
    if (!initialized)
    {
        initialized = BSP_CAN_RegisterCallback(TransportConfig::kUplinkCanId,
            TransportConfig_Bus(), Receive, nullptr) &&
            DaemonManager::Register(transport_daemon);
    }
    return initialized;
}

bool BoardTransport_IsOnline(void)
{
    return initialized && transport_daemon.IsOnline();
}

uint32_t BoardTransport_OfflineDurationMs(void)
{
    return transport_daemon.OfflineDurationMs();
}

void BoardTransport_Poll(void)
{
    if (!initialized) { return; }
    PollRate();
    uint8_t bytes[TransportProtocol::kPayloadSize];
    uint64_t received_us = 0U;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool available = pending;
    if (available)
    {
        for (uint8_t index = 0U; index < TransportProtocol::kPayloadSize; ++index)
        {
            bytes[index] = pending_bytes[index];
        }
        received_us = pending_rx_us;
        pending = false;
    }
    __DMB();
    __set_PRIMASK(primask);
    if (!available)
    {
        return;
    }
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (now_us < received_us ||
        now_us - received_us > TransportProtocol::kCommandMaxAgeUs)
    {
        return;
    }
    ChassisFeedback feedback{};
    uint8_t sequence = 0U;
    if (!TransportProtocol::DecodeChassisFeedback(bytes, sizeof(bytes),
                                                  feedback, sequence))
    {
        return;
    }
    // 合法重复帧只证明链路活性；连续数据流不中断时不能重建基准并刷新旧 Topic。
    transport_daemon.Feed();
    if (has_sequence && received_us - last_valid_rx_us >
                            TransportProtocol::kCommandMaxAgeUs)
    {
        has_sequence = false;
    }
    last_valid_rx_us = received_us;
    if (has_sequence && !TransportProtocol::SequenceNewer(sequence, last_sequence))
    {
        return;
    }
    MessageCenter::Chassis_Feedback_Topic.PublishAt(feedback, received_us);
    last_sequence = sequence;
    has_sequence = true;
}

void BoardTransport_SendChassis(const ChassisCmd &command)
{
    if (!initialized) { return; }
    static uint8_t sequence;
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = TransportConfig_Bus();
    message.id = TransportConfig::kChassisCmdCanId;
    message.len = TransportProtocol::kPayloadSize;
    if (!TransportProtocol::EncodeChassisCmd(command, sequence, message.data))
    {
        return;
    }
    if (CAN_Tx_Perform(&message))
    {
        ++sequence;
    }
}

bool BoardTransport_SendMouseCommand(uint8_t command)
{
    if (!initialized) { return false; }
    static uint8_t sequence;
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = TransportConfig_Bus();
    message.id = TransportConfig::kDownlinkCanId;
    message.len = TransportProtocol::kPayloadSize;
    if (!GimbalLink_EncodeMouseCommand(command, sequence, message.data) || !CAN_Tx_Submit(&message))
    {
        return false;
    }
    ++sequence;
    return true;
}

bool BoardTransport_SendGimbalYawState(void)
{
    INS_State ins{};
    if (!initialized || !MessageCenter::Gimbal_INS_State_Topic.ReadFresh(ins, 20000U))
    {
        return false;
    }
    const GimbalLink_YawState state{std::remainder(ins.yaw_rad, 6.283185307179586f), ins.gyro_z_rad_s};
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = TransportConfig_Bus();
    message.id = TransportConfig::kGimbalYawStateCanId;
    message.len = TransportProtocol::kPayloadSize;
    // 独立标识只保留最新版本，1kHz连续姿态不占用鼠标动作的逐帧发送队列。
    return GimbalLink_EncodeYawState(&state, message.data) && CAN_Tx_Perform(&message);
}
