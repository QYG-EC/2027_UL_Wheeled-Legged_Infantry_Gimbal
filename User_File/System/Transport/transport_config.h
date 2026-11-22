#ifndef BOARD_TRANSPORT_CONFIG_H
#define BOARD_TRANSPORT_CONFIG_H

#include "fdcan.h"
#include "transport_protocol.h"

namespace TransportConfig
{
// 方向决定CAN标识，数据域类型区分同方向消息；板间总线由板型固定绑定。
constexpr uint16_t kUplinkCanId = GIMBAL_LINK_UPLINK_ID;
constexpr uint16_t kDownlinkCanId = GIMBAL_LINK_DOWNLINK_ID;
constexpr uint16_t kChassisCmdCanId = kDownlinkCanId;
constexpr uint16_t kChassisFeedbackCanId = kUplinkCanId;
constexpr uint16_t kGimbalRateInputCanId = kUplinkCanId;
}

/** One fixed CAN link per firmware target; no runtime routing table. */
FDCAN_HandleTypeDef *TransportConfig_Bus(void);

#endif
