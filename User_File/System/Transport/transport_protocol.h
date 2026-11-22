#ifndef BOARD_TRANSPORT_PROTOCOL_H
#define BOARD_TRANSPORT_PROTOCOL_H

#include "message_types.h"
#include "gimbal_link_protocol.h"

#include <stdint.h>

namespace TransportProtocol
{
constexpr uint8_t kVersion = GIMBAL_LINK_VERSION;
constexpr uint8_t kPayloadSize = GIMBAL_LINK_PAYLOAD_SIZE;
constexpr uint32_t kCommandMaxAgeUs = 100000U;
bool EncodeChassisCmd(const ChassisCmd &command, uint8_t sequence,
                      uint8_t (&bytes)[kPayloadSize]);
bool DecodeChassisCmd(const uint8_t *bytes, uint32_t size,
                      ChassisCmd &command, uint8_t &sequence);
bool EncodeChassisFeedback(const ChassisFeedback &feedback, uint8_t sequence,
                           uint8_t (&bytes)[kPayloadSize]);
bool DecodeChassisFeedback(const uint8_t *bytes, uint32_t size,
                           ChassisFeedback &feedback, uint8_t &sequence);
bool SequenceNewer(uint8_t candidate, uint8_t previous);
/** @brief 编码下板遥控输入；两轴分辨率 0.001 rad/s，返回 false 时不修改输出。 */
bool EncodeGimbalRateInput(const GimbalRateInput &input, uint8_t sequence,
                          uint8_t (&bytes)[kPayloadSize]);
/** @brief 解码固定版本与长度的遥控输入，保留字节须为零。 */
bool DecodeGimbalRateInput(const uint8_t *bytes, uint32_t size,
                          GimbalRateInput &input, uint8_t &sequence);
}

#endif
