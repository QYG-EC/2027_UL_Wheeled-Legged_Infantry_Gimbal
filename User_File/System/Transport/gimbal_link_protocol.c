#include "gimbal_link_protocol.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool EncodeScalar(float value, uint8_t* bytes)
{
    const float scaled = value * 1000.0f;
    if (!isfinite(scaled) || scaled < -32768.0f || scaled > 32767.0f)
    {
        return false;
    }
    const uint16_t bits = (uint16_t) (int16_t) lroundf(scaled);
    bytes[0] = (uint8_t) bits;
    bytes[1] = (uint8_t) (bits >> 8U);
    return true;
}

static float DecodeScalar(const uint8_t* bytes)
{
    const uint16_t bits = (uint16_t) bytes[0] | ((uint16_t) bytes[1] << 8U);
    const int32_t value = bits < 0x8000U ? (int32_t) bits : (int32_t) bits - 0x10000;
    return (float) value * 0.001f;
}

static uint8_t Header(uint8_t type, uint8_t flags)
{
    return (uint8_t) ((GIMBAL_LINK_VERSION << 4U) | (type << 2U) | flags);
}

bool GimbalLink_GetMessageType(const uint8_t* bytes, uint32_t size, uint8_t* type)
{
    if (bytes == NULL || type == NULL || size != GIMBAL_LINK_PAYLOAD_SIZE || (bytes[1] >> 4U) != GIMBAL_LINK_VERSION)
    {
        return false;
    }
    const uint8_t decoded = (bytes[1] >> 2U) & 3U;
    if (decoded > GIMBAL_LINK_MOUSE_COMMAND)
    {
        return false;
    }
    *type = decoded;
    return true;
}

static bool HasType(const uint8_t* bytes, uint32_t size, uint8_t expected)
{
    uint8_t type;
    return GimbalLink_GetMessageType(bytes, size, &type) && type == expected;
}

bool GimbalLink_EncodeRateInput(const GimbalLink_RateInput* input, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE])
{
    if (input == NULL || bytes == NULL)
    {
        return false;
    }
    uint8_t encoded[GIMBAL_LINK_PAYLOAD_SIZE] = {sequence, Header(GIMBAL_LINK_RATE_INPUT, (input->enabled ? 1U : 0U) | (input->autoaim_requested ? 2U : 0U))};
    if (!EncodeScalar(input->yaw_speed_rad_s, encoded + 2U) || !EncodeScalar(input->pitch_speed_rad_s, encoded + 4U))
    {
        return false;
    }
    memcpy(bytes, encoded, sizeof(encoded));
    return true;
}

bool GimbalLink_DecodeRateInput(const uint8_t* bytes, uint32_t size, GimbalLink_RateInput* input, uint8_t* sequence)
{
    if (input == NULL || sequence == NULL || !HasType(bytes, size, GIMBAL_LINK_RATE_INPUT) ||
        bytes[6] != 0U || bytes[7] != 0U)
    {
        return false;
    }
    GimbalLink_RateInput decoded;
    decoded.yaw_speed_rad_s = DecodeScalar(bytes + 2U);
    decoded.pitch_speed_rad_s = DecodeScalar(bytes + 4U);
    decoded.enabled = (bytes[1] & 1U) != 0U;
    decoded.autoaim_requested = (bytes[1] & 2U) != 0U;
    *input = decoded;
    *sequence = bytes[0];
    return true;
}

bool GimbalLink_EncodeChassisCommand(const GimbalLink_ChassisCommand* command, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE])
{
    if (command == NULL || bytes == NULL || command->mode > 3U)
    {
        return false;
    }
    uint8_t encoded[GIMBAL_LINK_PAYLOAD_SIZE] = {sequence, Header(GIMBAL_LINK_CHASSIS_COMMAND, command->mode)};
    if (!EncodeScalar(command->velocity_x_m_s, encoded + 2U) || !EncodeScalar(command->velocity_y_m_s, encoded + 4U) ||
        !EncodeScalar(command->angular_velocity_rad_s, encoded + 6U))
    {
        return false;
    }
    memcpy(bytes, encoded, sizeof(encoded));
    return true;
}

bool GimbalLink_DecodeChassisCommand(const uint8_t* bytes, uint32_t size, GimbalLink_ChassisCommand* command, uint8_t* sequence)
{
    if (command == NULL || sequence == NULL || !HasType(bytes, size, GIMBAL_LINK_CHASSIS_COMMAND))
    {
        return false;
    }
    GimbalLink_ChassisCommand decoded;
    decoded.mode = bytes[1] & 3U;
    decoded.velocity_x_m_s = DecodeScalar(bytes + 2U);
    decoded.velocity_y_m_s = DecodeScalar(bytes + 4U);
    decoded.angular_velocity_rad_s = DecodeScalar(bytes + 6U);
    *command = decoded;
    *sequence = bytes[0];
    return true;
}

bool GimbalLink_EncodeChassisFeedback(const GimbalLink_ChassisFeedback* feedback, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE])
{
    if (feedback == NULL || bytes == NULL)
    {
        return false;
    }
    const uint8_t flags = (feedback->enabled ? 1U : 0U) | (feedback->online ? 2U : 0U);
    uint8_t encoded[GIMBAL_LINK_PAYLOAD_SIZE] = {sequence, Header(GIMBAL_LINK_CHASSIS_FEEDBACK, flags)};
    if (!EncodeScalar(feedback->velocity_x_m_s, encoded + 2U) || !EncodeScalar(feedback->velocity_y_m_s, encoded + 4U) ||
        !EncodeScalar(feedback->angular_velocity_rad_s, encoded + 6U))
    {
        return false;
    }
    memcpy(bytes, encoded, sizeof(encoded));
    return true;
}

bool GimbalLink_DecodeChassisFeedback(const uint8_t* bytes, uint32_t size, GimbalLink_ChassisFeedback* feedback, uint8_t* sequence)
{
    if (feedback == NULL || sequence == NULL || !HasType(bytes, size, GIMBAL_LINK_CHASSIS_FEEDBACK))
    {
        return false;
    }
    GimbalLink_ChassisFeedback decoded;
    decoded.enabled = (bytes[1] & 1U) != 0U;
    decoded.online = (bytes[1] & 2U) != 0U;
    decoded.velocity_x_m_s = DecodeScalar(bytes + 2U);
    decoded.velocity_y_m_s = DecodeScalar(bytes + 4U);
    decoded.angular_velocity_rad_s = DecodeScalar(bytes + 6U);
    *feedback = decoded;
    *sequence = bytes[0];
    return true;
}

bool GimbalLink_SequenceNewer(uint8_t candidate, uint8_t previous)
{
    const uint8_t distance = (uint8_t) (candidate - previous);
    return distance != 0U && distance < 128U;
}

bool GimbalLink_EncodeMouseCommand(uint8_t command, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE])
{
    if (bytes == NULL || command > 2U) { return false; }
    const uint8_t encoded[GIMBAL_LINK_PAYLOAD_SIZE] = {sequence, Header(GIMBAL_LINK_MOUSE_COMMAND, 0U), command};
    memcpy(bytes, encoded, sizeof(encoded));
    return true;
}

bool GimbalLink_DecodeMouseCommand(const uint8_t* bytes, uint32_t size, uint8_t* command, uint8_t* sequence)
{
    if (command == NULL || sequence == NULL || !HasType(bytes, size, GIMBAL_LINK_MOUSE_COMMAND) ||
        bytes[1] != Header(GIMBAL_LINK_MOUSE_COMMAND, 0U) || bytes[2] > 2U ||
        bytes[3] != 0U || bytes[4] != 0U || bytes[5] != 0U || bytes[6] != 0U || bytes[7] != 0U)
    {
        return false;
    }
    *command = bytes[2];
    *sequence = bytes[0];
    return true;
}

// 按位复制浮点表示，再显式排列字节，避免对接收缓冲进行未对齐访问。
static void EncodeFloat(float value, uint8_t* bytes)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        bytes[index] = (uint8_t) (bits >> (8U * index));
    }
}

static float DecodeFloat(const uint8_t* bytes)
{
    uint32_t bits = 0U;
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        bits |= (uint32_t) bytes[index] << (8U * index);
    }
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool GimbalLink_EncodeYawState(const GimbalLink_YawState* state, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE])
{
    if (state == NULL || bytes == NULL || !isfinite(state->yaw_rad) || !isfinite(state->yaw_speed_rad_s))
    {
        return false;
    }
    EncodeFloat(state->yaw_rad, bytes);
    EncodeFloat(state->yaw_speed_rad_s, bytes + 4U);
    return true;
}

bool GimbalLink_DecodeYawState(const uint8_t* bytes, uint32_t size, GimbalLink_YawState* state)
{
    if (bytes == NULL || state == NULL || size != GIMBAL_LINK_PAYLOAD_SIZE)
    {
        return false;
    }
    const GimbalLink_YawState decoded = {DecodeFloat(bytes), DecodeFloat(bytes + 4U)};
    if (!isfinite(decoded.yaw_rad) || !isfinite(decoded.yaw_speed_rad_s))
    {
        return false;
    }
    *state = decoded;
    return true;
}
