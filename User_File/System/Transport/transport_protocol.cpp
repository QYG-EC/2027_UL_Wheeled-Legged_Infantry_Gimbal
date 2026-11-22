#include "transport_protocol.h"

namespace TransportProtocol
{
bool EncodeChassisCmd(const ChassisCmd& command, uint8_t sequence, uint8_t (&bytes)[kPayloadSize])
{
    const GimbalLink_ChassisCommand input{command.velocity_x_m_s, command.velocity_y_m_s,
                                          command.angular_velocity_rad_s, static_cast<uint8_t>(command.mode)};
    return GimbalLink_EncodeChassisCommand(&input, sequence, bytes);
}

bool DecodeChassisCmd(const uint8_t* bytes, uint32_t size, ChassisCmd& command, uint8_t& sequence)
{
    GimbalLink_ChassisCommand input{};
    if (!GimbalLink_DecodeChassisCommand(bytes, size, &input, &sequence))
    {
        return false;
    }
    command.mode = static_cast<ChassisMode>(input.mode);
    command.velocity_x_m_s = input.velocity_x_m_s;
    command.velocity_y_m_s = input.velocity_y_m_s;
    command.angular_velocity_rad_s = input.angular_velocity_rad_s;
    return true;
}

bool EncodeChassisFeedback(const ChassisFeedback& feedback, uint8_t sequence, uint8_t (&bytes)[kPayloadSize])
{
    const GimbalLink_ChassisFeedback input{feedback.velocity_x_m_s, feedback.velocity_y_m_s,
                                           feedback.angular_velocity_rad_s, feedback.enabled, feedback.online};
    return GimbalLink_EncodeChassisFeedback(&input, sequence, bytes);
}

bool DecodeChassisFeedback(const uint8_t* bytes, uint32_t size, ChassisFeedback& feedback, uint8_t& sequence)
{
    GimbalLink_ChassisFeedback input{};
    if (!GimbalLink_DecodeChassisFeedback(bytes, size, &input, &sequence))
    {
        return false;
    }
    feedback.velocity_x_m_s = input.velocity_x_m_s;
    feedback.velocity_y_m_s = input.velocity_y_m_s;
    feedback.angular_velocity_rad_s = input.angular_velocity_rad_s;
    feedback.enabled = input.enabled;
    feedback.online = input.online;
    return true;
}

bool SequenceNewer(uint8_t candidate, uint8_t previous)
{
    return GimbalLink_SequenceNewer(candidate, previous);
}

bool EncodeGimbalRateInput(const GimbalRateInput& input, uint8_t sequence, uint8_t (&bytes)[kPayloadSize])
{
    const GimbalLink_RateInput value{input.yaw_speed_rad_s, input.pitch_speed_rad_s, input.enabled, input.autoaim_requested};
    return GimbalLink_EncodeRateInput(&value, sequence, bytes);
}

bool DecodeGimbalRateInput(const uint8_t* bytes, uint32_t size, GimbalRateInput& input, uint8_t& sequence)
{
    GimbalLink_RateInput value{};
    if (!GimbalLink_DecodeRateInput(bytes, size, &value, &sequence))
    {
        return false;
    }
    input.yaw_speed_rad_s = value.yaw_speed_rad_s;
    input.pitch_speed_rad_s = value.pitch_speed_rad_s;
    input.enabled = value.enabled;
    input.autoaim_requested = value.autoaim_requested;
    return true;
}
} // namespace TransportProtocol
