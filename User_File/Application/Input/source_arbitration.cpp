#include "source_arbitration.h"

#include <cmath>

namespace
{
constexpr uint32_t REMOTE_MAX_AGE_MS = INPUT_REMOTE_MAX_AGE_MS;
constexpr uint32_t OTHER_MAX_AGE_MS = INPUT_KEYBOARD_MAX_AGE_MS;

bool Fresh(bool valid, uint32_t timestamp, uint32_t now, uint32_t max_age)
{
    return valid && now - timestamp <= max_age;
}

bool AtOrAfter(uint32_t timestamp, uint32_t boundary)
{
    // 用无符号差处理毫秒计数回绕；两时刻间隔须小于半个 uint32_t 计数周期。
    return timestamp - boundary < 0x80000000U;
}

bool CommandValid(const ControlInput &input)
{
    // 在输入边界统一校验枚举和数值，后续机构控制使用已通过仲裁的命令。
    const ChassisCmd &c = input.chassis;
    const GimbalCmd &g = input.gimbal;
    const ShootCmd &s = input.shoot;
    return (c.mode == ChassisMode::ZERO_FORCE || c.mode == ChassisMode::NO_FOLLOW ||
            c.mode == ChassisMode::FOLLOW_GIMBAL_YAW || c.mode == ChassisMode::ROTATE) &&
           std::isfinite(c.velocity_x_m_s) && std::fabs(c.velocity_x_m_s) <= INPUT_MAX_TRANSLATION_M_S &&
           std::isfinite(c.velocity_y_m_s) && std::fabs(c.velocity_y_m_s) <= INPUT_MAX_TRANSLATION_M_S &&
           std::isfinite(c.angular_velocity_rad_s) &&
           std::fabs(c.angular_velocity_rad_s) <= INPUT_MAX_ROTATION_RAD_S &&
           (g.mode == GimbalMode::DISABLED || g.mode == GimbalMode::IMU ||
            g.mode == GimbalMode::LOCK || g.mode == GimbalMode::RATE) &&
           std::isfinite(g.yaw_angle_rad) && std::isfinite(g.pitch_angle_rad) &&
           std::isfinite(g.yaw_speed_rad_s) && std::isfinite(g.pitch_speed_rad_s) &&
           (s.shoot_mode == ShootMode::OFF || s.shoot_mode == ShootMode::ON) &&
           (s.friction_mode == FrictionMode::OFF || s.friction_mode == FrictionMode::ON) &&
           (s.loader_mode == LoaderMode::STOP || s.loader_mode == LoaderMode::REVERSE ||
            s.loader_mode == LoaderMode::BURST) &&
           std::isfinite(s.friction_speed_rad_s) &&
           std::isfinite(s.loader_speed_rad_s) && std::isfinite(s.shoot_rate_hz);
}
}

bool SourceArbitration_VisionReady(const InputState& state)
{
    const auto& vision = state.vision;
    return vision.valid &&
           (state.vision_max_age_ms == 0U || state.now_ms - vision.received_ms <= state.vision_max_age_ms) &&
           AtOrAfter(vision.received_ms, state.selected_at_ms) &&
           std::isfinite(vision.yaw_angle_rad) && std::isfinite(vision.pitch_angle_rad);
}

InputDecision SourceArbitration_Resolve(const InputState &state)
{
    // 默认结果不授予控制许可；遥控或选中控制来源校验失败时返回该安全结果。
    InputDecision decision{};
    if (state.gimbal_rate_sources)
    {
        // 控制权由总开关决定；键鼠健康与否均不能让遥控抢占该控制权。
        decision.source = state.keyboard_enabled ? InputSource::Keyboard : InputSource::Remote;
        const ControlInput& input = state.keyboard_enabled ? state.keyboard : state.remote;
        if (Fresh(input.valid, input.received_ms, state.now_ms,
                  state.keyboard_enabled ? OTHER_MAX_AGE_MS : REMOTE_MAX_AGE_MS) && CommandValid(input))
        {
            decision.gimbal = input.gimbal;
            decision.shoot = input.shoot;
            decision.armed = input.gimbal.mode != GimbalMode::DISABLED;
        }
        else if (state.keyboard_enabled)
        {
            // RATE的零速度保留已有姿态目标，避免LOCK边沿重新捕获实测姿态。
            decision.gimbal.mode = GimbalMode::RATE;
            decision.armed = true;
        }
        if (decision.armed && state.vision_enabled && SourceArbitration_VisionReady(state))
        {
            decision.gimbal.mode = GimbalMode::IMU;
            decision.gimbal.yaw_angle_rad = state.vision.yaw_angle_rad;
            decision.gimbal.pitch_angle_rad = state.vision.pitch_angle_rad;
            decision.gimbal.yaw_speed_rad_s = 0.0f;
            decision.gimbal.pitch_speed_rad_s = 0.0f;
        }
        // 无目标或过期保留刚算出的手动结果，不恢复到旧的自瞄目标。
        return decision;
    }
    decision.source = state.selected;
    /* Remote 失联始终停机，VTM/键鼠不能绕过 Remote 安全许可。 */
    if (!Fresh(state.remote.valid, state.remote.received_ms,
               state.now_ms, REMOTE_MAX_AGE_MS) || !CommandValid(state.remote))
    {
        return decision;
    }

    const ControlInput *selected = &state.remote;
    if (state.selected == InputSource::Vtm)
    {
        selected = &state.vtm;
    }
    else if (state.selected == InputSource::Keyboard)
    {
        selected = &state.keyboard;
    }
    else if (state.selected != InputSource::Remote)
    {
        return decision;
    }

    // 命令时间戳须不早于来源切换时刻；过期或无效时不自动回退到其他来源。
    if (!Fresh(selected->valid, selected->received_ms, state.now_ms,
               selected == &state.remote ? REMOTE_MAX_AGE_MS : OTHER_MAX_AGE_MS) ||
        !AtOrAfter(selected->received_ms, state.selected_at_ms) || !CommandValid(*selected))
    {
        return decision;
    }

    decision.chassis = selected->chassis;
    decision.gimbal = selected->gimbal;
    decision.shoot = selected->shoot;
    decision.armed = true;
    if (state.vision_enabled && decision.gimbal.mode != GimbalMode::DISABLED)
    {
        const VisionAimInput &vision = state.vision;
        if (Fresh(vision.valid, vision.received_ms, state.now_ms, OTHER_MAX_AGE_MS) &&
            AtOrAfter(vision.received_ms, state.selected_at_ms) &&
            std::isfinite(vision.yaw_angle_rad) && std::isfinite(vision.pitch_angle_rad))
        {
            decision.gimbal.mode = GimbalMode::IMU;
            decision.gimbal.yaw_angle_rad = vision.yaw_angle_rad;
            decision.gimbal.pitch_angle_rad = vision.pitch_angle_rad;
            decision.gimbal.yaw_speed_rad_s = 0.0f;
            decision.gimbal.pitch_speed_rad_s = 0.0f;
        }
        else
        {
            decision.gimbal = {};
            decision.gimbal.mode = GimbalMode::LOCK;
        }
    }
    return decision;
}
