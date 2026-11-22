#include "gimbal_input.h"

#include "Gimbal_Config.h"
#include "input_state.h"
#include "message_center.h"
#include "referee_26.h"
#include "vision.h"
#include "Shoot.h"
#include "Shoot_Config.h"
#include "board_transport.h"

namespace
{
Struct_Gimbal_Input_Config config;
bool keyboard_enabled;
bool keyboard_autoaim_enabled;
bool combo_pressed;
bool friction_enabled;
bool f_pressed;
bool right_pressed;
uint32_t right_started_ms;
referee_keyboard_snapshot_t keyboard_cache;
bool keyboard_new;
bool left_pressed;
bool left_accepted;
bool left_long;
uint32_t left_started_ms;
uint8_t mouse_command;
uint32_t mouse_sent_ms;
constexpr uint32_t BUTTON_HOLD_MS = 300U;
constexpr uint32_t MOUSE_SEND_PERIOD_MS = 10U;

// 取消后必须观察松开，避免切源或升速完成时触发此前的按压。
void CancelLeft()
{
    left_accepted = false;
    left_long = false;
}
uint32_t keyboard_sequence;
uint32_t vision_last_send_ms;
uint32_t vision_ins_sequence;
Struct_Vision_Feedback vision_feedback;
Struct_Gimbal_Config gimbal_config;

// 反馈使用同份惯性姿态；重复快照不刷新采样时间。
void UpdateVisionFeedback(uint32_t now_ms)
{
    if (now_ms - vision_last_send_ms < config.vision_feedback_period_ms)
    {
        return;
    }
    vision_last_send_ms = now_ms;
    const auto sample = MessageCenter::Gimbal_INS_State_Topic.ReadWithMeta();
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (!sample.valid || now_us < sample.timestamp_us || (config.vision_feedback_max_age_ms != 0U && now_us - sample.timestamp_us > static_cast<uint64_t>(config.vision_feedback_max_age_ms) * 1000U))
    {
        return;
    }
    if (sample.sequence != vision_ins_sequence)
    {
        vision_ins_sequence = sample.sequence;
        const auto& ins = sample.data;
        const float angles[] = {ins.roll_rad, ins.pitch_rad, ins.yaw_rad};
        const auto& yaw = gimbal_config.yaw;
        const auto& pitch = gimbal_config.pitch;
        vision_feedback.yaw_rad = yaw.angle_sign * angles[static_cast<unsigned>(yaw.angle_axis)];
        vision_feedback.pitch_rad = pitch.angle_sign * angles[static_cast<unsigned>(pitch.angle_axis)];
        // Pitch按轴配置取负传感器Pitch，横滚独立保留传感器Roll。
        vision_feedback.roll_rad = ins.roll_rad;
        // 按实际帧年龄投影到HAL毫秒时基；缓存后重复发送不重新盖时戳。
        vision_feedback.sampled_ms = now_ms - static_cast<uint32_t>((now_us - sample.timestamp_us) / 1000U);
    }
    uint16_t robot_id = 0U;
    const bool identity_known = RefereeReadRobotId(&robot_id) != 0U;
    vision_feedback.mode = identity_known && robot_id >= 1U && robot_id <= 11U ? 1U : 0U;
    vision_feedback.bullet_speed_m_s = config.vision_bullet_speed_m_s;
    (void) Vision_SendFeedback(vision_feedback);
}
} // namespace

void GimbalInput_Init(const Struct_Gimbal_Input_Config& requested)
{
    config = requested;
    friction_enabled = false;
    f_pressed = false;
    right_pressed = false;
    right_started_ms = 0U;
    keyboard_cache = {};
    keyboard_new = false;
    left_pressed = false;
    left_accepted = false;
    left_long = false;
    left_started_ms = 0U;
    mouse_command = 0U;
    mouse_sent_ms = HAL_GetTick();
    keyboard_enabled = false;
    keyboard_autoaim_enabled = false;
    combo_pressed = false;
    keyboard_sequence = 0U;
    vision_last_send_ms = HAL_GetTick();
    vision_ins_sequence = 0U;
    vision_feedback = {};
    gimbal_config = Gimbal_Default_Config();
    InputState_Reset();
    InputState_EnableGimbalRateSources(config.vision_target_max_age_ms);
    InputState_SetTime(HAL_GetTick());
}

void GimbalInput_Update()
{
    const uint32_t now_ms = HAL_GetTick();
    InputState_SetTime(now_ms);
    const auto remote = MessageCenter::Remote_Gimbal_Input_Topic.ReadWithMeta();
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    ControlInput remote_input{};
    if (remote.valid && now_us >= remote.timestamp_us)
    {
        remote_input.gimbal.mode = remote.data.enabled ? GimbalMode::RATE : GimbalMode::DISABLED;
        remote_input.gimbal.yaw_speed_rad_s = remote.data.yaw_speed_rad_s;
        remote_input.gimbal.pitch_speed_rad_s = remote.data.pitch_speed_rad_s;
        // 微秒时钟和HAL tick起点不同，按本地帧年龄投影到HAL时基，不能直接除以1000。
        remote_input.received_ms = now_ms - static_cast<uint32_t>((now_us - remote.timestamp_us) / 1000U);
        remote_input.valid = true;
    }
    InputState_SubmitRemote(remote_input);

    keyboard_new = false;
    if (!keyboard_cache.received || now_ms - keyboard_cache.received_ms > INPUT_KEYBOARD_MAX_AGE_MS)
    {
        friction_enabled = false;
        keyboard_autoaim_enabled = false;
        right_pressed = false;
        CancelLeft();
    }
    referee_keyboard_snapshot_t keyboard{};
    if (RefereeReadKeyboardSnapshot(&keyboard) && keyboard.sequence != keyboard_sequence)
    {
        keyboard_sequence = keyboard.sequence;
        keyboard_cache = keyboard;
        keyboard_new = true;
        constexpr uint16_t CTRL_S = (1U << 5U) | (1U << 1U);
        const bool pressed = (keyboard.keyboard & CTRL_S) == CTRL_S;
        if (now_ms - keyboard.received_ms <= INPUT_KEYBOARD_MAX_AGE_MS)
        {
            if (pressed && !combo_pressed)
            {
                keyboard_enabled = !keyboard_enabled;
            }
            combo_pressed = pressed;
        }
        const bool fresh = now_ms - keyboard.received_ms <= INPUT_KEYBOARD_MAX_AGE_MS;
        const bool f_down = (keyboard.keyboard & (1U << 9U)) != 0U;
        if (fresh && keyboard_enabled)
        {
            if (f_down && !f_pressed) { friction_enabled = !friction_enabled; }
            const bool right_down = keyboard.mouse_right != 0U;
            if (right_down && !right_pressed) { right_started_ms = keyboard.received_ms; }
            right_pressed = right_down;
            keyboard_autoaim_enabled = right_down && keyboard.received_ms - right_started_ms >= BUTTON_HOLD_MS;
        }
        else
        {
            friction_enabled = false;
            keyboard_autoaim_enabled = false;
            right_pressed = false;
        }
        f_pressed = f_down;
        ControlInput input{};
        input.gimbal.mode = GimbalMode::RATE;
        input.gimbal.pitch_speed_rad_s = keyboard.mouse_y * config.pitch_rad_per_count / config.mouse_frame_period_s;
        input.gimbal.yaw_speed_rad_s = -keyboard.mouse_x * config.yaw_rad_per_count / config.mouse_frame_period_s;
        input.shoot.shoot_mode = friction_enabled ? ShootMode::ON : ShootMode::OFF;
        input.shoot.friction_mode = friction_enabled ? FrictionMode::ON : FrictionMode::OFF;
        input.shoot.friction_speed_rad_s = kShootConfig.default_friction_speed_rad_s;
        input.received_ms = keyboard.received_ms;
        input.valid = true;
        InputState_SubmitKeyboard(input);
    }
    InputState_SetKeyboardEnabled(keyboard_enabled);
    // CH7只在遥控所有权、CH8许可和原有遥控时效均成立时生效。
    const bool remote_autoaim = remote_input.valid && remote.data.enabled && remote.data.autoaim_requested &&
                                now_ms - remote_input.received_ms <= INPUT_REMOTE_MAX_AGE_MS;
    InputState_Select(keyboard_enabled ? InputSource::Keyboard : InputSource::Remote,
                      keyboard_enabled ? keyboard_autoaim_enabled : remote_autoaim, now_ms);
    Struct_Vision_Reference reference;
    VisionAimInput aim{};
    if (Vision_ReadLatest(reference))
    {
        aim.yaw_angle_rad = reference.yaw_rad;
        aim.pitch_angle_rad = reference.pitch_rad;
        aim.received_ms = reference.received_ms;
        aim.valid = reference.valid;
    }
    InputState_SubmitVision(aim);
    UpdateVisionFeedback(now_ms);
}

void GimbalInput_SendMouseCommand()
{
    const uint32_t now_ms = HAL_GetTick();
    const bool active = keyboard_enabled && keyboard_cache.received &&
                        now_ms - keyboard_cache.received_ms <= INPUT_KEYBOARD_MAX_AGE_MS &&
                        friction_enabled && !keyboard_autoaim_enabled;
    const bool ready = active && Shoot_FrictionReady();
    uint8_t next = 0U;
    if (!active) { CancelLeft(); }
    if (keyboard_new)
    {
        const bool down = keyboard_cache.mouse_left != 0U;
        if (down && !left_pressed)
        {
            left_accepted = ready;
            left_started_ms = keyboard_cache.received_ms;
            left_long = false;
        }
        if (down && left_accepted &&
            keyboard_cache.received_ms - left_started_ms >= BUTTON_HOLD_MS)
        {
            left_long = true;
        }
        if (!down)
        {
            if (left_pressed && left_accepted && !left_long && ready &&
                keyboard_cache.received_ms - left_started_ms < BUTTON_HOLD_MS)
            {
                next = 1U;
            }
            CancelLeft();
        }
        left_pressed = down;
    }
    if (ready && left_accepted && left_long) { next = 2U; }
    // 单次事件只提交一次；0/2在切换时及每10ms提交最新状态。
    if (next == 1U || next != mouse_command || now_ms - mouse_sent_ms >= MOUSE_SEND_PERIOD_MS)
    {
        (void) BoardTransport_SendMouseCommand(next);
        mouse_sent_ms = now_ms;
    }
    mouse_command = next == 1U ? 0U : next;
}
