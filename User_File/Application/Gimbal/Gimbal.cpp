#include "Gimbal.h"

#include "dmmotor.h"
#include "message_center.h"
#include <cmath>

namespace
{
constexpr uint64_t INS_MAX_AGE_US = 20000U; // 用户确认的惯性快照时效，20ms。
constexpr float CONTROL_PERIOD_S = 0.001f;
constexpr float TWO_PI = 6.283185307179586f;
constexpr float STATIC_FRICTION_RANGE_RAD = 0.03490658503988659f; // 与Pitch实验相同的2°连续区间。
struct AxisContext
{
    Class_DMMotor motor;
    Struct_DMMotor_Snapshot snapshot{};
    bool registered = false;
    bool was_ready = false;
    GimbalMode last_mode = GimbalMode::DISABLED;
    float target_angle_rad = 0.0f;
    float target_speed_rad_s = 0.0f;
};
struct GimbalContext
{
    Struct_Gimbal_Config config{};
    AxisContext yaw;
    AxisContext pitch;
    INS_State ins{};
    GimbalCmd command{};
    bool initialized = false;
    bool ins_valid = false;
    uint8_t feedback_divider = 0U;
};
GimbalContext ctx;

float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool AxisConfigValid(const Struct_Gimbal_Axis_Config& config)
{
    const float nonnegative[] = {config.k_angle, config.k_speed, config.moving_friction, config.static_friction};
    for (float value : nonnegative)
    {
        if (!std::isfinite(value) || value < 0.0f)
        {
            return false;
        }
    }
    return config.angle_axis <= GimbalAngleAxis::Yaw && config.gyro_axis <= GimbalGyroAxis::Z &&
           (config.angle_sign == 1.0f || config.angle_sign == -1.0f) &&
           (config.gyro_sign == 1.0f || config.gyro_sign == -1.0f) &&
           (config.torque_sign == 1.0f || config.torque_sign == -1.0f) &&
           std::isfinite(config.motor_per_axis) && config.motor_per_axis != 0.0f &&
           std::isfinite(config.gravity_constant) && std::isfinite(config.gravity_linear) &&
           std::isfinite(config.torque_limit) && config.torque_limit > 0.0f &&
           config.torque_limit <= config.motor.torque_max;
}

float Angle(const Struct_Gimbal_Axis_Config& config)
{
    return Gimbal_AxisAngle(config, ctx.ins);
}

float Speed(const AxisContext& axis, const Struct_Gimbal_Axis_Config& config)
{
    return axis.snapshot.feedback.velocity / config.motor_per_axis;
}

bool InitAxis(AxisContext& axis, const Struct_Gimbal_Axis_Config& config)
{
    const auto& motor = config.motor;
    axis.motor.SetFeedbackStateCodes(motor.disabled_state, motor.enabled_state);
    axis.registered = axis.motor.Init(motor.bus, motor.id, motor.feedback_id, Enum_DMMotor_Mode::MIT,
                                      motor.reverse, motor.position_max, motor.velocity_max, motor.torque_max);
    return axis.registered;
}

// 控制目标属于Gimbal；同一速度消息在每个1ms周期积分，而不是只在新消息时积分一次。
void UpdateTarget(AxisContext& axis, float measured, float angle, float speed,
                  bool yaw)
{
    if (!axis.snapshot.ready || !axis.was_ready ||
        (ctx.command.mode == GimbalMode::LOCK && axis.last_mode != GimbalMode::LOCK))
    {
        axis.target_angle_rad = measured;
        axis.target_speed_rad_s = 0.0f;
    }
    else if (ctx.command.mode == GimbalMode::RATE)
    {
        if (axis.last_mode == GimbalMode::IMU)
        {
            // 自瞄退出从实测姿态接回手动，不继续追赶最后视觉目标。
            axis.target_angle_rad = measured;
        }
        axis.target_angle_rad += speed * CONTROL_PERIOD_S;
        if (yaw)
        {
            axis.target_angle_rad = std::remainder(axis.target_angle_rad, TWO_PI);
        }
        axis.target_speed_rad_s = speed;
    }
    // 已就绪时每周期应用绝对目标，同值目标及恢复后的目标不依赖消息序号。
    if (axis.snapshot.ready && ctx.command.mode == GimbalMode::IMU)
    {
        axis.target_angle_rad = angle;
        axis.target_speed_rad_s = speed;
    }
    axis.was_ready = axis.snapshot.ready;
    axis.last_mode = ctx.command.mode;
}

// 两轴位置取惯性单元，速度取各自电机反馈并按传动比映射到应用坐标。
void ControlAxis(AxisContext& axis, const Struct_Gimbal_Axis_Config& config, bool yaw)
{
    float error = axis.target_angle_rad - Angle(config);
    if (yaw)
    {
        error = std::remainder(error, TWO_PI);
    }
    float motor_target = axis.snapshot.feedback.position + config.motor_per_axis * error;
    if (!yaw)
    {
        const float limited_target = Clamp(motor_target, ctx.config.pitch_min, ctx.config.pitch_max);
        if (limited_target != motor_target)
        {
            // 将裁剪结果写回积分目标，避免持续向外操作后需要长时间反向“退积分”。
            axis.target_angle_rad = Angle(config) +
                                    (limited_target - axis.snapshot.feedback.position) / config.motor_per_axis;
        }
        motor_target = limited_target;
        error = (motor_target - axis.snapshot.feedback.position) / config.motor_per_axis;
        const float reference_speed = config.motor_per_axis * axis.target_speed_rad_s;
        if ((motor_target >= ctx.config.pitch_max && reference_speed > 0.0f) ||
            (motor_target <= ctx.config.pitch_min && reference_speed < 0.0f))
        {
            // 位置目标停在边界时，速度反馈项和运动摩擦前馈也停止继续向外推动。
            axis.target_speed_rad_s = 0.0f;
        }
    }
    const float motor_reference_speed = config.motor_per_axis * axis.target_speed_rad_s;
    const float direction = motor_reference_speed > 0.0f ? 1.0f : (motor_reference_speed < 0.0f ? -1.0f : 0.0f);
    const float feedback_torque = config.torque_sign *
                                  (config.k_angle * error + config.k_speed * (axis.target_speed_rad_s - Speed(axis, config)));
    // 静止参考复用实验到位辅助公式；运动时不保留静态方向力矩。
    const float static_friction = !yaw && axis.target_speed_rad_s == 0.0f ?
        config.torque_sign * config.static_friction * Clamp(error / STATIC_FRICTION_RANGE_RAD, -1.0f, 1.0f) : 0.0f;
    const float feedforward = config.gravity_constant + config.gravity_linear * motor_target +
                              config.moving_friction * direction;
    (void) axis.motor.SetTorque(Clamp(feedback_torque + feedforward + static_friction, -config.torque_limit, config.torque_limit));
}

void UpdateAxis(AxisContext& axis, const Struct_Gimbal_Axis_Config& config,
                bool yaw)
{
    const bool enabled = ctx.initialized && config.control_enabled && ctx.ins_valid &&
                         ctx.command.mode != GimbalMode::DISABLED;
    if (axis.registered)
    {
        (void) axis.motor.RequestEnabled(enabled);
    }
    if (enabled)
    {
        float speed = yaw ? ctx.command.yaw_speed_rad_s : ctx.command.pitch_speed_rad_s;
        if (!yaw)
        {
            const float motor_speed = config.motor_per_axis * speed;
            if ((axis.snapshot.feedback.position >= ctx.config.pitch_max && motor_speed > 0.0f) ||
                (axis.snapshot.feedback.position <= ctx.config.pitch_min && motor_speed < 0.0f))
            {
                // 实测到达机械坐标限位时禁止向外积分；向内指令直接通过，不锁存、不失能。
                speed = 0.0f;
            }
        }
        UpdateTarget(axis, Angle(config), yaw ? ctx.command.yaw_angle_rad : ctx.command.pitch_angle_rad,
                     speed, yaw);
        // 先请求使能，再等待驱动ready；两轴分别捕获/恢复，未配置Yaw不阻塞Pitch。
        if (axis.snapshot.ready)
        {
            ControlAxis(axis, config, yaw);
        }
    }
    else
    {
        axis.was_ready = false;
        axis.last_mode = GimbalMode::DISABLED;
    }
}

void PublishFeedback()
{
    if (++ctx.feedback_divider < 10U)
    {
        return;
    }
    ctx.feedback_divider = 0U;
    GimbalFeedback feedback{};
    if (ctx.ins_valid)
    {
        feedback.yaw_rad = Angle(ctx.config.yaw);
        feedback.pitch_rad = Angle(ctx.config.pitch);
        feedback.yaw_speed_rad_s = Speed(ctx.yaw, ctx.config.yaw);
        feedback.pitch_speed_rad_s = Speed(ctx.pitch, ctx.config.pitch);
    }
    feedback.ins_valid = ctx.ins_valid;
    feedback.yaw_ready = ctx.yaw.was_ready;
    feedback.pitch_ready = ctx.pitch.was_ready;
    feedback.enabled = (ctx.config.yaw.control_enabled || ctx.config.pitch.control_enabled) &&
                       (!ctx.config.yaw.control_enabled || ctx.yaw.was_ready) &&
                       (!ctx.config.pitch.control_enabled || ctx.pitch.was_ready);
    MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
}
} // namespace

bool Gimbal_Init(const Struct_Gimbal_Config& config)
{
    if (!AxisConfigValid(config.yaw) || !AxisConfigValid(config.pitch) ||
        !std::isfinite(config.pitch_min) || !std::isfinite(config.pitch_max) || config.pitch_min >= config.pitch_max ||
        (config.yaw.motor.bus == config.pitch.motor.bus &&
         (config.yaw.motor.id == config.pitch.motor.id || config.yaw.motor.feedback_id == config.pitch.motor.feedback_id)))
    {
        return false;
    }
    ctx.config = config;
    const bool yaw = InitAxis(ctx.yaw, config.yaw);
    const bool pitch = InitAxis(ctx.pitch, config.pitch);
    ctx.initialized = (!config.yaw.control_enabled || yaw) && (!config.pitch.control_enabled || pitch);
    return ctx.initialized;
}

Enum_Gimbal_Status Gimbal_GetStatus()
{
    if (!ctx.initialized)
    {
        return Gimbal_Status_CONFIG_ERROR;
    }
    if (ctx.command.mode == GimbalMode::DISABLED ||
        (!ctx.config.yaw.control_enabled && !ctx.config.pitch.control_enabled))
    {
        return Gimbal_Status_DISABLE;
    }
    if (!ctx.ins_valid || (ctx.config.yaw.control_enabled && ctx.yaw.snapshot.fault) ||
        (ctx.config.pitch.control_enabled && ctx.pitch.snapshot.fault))
    {
        return Gimbal_Status_FAULT;
    }
    return (!ctx.config.yaw.control_enabled || ctx.yaw.was_ready) &&
                   (!ctx.config.pitch.control_enabled || ctx.pitch.was_ready)
               ? Gimbal_Status_READY
               : Gimbal_Status_ENABLING;
}

void Gimbal_Update()
{
    auto& ins_topic = ctx.config.external_imu ? MessageCenter::Gimbal_INS_State_Topic : MessageCenter::INS_State_Topic;
    ctx.ins_valid = ins_topic.ReadFresh(ctx.ins, INS_MAX_AGE_US);
    const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.command = message.valid ? message.data : GimbalCmd{};
    ctx.yaw.snapshot = ctx.yaw.motor.GetFeedbackSnapshot();
    ctx.pitch.snapshot = ctx.pitch.motor.GetFeedbackSnapshot();
    UpdateAxis(ctx.yaw, ctx.config.yaw, true);
    UpdateAxis(ctx.pitch, ctx.config.pitch, false);
    PublishFeedback();
}

bool Gimbal_SetTuningParameters(const float (&values)[10])
{
    if (!ctx.initialized)
    {
        return false;
    }
    for (float value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    for (unsigned offset = 0U; offset < 10U; offset += 5U)
    {
        if (values[offset] < 0.0f || values[offset + 1U] < 0.0f || values[offset + 4U] < 0.0f)
        {
            return false;
        }
    }
    // 全组校验后再写入；与控制计算同任务，保留模式、姿态目标及已有输出约束。
    Struct_Gimbal_Axis_Config* axes[] = {&ctx.config.yaw, &ctx.config.pitch};
    for (unsigned index = 0U; index < 2U; ++index)
    {
        auto& axis = *axes[index];
        const float* source = values + 5U * index;
        axis.k_angle = source[0];
        axis.k_speed = source[1];
        axis.gravity_constant = source[2];
        axis.gravity_linear = source[3];
        axis.moving_friction = source[4];
    }
    return true;
}

bool Gimbal_GetTuningParameters(float (&values)[10])
{
    const Struct_Gimbal_Axis_Config* axes[] = {&ctx.config.yaw, &ctx.config.pitch};
    for (unsigned index = 0U; index < 2U; ++index)
    {
        const auto& axis = *axes[index];
        float* destination = values + 5U * index;
        destination[0] = axis.k_angle;
        destination[1] = axis.k_speed;
        destination[2] = axis.gravity_constant;
        destination[3] = axis.gravity_linear;
        destination[4] = axis.moving_friction;
    }
    return ctx.initialized;
}
