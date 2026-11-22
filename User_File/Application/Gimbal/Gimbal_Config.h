#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include "board_config.h"
#include "message_types.h"

enum class GimbalGyroAxis : uint8_t
{
    X,
    Y,
    Z
};
enum class GimbalAngleAxis : uint8_t
{
    Roll,
    Pitch,
    Yaw
};

struct Struct_Gimbal_Motor_Config
{
    FDCAN_HandleTypeDef* bus = nullptr;
    uint8_t id = 0x03U;
    uint16_t feedback_id = 0x005U;
    bool reverse = false;
    float position_max = 3.14f; // 协议量程：rad、rad/s、N·m。
    float velocity_max = 30.0f;
    float torque_max = 10.0f;
    uint8_t disabled_state = 0U;
    uint8_t enabled_state = 1U;
};

/** 单轴LQR（Linear Quadratic Regulator，线性二次调节器）及坐标配置。 */
struct Struct_Gimbal_Axis_Config
{
    Struct_Gimbal_Motor_Config motor;
    bool control_enabled = false; // 参数/方向未确认的轴保持失能，可独立接入另一轴。
    GimbalAngleAxis angle_axis = GimbalAngleAxis::Yaw;
    GimbalGyroAxis gyro_axis = GimbalGyroAxis::Z;
    float angle_sign = 1.0f;
    float gyro_sign = 1.0f;
    float torque_sign = 1.0f;      // 应用轴正转矩到驱动逻辑转矩的符号。
    float motor_per_axis = 1.0f;   // 应用轴到电机的角度/速度比例，用于双轴速度反馈、前馈及位置约束。
    float k_angle = 0.0f;          // N·m/rad。
    float k_speed = 0.0f;          // N·m·s/rad。
    float gravity_constant = 0.0f; // 前馈保持电机逻辑坐标，单位N·m。
    float gravity_linear = 0.0f;   // N·m/rad。
    float moving_friction = 0.0f;  // N·m，仅随参考电机速度取符号，静止时为零。
    float static_friction = 0.0f;  // N·m，Pitch静止参考的到位辅助，按误差/2°连续缩放。
    float torque_limit = 10.0f;
};

/** @brief 将惯性快照映射为应用轴角度，单位rad。 */
inline float Gimbal_AxisAngle(const Struct_Gimbal_Axis_Config& config, const INS_State& state)
{
    const float values[] = {state.roll_rad, state.pitch_rad, state.yaw_rad};
    return config.angle_sign * values[static_cast<unsigned>(config.angle_axis)];
}

/** @brief 将原始惯性快照映射为应用轴角速度，单位rad/s；当前双轴控制速度取电机反馈。 */
inline float Gimbal_AxisSpeed(const Struct_Gimbal_Axis_Config& config, const INS_State& state)
{
    const float values[] = {state.gyro_x_rad_s, state.gyro_y_rad_s, state.gyro_z_rad_s};
    return config.gyro_sign * values[static_cast<unsigned>(config.gyro_axis)];
}

struct Struct_Gimbal_Config
{
    Struct_Gimbal_Axis_Config yaw;
    Struct_Gimbal_Axis_Config pitch;
    bool external_imu = false;
    // Pitch闭环位置目标限制为±20°，触限停止向外积分；不因限位失能。
    float pitch_min = -0.34906585f;
    float pitch_max = 0.34906585f;
};

/**
 * @brief 从BoardConfig绑定总线并装载已归档参数；不访问硬件、不写电机参数。
 * @return 两轴配置；正式双轴链路已开启，实机仍需核对方向及惯性闭环。
 * @note 两轴正式闭环仍需实机验收。
 */
inline Struct_Gimbal_Config Gimbal_Default_Config()
{
    Struct_Gimbal_Config config;
    config.external_imu = BoardConfig_Get().gimbal_imu_uart != nullptr;
    // 已确认左转时yaw、gyro Z和电机位置均增大，沿用默认正向坐标。
    config.yaw.motor.bus = BoardConfig_Get().gimbal_yaw_bus;
    config.yaw.control_enabled = true;
    config.yaw.motor.id = 0x61U;
    config.yaw.motor.feedback_id = 0x71U;
    config.yaw.motor.position_max = 3.14159f;
    config.yaw.motor.torque_max = 25.0f;
    config.yaw.motor.disabled_state = 6U;
    config.yaw.motor.enabled_state = 7U;
    // 1 ms电机反馈模型离线权重为Q=diag([100,0.05])、R=1。
    config.yaw.k_angle = 9.89562444f;
    config.yaw.k_speed = 0.469133869f;
    config.yaw.moving_friction = 0.14f;
    config.yaw.torque_limit = 3.0f;
    config.pitch.motor.bus = BoardConfig_Get().gimbal_pitch_bus;
    config.pitch.motor.id = 0x60U;
    config.pitch.motor.feedback_id = 0x70U;
    config.pitch.motor.position_max = 12.5f;
    config.pitch.motor.torque_max = 50.0f;
    config.pitch.motor.disabled_state = 6U;
    config.pitch.motor.enabled_state = 7U;
    config.pitch.control_enabled = true;
    // 抬头为正；位置取−Pitch，控制速度取电机速度/传动比，gyro配置仅保留原生惯性映射。
    config.pitch.angle_axis = GimbalAngleAxis::Pitch;
    config.pitch.gyro_axis = GimbalGyroAxis::X;
    config.pitch.angle_sign = -1.0f;
    config.pitch.gyro_sign = -1.0f;
    // 用户2026-11-20实测调定的1ms离散增益；固件不在线求解，Q/R未随此K记录。
    config.pitch.k_angle = 9.29481325f;
    config.pitch.k_speed = 1.99631282f;
    config.pitch.gravity_constant = 0.210217638139f;
    config.pitch.gravity_linear = -0.714052927197f;
    config.pitch.moving_friction = 0.0f;
    config.pitch.static_friction = 0.15f;
    config.pitch.torque_limit = 50.0f;
    return config;
}

#endif
