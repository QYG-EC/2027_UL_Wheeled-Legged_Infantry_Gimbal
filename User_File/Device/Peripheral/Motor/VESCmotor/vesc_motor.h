/** @file vesc_motor.h
 * @brief VESC Classic CAN 单帧控制；实例静态存活，一个应用任务拥有目标。
 */
#ifndef VESC_MOTOR_H
#define VESC_MOTOR_H

#include "bsp_can.h"
#include "daemon.h"

enum class Enum_VESCMotor_Mode : uint8_t
{
    DUTY = 0,
    CURRENT = 1,
    BRAKE_CURRENT = 2,
    SPEED = 3,
    POSITION = 4,
    CURRENT_REL = 10,
    BRAKE_CURRENT_REL = 11,
    HANDBRAKE_CURRENT = 12,
    HANDBRAKE_CURRENT_REL = 13
};

struct Struct_VESCMotor_Init_Config
{
    FDCAN_HandleTypeDef* hfdcan = nullptr; ///< 对应总线须开启 H7_CANx_EXTENDED。
    uint8_t can_id = 0;                    ///< 0..254；255 广播不允许。
    uint16_t pole_pairs = 0;               ///< 必填，速度按电机轴机械 rad/s 换算。
    float current_limit_a = 0.0f;          ///< 对称绝对电流限值，含制动/手刹；0 只允许零。
    float speed_limit_rad_s = 0.0f;
    float duty_limit = 0.0f;             ///< [0,1]。
    float relative_current_limit = 0.0f; ///< [0,1]，相对电流由电调配置换算，不受本地 A 限值约束。
    bool allow_position = false;         ///< 位置模式需显式许可并在电调侧配置电流/速度边界。
    uint32_t feedback_timeout_ms = 100;
    uint32_t telemetry_timeout_ms = 500;
};

struct Struct_VESCMotor_Snapshot
{
    float speed_rad_s = 0.0f;
    float current_a = 0.0f;
    float duty = 0.0f;
    float fet_temperature_c = 0.0f;
    float motor_temperature_c = 0.0f;
    float input_current_a = 0.0f;
    float pid_position_rad = 0.0f; ///< VESC PID 原生坐标，不声明多圈/减速后位置。
    float input_voltage_v = 0.0f;
    int32_t tachometer_raw = 0; ///< 协议原始计数，不作为绝对角度。
    uint64_t motion_timestamp_us = 0;
    uint64_t status4_timestamp_us = 0;
    uint64_t status5_timestamp_us = 0;
    bool online = false;
    bool status4_valid = false;
    bool status5_valid = false;
    bool requested_enabled = false;
    bool ready = false; ///< 软件许可及新鲜 STATUS；不是电调使能或无故障确认。
};

class Class_VESCMotor
{
  public:
    Class_VESCMotor() = default;
    Class_VESCMotor(const Class_VESCMotor&) = delete;
    Class_VESCMotor& operator=(const Class_VESCMotor&) = delete;
    bool Init(const Struct_VESCMotor_Init_Config& config);
    bool RequestEnabled(bool enabled); ///< false 清除旧目标并提交零电流；返回软件接受结果。
    /** 只更新目标；须 ready。非法值返回 false 且保留原目标；模式切换在 Send 时发布。 */
    bool SetTarget(Enum_VESCMotor_Mode mode, float value);
    bool SetDuty(float value)
    {
        return SetTarget(Enum_VESCMotor_Mode::DUTY, value);
    }
    bool SetCurrent(float amperes)
    {
        return SetTarget(Enum_VESCMotor_Mode::CURRENT, amperes);
    }
    bool SetBrakeCurrent(float amperes)
    {
        return SetTarget(Enum_VESCMotor_Mode::BRAKE_CURRENT, amperes);
    }
    bool SetSpeed(float rad_s)
    {
        return SetTarget(Enum_VESCMotor_Mode::SPEED, rad_s);
    }
    bool SetPosition(float rad)
    {
        return SetTarget(Enum_VESCMotor_Mode::POSITION, rad);
    }
    bool SetCurrentRelative(float value)
    {
        return SetTarget(Enum_VESCMotor_Mode::CURRENT_REL, value);
    }
    bool SetBrakeCurrentRelative(float value)
    {
        return SetTarget(Enum_VESCMotor_Mode::BRAKE_CURRENT_REL, value);
    }
    bool SetHandbrakeCurrent(float amperes)
    {
        return SetTarget(Enum_VESCMotor_Mode::HANDBRAKE_CURRENT, amperes);
    }
    bool SetHandbrakeCurrentRelative(float value)
    {
        return SetTarget(Enum_VESCMotor_Mode::HANDBRAKE_CURRENT_REL, value);
    }
    bool Send(); ///< 周期调用；未就绪/无新目标时发布零电流，成功不表示设备执行。
    Struct_VESCMotor_Snapshot GetSnapshot() const;
    static void ServiceAll(); ///< StatusTask 调用，只维护安全输出，不重发正常旧目标。

  private:
    static void RxCallback(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void*);
    void CheckSafety(); ///< 调用者持有临界区。
    bool Publish();     ///< 调用者持有临界区。
    void InvalidateTarget();
    Struct_VESCMotor_Init_Config config{};
    Struct_VESCMotor_Snapshot feedback{};
    Daemon daemon{100};
    Class_VESCMotor* next = nullptr;
    static Class_VESCMotor* head;
    Enum_VESCMotor_Mode target_mode = Enum_VESCMotor_Mode::CURRENT;
    int32_t target_raw = 0;
    bool initialized = false;
    bool requested_enabled = false;
    bool target_valid = false;
    bool safe_output_pending = true;
    bool motion_seen = false;
    bool status4_seen = false;
    bool status5_seen = false;
};

#endif
