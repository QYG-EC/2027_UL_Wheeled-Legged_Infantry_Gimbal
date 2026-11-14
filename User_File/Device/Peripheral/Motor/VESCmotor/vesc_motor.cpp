#include "vesc_motor.h"

#include "sys_timestamp.h"

#include <cmath>

Class_VESCMotor* Class_VESCMotor::head = nullptr;

namespace
{
constexpr float TWO_PI = 6.283185307179586f;
constexpr float RAD_PER_DEG = 0.017453292519943295f;
constexpr uint8_t STATUS_IDS[] = {9, 16, 27};

uint32_t EnterCritical()
{
    const uint32_t state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return state;
}

void ExitCritical(uint32_t state)
{
    __DMB();
    __set_PRIMASK(state);
}

int32_t ReadI32(const uint8_t* data)
{
    const uint32_t bits = (static_cast<uint32_t>(data[0]) << 24) |
                          (static_cast<uint32_t>(data[1]) << 16) |
                          (static_cast<uint32_t>(data[2]) << 8) | data[3];
    return static_cast<int32_t>(bits < 0x80000000U ? bits : static_cast<int64_t>(bits) - 0x100000000LL);
}

int16_t ReadI16(const uint8_t* data)
{
    const uint16_t bits = static_cast<uint16_t>((data[0] << 8) | data[1]);
    return static_cast<int16_t>(bits < 0x8000U ? bits : static_cast<int32_t>(bits) - 65536);
}

bool Nonnegative(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}
} // namespace

bool Class_VESCMotor::Init(const Struct_VESCMotor_Init_Config& value)
{
    // 仅启动阶段调用；成功后不可换总线/节点，构造函数不操作硬件。
    if (initialized || !BSP_CAN_ExtendedEnabled(value.hfdcan) || value.can_id == 255 ||
        value.pole_pairs == 0 || value.feedback_timeout_ms == 0 || value.telemetry_timeout_ms == 0 ||
        !Nonnegative(value.current_limit_a) || !Nonnegative(value.speed_limit_rad_s) ||
        !Nonnegative(value.duty_limit) || value.duty_limit > 1.0f ||
        !Nonnegative(value.relative_current_limit) || value.relative_current_limit > 1.0f)
    {
        return false;
    }
    config = value;
    if (!daemon.SetTimeoutMs(config.feedback_timeout_ms))
        return false;

    uint8_t registered = 0;
    for (uint8_t command : STATUS_IDS)
    {
        if (!BSP_CAN_RegisterCallbackEx((static_cast<uint32_t>(command) << 8) | config.can_id,
                                        CAN_ID_EXTENDED, config.hfdcan, RxCallback, this))
            break;
        ++registered;
    }
    if (registered != 3 || !DaemonManager::Register(daemon))
    {
        for (uint8_t index = 0; index < registered; ++index)
        {
            BSP_CAN_UnregisterCallbackEx((static_cast<uint32_t>(STATUS_IDS[index]) << 8) | config.can_id,
                                         CAN_ID_EXTENDED, config.hfdcan, this);
        }
        return false;
    }
    const uint32_t state = EnterCritical();
    initialized = true;
    next = head;
    head = this;
    ExitCritical(state);
    return true;
}

void Class_VESCMotor::InvalidateTarget()
{
    target_valid = false;
    target_mode = Enum_VESCMotor_Mode::CURRENT;
    target_raw = 0;
    safe_output_pending = true;
}

void Class_VESCMotor::CheckSafety()
{
    if (target_valid && (!requested_enabled || !daemon.IsOnline()))
        InvalidateTarget();
}

bool Class_VESCMotor::RequestEnabled(bool enabled)
{
    const uint32_t state = EnterCritical();
    if (!initialized)
    {
        ExitCritical(state);
        return false;
    }
    requested_enabled = enabled;
    CheckSafety();
    bool accepted = true;
    if (!enabled)
    {
        InvalidateTarget();
        accepted = Publish();
    }
    ExitCritical(state);
    return accepted;
}

bool Class_VESCMotor::SetTarget(Enum_VESCMotor_Mode mode, float value)
{
    if (!initialized || !std::isfinite(value))
        return false;
    float limit = 0.0f;
    double scale = 1.0;
    switch (mode)
    {
    case Enum_VESCMotor_Mode::DUTY:
        limit = config.duty_limit;
        scale = 100000.0;
        break;
    case Enum_VESCMotor_Mode::CURRENT:
    case Enum_VESCMotor_Mode::BRAKE_CURRENT:
    case Enum_VESCMotor_Mode::HANDBRAKE_CURRENT:
        limit = config.current_limit_a;
        scale = 1000.0;
        break;
    case Enum_VESCMotor_Mode::SPEED:
        limit = config.speed_limit_rad_s;
        scale = 60.0 * config.pole_pairs / static_cast<double>(TWO_PI);
        break;
    case Enum_VESCMotor_Mode::POSITION:
        if (!config.allow_position || value < 0.0f)
            return false;
        limit = TWO_PI;
        scale = 360000000.0 / static_cast<double>(TWO_PI);
        break;
    case Enum_VESCMotor_Mode::CURRENT_REL:
    case Enum_VESCMotor_Mode::BRAKE_CURRENT_REL:
    case Enum_VESCMotor_Mode::HANDBRAKE_CURRENT_REL:
        limit = config.relative_current_limit;
        scale = 100000.0;
        break;
    default:
        return false;
    }
    const double scaled = static_cast<double>(value) * scale;
    if (std::fabs(value) > limit || scaled < -2147483648.0 || scaled > 2147483647.0)
        return false;

    const uint32_t state = EnterCritical();
    CheckSafety();
    if (!requested_enabled || !daemon.IsOnline())
    {
        ExitCritical(state);
        return false;
    }
    target_mode = mode;
    target_raw = static_cast<int32_t>(scaled);
    target_valid = true;
    ExitCritical(state);
    return true;
}

bool Class_VESCMotor::Publish()
{
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = config.hfdcan;
    message.id_type = CAN_ID_EXTENDED;
    const auto mode = target_valid ? target_mode : Enum_VESCMotor_Mode::CURRENT;
    message.id = (static_cast<uint32_t>(mode) << 8) | config.can_id;
    message.len = 4;
    const uint32_t bits = target_valid ? static_cast<uint32_t>(target_raw) : 0U;
    message.data[0] = static_cast<uint8_t>(bits >> 24);
    message.data[1] = static_cast<uint8_t>(bits >> 16);
    message.data[2] = static_cast<uint8_t>(bits >> 8);
    message.data[3] = static_cast<uint8_t>(bits);
    const bool submitted = CAN_Tx_PerformOwned(&message, this);
    if (submitted)
        safe_output_pending = false;
    return submitted;
}

bool Class_VESCMotor::Send()
{
    const uint32_t state = EnterCritical();
    if (!initialized)
    {
        ExitCritical(state);
        return false;
    }
    CheckSafety();
    const bool submitted = Publish();
    ExitCritical(state);
    return submitted;
}

void Class_VESCMotor::ServiceAll()
{
    for (Class_VESCMotor* motor = head; motor != nullptr; motor = motor->next)
    {
        const uint32_t state = EnterCritical();
        motor->CheckSafety();
        // 正常目标只由应用 Send 发布；安全补交不得替应用续发旧控制命令。
        if (motor->safe_output_pending && !motor->target_valid)
            (void) motor->Publish();
        ExitCritical(state);
    }
}

void Class_VESCMotor::RxCallback(FDCAN_HandleTypeDef* bus, uint32_t id, uint8_t* data,
                                 uint32_t len, void* context)
{
    auto* motor = static_cast<Class_VESCMotor*>(context);
    if (!motor->initialized || bus != motor->config.hfdcan || (id & 0xFFU) != motor->config.can_id)
        return;
    const uint32_t command = id >> 8;
    // STATUS_5 文档有效字段为 6 字节，上游当前发送 8 字节（尾部两字节保留）。
    if ((command == 9 || command == 16) ? len != 8 : (command != 27 || (len != 6 && len != 8)))
        return;
    const uint32_t state = EnterCritical();
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    auto& feedback = motor->feedback;
    if (command == 9)
    {
        // ISR 先发现超时间隙时也清除目标，不能让 Feed 掩盖尚未被 StatusTask 观察的掉线。
        if (motor->motion_seen && !motor->daemon.IsOnline())
        {
            motor->InvalidateTarget();
            // 只复制到 BSP 内存槽，不调用 HAL/RTOS；避免发送任务先于安全任务发出旧目标。
            (void) motor->Publish();
        }
        feedback.speed_rad_s = static_cast<float>(ReadI32(data)) * TWO_PI / (60.0f * motor->config.pole_pairs);
        feedback.current_a = ReadI16(data + 4) / 10.0f;
        feedback.duty = ReadI16(data + 6) / 1000.0f;
        feedback.motion_timestamp_us = now;
        motor->motion_seen = true;
        motor->daemon.Feed();
    }
    else if (command == 16)
    {
        feedback.fet_temperature_c = ReadI16(data) / 10.0f;
        feedback.motor_temperature_c = ReadI16(data + 2) / 10.0f;
        feedback.input_current_a = ReadI16(data + 4) / 10.0f;
        feedback.pid_position_rad = ReadI16(data + 6) * (RAD_PER_DEG / 50.0f);
        feedback.status4_timestamp_us = now;
        motor->status4_seen = true;
    }
    else
    {
        feedback.tachometer_raw = ReadI32(data);
        feedback.input_voltage_v = ReadI16(data + 4) / 10.0f;
        feedback.status5_timestamp_us = now;
        motor->status5_seen = true;
    }
    ExitCritical(state);
}

Struct_VESCMotor_Snapshot Class_VESCMotor::GetSnapshot() const
{
    const uint32_t state = EnterCritical();
    Struct_VESCMotor_Snapshot result = feedback;
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    const uint64_t timeout = static_cast<uint64_t>(config.telemetry_timeout_ms) * 1000ULL;
    result.online = initialized && daemon.IsOnline();
    result.requested_enabled = requested_enabled;
    result.ready = result.online && requested_enabled;
    result.status4_valid = status4_seen && now - feedback.status4_timestamp_us < timeout;
    result.status5_valid = status5_seen && now - feedback.status5_timestamp_us < timeout;
    ExitCritical(state);
    return result;
}
