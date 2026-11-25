#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <cstdint>

enum class ChassisMode : uint8_t
{
    ZERO_FORCE = 0,
    NO_FOLLOW,
    FOLLOW_GIMBAL_YAW,
    ROTATE,
};

enum class GimbalMode : uint8_t
{
    DISABLED = 0,
    IMU,
    LOCK,
    RATE, ///< rad/s目标每个控制周期积分；角度字段在此模式下不使用。
};

enum class ShootMode : uint8_t
{
    OFF = 0,
    ON,
};

enum class FrictionMode : uint8_t
{
    OFF = 0,
    ON,
};

/** 拨弹盘的连续工作模式；单发和三连发由 ShootEvent 表示。 */
enum class LoaderMode : uint8_t
{
    STOP = 0,
    REVERSE,
    BURST,
};

enum class ShootEventType : uint8_t
{
    ShootOnce = 0,
    ShootTriple,
};

struct ShootEvent
{
    ShootEventType type = ShootEventType::ShootOnce;
};

struct INS_State
{
    float yaw_rad = 0.0f;
    float pitch_rad = 0.0f;
    float roll_rad = 0.0f;
    float gyro_x_rad_s = 0.0f;
    float gyro_y_rad_s = 0.0f;
    float gyro_z_rad_s = 0.0f;
};

/** 两轴目标均为应用惯性姿态 rad；速度字段为 IMU 前馈或 RATE 积分输入。
 * LOCK 捕获当前姿态并忽略目标字段；自动恢复后 IMU 重新应用当前有效绝对目标。
 */
struct GimbalCmd
{
    float yaw_angle_rad = 0.0f;
    float pitch_angle_rad = 0.0f;
    float yaw_speed_rad_s = 0.0f;
    float pitch_speed_rad_s = 0.0f;
    GimbalMode mode = GimbalMode::DISABLED;
};

/** 下板遥控生成的云台目标角速度；正号为 Yaw 左转、Pitch 抬头。 */
struct GimbalRateInput
{
    float yaw_speed_rad_s = 0.0f;
    float pitch_speed_rad_s = 0.0f;
    bool enabled = false;
    bool autoaim_requested = false; ///< CH8许可下的CH7自瞄请求。
};

struct ChassisCmd
{
    float velocity_x_m_s = 0.0f;
    float velocity_y_m_s = 0.0f;
    float angular_velocity_rad_s = 0.0f;
    ChassisMode mode = ChassisMode::ZERO_FORCE;
};

/** 内部物理量采用 SI：发射转速 rad/s，射频 Hz。 */
struct ShootCmd
{
    float friction_speed_rad_s = 0.0f;
    float loader_speed_rad_s = 0.0f;
    float shoot_rate_hz = 0.0f;
    ShootMode shoot_mode = ShootMode::OFF;
    FrictionMode friction_mode = FrictionMode::OFF;
    LoaderMode loader_mode = LoaderMode::STOP;
};

/** 外置IMU（Inertial Measurement Unit，惯性测量单元）Yaw，左转为正。
 * 接收端Topic时间戳是本板CAN收帧时间，不是远端传感器采样时间。
 */
struct GimbalYawState
{
    float yaw_rad = 0.0f;         ///< 单圈角度，rad，[-pi, pi]。
    float yaw_speed_rad_s = 0.0f; ///< 惯性Z轴角速度，rad/s，不是电机速度。
};

/** 姿态采用配置的惯性轴及符号；enabled表示功能获许可且参与控制的轴均ready。
 * gyro 轴由云台配置选择；INS 无效时姿态/速度清零，调用者须检查 ins_valid。
 */
struct GimbalFeedback
{
    float yaw_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_speed_rad_s = 0.0f;
    float pitch_speed_rad_s = 0.0f;
    bool ins_valid = false;
    bool enabled = false;
    bool yaw_ready = false;
    bool pitch_ready = false;
};

struct ChassisFeedback
{
    float velocity_x_m_s = 0.0f;
    float velocity_y_m_s = 0.0f;
    float angular_velocity_rad_s = 0.0f;
    bool enabled = false; ///< 八个 DJI 电机均 ready；ZERO_FORCE 或任一反馈过期时为 false。
    bool online = false;
};

/** DJI 输出轴反馈直接使用 rad 和 rad/s，不经角度制转换。 */
struct ShootFeedback
{
    float friction_left_speed_rad_s = 0.0f;
    float friction_right_speed_rad_s = 0.0f;
    float loader_angle_rad = 0.0f;
    float loader_speed_rad_s = 0.0f;
    bool enabled = false; ///< 本板拥有的 DJI 电机均 ready；正式上板仅双摩擦轮，OFF 时为 false。
    bool online = false;
};

/** UI 状态独立于控制模式，Unknown 不代表停机。 */
enum class Enum_Client_UI_Chassis : uint8_t
{
    Unknown,
    Stopped,
    Waiting,
    Active,
    Spin,
};

enum class Enum_Client_UI_Motion : uint8_t
{
    Unknown,
    Stopped,
    Moving,
};

enum class Enum_Client_UI_AutoAim : uint8_t
{
    Unknown,
    Off,
    Waiting,
    Active,
};

enum class Enum_Client_UI_FireMode : uint8_t
{
    Unknown,
    Single,
    Triple,
    Burst,
};

/** ControlTask 唯一发布，StatusTask 消费；原始反馈时间戳不会随采集刷新。
 * 文字为至多 29 字节的 ASCII 加结尾 NUL；报警无需显示时置空。
 * yaw/诊断文字由具体机器人适配器提供，整体快照有效期 200 ms。
 */
struct Struct_Client_UI_State
{
    uint64_t pitch_timestamp_us = 0U;
    uint64_t shoot_timestamp_us = 0U;
    uint64_t chassis_timestamp_us = 0U;
    float pitch_rad = 0.0f;
    float relative_yaw_rad = 0.0f;
    float left_speed_rad_s = 0.0f;
    float right_speed_rad_s = 0.0f;
    bool pitch_valid = false;
    bool yaw_valid = false;
    bool speed_valid = false;
    Enum_Client_UI_Chassis chassis = Enum_Client_UI_Chassis::Unknown;
    Enum_Client_UI_Motion friction = Enum_Client_UI_Motion::Unknown;
    Enum_Client_UI_Motion loader = Enum_Client_UI_Motion::Unknown;
    Enum_Client_UI_AutoAim autoaim = Enum_Client_UI_AutoAim::Unknown;
    // 目标源时间使用 HAL tick ms；自瞄状态由本板输入仲裁生成。
    uint32_t target_received_ms = 0U;
    uint64_t autoaim_timestamp_us = 0U;
    bool target_detected = false;
    bool initializing = true;
    // 功率来自实测源；默认适配器不提供功率或持久发射模式。
    uint64_t power_timestamp_us = 0U;
    uint64_t fire_mode_timestamp_us = 0U;
    float power_w = 0.0f;
    bool power_valid = false;
    Enum_Client_UI_FireMode fire_mode = Enum_Client_UI_FireMode::Unknown;
    // 裁判源时间使用 HAL tick ms，消费端按无符号差处理回绕。
    uint32_t referee_status_ms = 0U;
    uint32_t referee_heat_ms = 0U;
    uint32_t referee_allowance_ms = 0U;
    uint16_t buffer_energy_j = 0U;
    uint16_t barrel_heat = 0U;
    uint16_t barrel_heat_limit = 0U;
    uint16_t projectile_allowance = 0U;
    bool referee_status_valid = false;
    bool referee_heat_valid = false;
    bool referee_allowance_valid = false;
    char mode_text[30] = "UNKNOWN";
    char alert_text[3][30]{};
    bool alert_warning[3]{};
};

#endif
