#ifndef GIMBAL_LINK_PROTOCOL_H
#define GIMBAL_LINK_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define GIMBAL_LINK_UPLINK_ID 0x80U
#define GIMBAL_LINK_DOWNLINK_ID 0x81U
#define GIMBAL_LINK_VERSION 1U
#define GIMBAL_LINK_PAYLOAD_SIZE 8U

enum GimbalLink_MessageType
{
    GIMBAL_LINK_CHASSIS_COMMAND = 0U,
    GIMBAL_LINK_CHASSIS_FEEDBACK = 1U,
    GIMBAL_LINK_RATE_INPUT = 2U,
    GIMBAL_LINK_MOUSE_COMMAND = 3U
};

/** 两轴操作者目标角速度，rad/s；Yaw正为左转，Pitch正为抬头。 */
typedef struct
{
    float yaw_speed_rad_s;
    float pitch_speed_rad_s;
    bool enabled;
    bool autoaim_requested; /* 仅在enabled许可下应用。 */
} GimbalLink_RateInput;

/** 底盘目标：平移m/s、旋转rad/s，mode为现有0～3模式。 */
typedef struct
{
    float velocity_x_m_s;
    float velocity_y_m_s;
    float angular_velocity_rad_s;
    uint8_t mode;
} GimbalLink_ChassisCommand;

/** 底盘实测速度与状态，单位同命令，不是云台输入。 */
typedef struct
{
    float velocity_x_m_s;
    float velocity_y_m_s;
    float angular_velocity_rad_s;
    bool enabled;
    bool online;
} GimbalLink_ChassisFeedback;

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief 编码上板左键动作，command为0停止、1单次、2持续。 */
    bool GimbalLink_EncodeMouseCommand(uint8_t command, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE]);
    /** @brief 解码左键动作，失败不修改输出。 */
    bool GimbalLink_DecodeMouseCommand(const uint8_t* bytes, uint32_t size, uint8_t* command, uint8_t* sequence);
    /** @brief 读取8字节帧中的版本与类型；不判断总线方向或刷新控制时效。 */
    bool GimbalLink_GetMessageType(const uint8_t* bytes, uint32_t size, uint8_t* type);
    /**
     * @brief 编码0x80云台速度帧，分辨率0.001 rad/s。
     * @param input 完整目标；非有限值或超出编码范围时失败，不做物理限速。
     * @param sequence 由调用方为本类型独立维护的8位序号。
     * @param bytes 至少8字节的输出；失败不修改输出，不提交CAN发送。
     */
    bool GimbalLink_EncodeRateInput(const GimbalLink_RateInput* input, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE]);
    /** @brief 解码云台输入；校验版本、类型、保留位，失败不修改输出及序号。 */
    bool GimbalLink_DecodeRateInput(const uint8_t* bytes, uint32_t size, GimbalLink_RateInput* input, uint8_t* sequence);
    /** @brief 编码0x81底盘命令；平移/旋转分辨率为0.001 m/s和0.001 rad/s。 */
    bool GimbalLink_EncodeChassisCommand(const GimbalLink_ChassisCommand* command, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE]);
    /** @brief 解码0x81底盘命令，失败不修改输出及序号。 */
    bool GimbalLink_DecodeChassisCommand(const uint8_t* bytes, uint32_t size, GimbalLink_ChassisCommand* command, uint8_t* sequence);
    /** @brief 编码0x80底盘实测反馈，失败不修改输出。 */
    bool GimbalLink_EncodeChassisFeedback(const GimbalLink_ChassisFeedback* feedback, uint8_t sequence, uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE]);
    /** @brief 解码底盘反馈，失败不修改输出及序号。 */
    bool GimbalLink_DecodeChassisFeedback(const uint8_t* bytes, uint32_t size, GimbalLink_ChassisFeedback* feedback, uint8_t* sequence);
    /** @brief 无符号序号前进1～127为新帧，支持255→0；不维护接收状态。 */
    bool GimbalLink_SequenceNewer(uint8_t candidate, uint8_t previous);

#ifdef __cplusplus
}
#endif
#endif
