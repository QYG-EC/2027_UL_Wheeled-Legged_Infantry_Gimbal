#ifndef VISION_H
#define VISION_H

#include "usart.h"

#include <cstdint>

constexpr uint32_t VISION_BAUD_RATE = 921600U;

/** 24字节视觉参考；应用坐标Yaw左转、Pitch抬头为正，单位rad、m。 */
struct Struct_Vision_Reference
{
    float yaw_rad = 0.0f;
    float pitch_rad = 0.0f;
    float distance_m = 0.0f;
    float target_spin_rad_s = 0.0f; ///< 目标自转，不是云台参考速度。
    uint32_t received_ms = 0U;
    uint32_t sequence = 0U;
    uint8_t target_id = 0U;
    bool fire_advice = false;
    bool valid = false; ///< target_state为0时明确撤销参考，不作为回零命令。
};

/** 同次姿态快照的24字节反馈；角度rad，Pitch抬头为正。 */
struct Struct_Vision_Feedback
{
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    float bullet_speed_m_s = 0.0f;
    uint32_t sampled_ms = 0U; ///< 同一快照重复发送时不得刷新。
    uint8_t mode = 0U;        ///< 0红装甲、1蓝装甲，2～5为能量机关模式。
};

/**
 * @brief 启动串口1的24字节协议中断接收；仅初始化阶段调用，不授予控制许可。
 * @param uart BoardConfig绑定的无接收DMA串口，8N1、无流控。
 * @return VISION_BAUD_RATE波特率配置及接收启动成功时为true。
 */
bool Vision_Init(UART_HandleTypeDef* uart);
/** @brief 读取最新完整参考，包括无目标帧；首次收帧前返回false，不修改输出、不续期。 */
bool Vision_ReadLatest(Struct_Vision_Reference& reference);
/** @brief 向唯一接收解析器交付字节；支持拆帧/粘帧，仅串口回调或无并发测试调用。 */
void Vision_Receive(uint8_t* bytes, uint16_t size);
/**
 * @brief 编码并非阻塞提交24字节反馈；忙时不排队，下一周期使用最新姿态。
 * @param feedback 角度rad、弹速m/s，浮点值须有限，mode为0～5。
 * @return HAL（Hardware Abstraction Layer，硬件抽象层）提交状态，不表示对端已收到。
 */
uint8_t Vision_SendFeedback(const Struct_Vision_Feedback& feedback);

#endif
