#ifndef HIPNUC_H
#define HIPNUC_H

#include "usart.h"
#include <cstdint>

/** HI91/HI83 中同时取得的姿态和机体系角速度；均为 rad、rad/s。 */
struct Struct_Hipnuc_State
{
    float angle_rad[3]{}; // Roll、Pitch、Yaw，保留传感器坐标。
    float gyro_rad_s[3]{};
    uint64_t received_us = 0U;
    uint32_t sequence = 0U;
    uint16_t status = 0U;
    uint8_t tag = 0U;
};

/**
 * @brief 启动单个惯性单元的串口接收；不发送配置命令或改写设备参数。
 * @param uart BoardConfig 指定、已配置接收 DMA 的串口。
 * @return 接口具备接收条件时返回 true；仅启动阶段调用一次。
 */
bool Hipnuc_Init(UART_HandleTypeDef* uart);
/** @brief 原子复制最新完整反馈；首次完整帧前返回 false，不改变输出。 */
bool Hipnuc_ReadLatest(Struct_Hipnuc_State& state);
/**
 * @brief 向解析器交付接收字节，支持拆帧和连续多帧。
 * @note 由唯一串口回调调用；测试可在没有并发串口接收时直接调用。
 */
void Hipnuc_Receive(uint8_t* bytes, uint16_t size);

#endif
