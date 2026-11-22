#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

/** @brief ControlTask的1kHz入口：双轴惯性角度、电机速度反馈与转矩输出；状态100Hz发布。 */
void Gimbal_Update(void);

#include "Gimbal_Config.h"

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE,
    Gimbal_Status_ENABLING,
    Gimbal_Status_READY,
    Gimbal_Status_FAULT,
    Gimbal_Status_CONFIG_ERROR,
};

/**
 * @brief 启动阶段复制两轴配置并注册驱动，不等待应答、不置零、不切换电机模式。
 * @param config 电机、坐标映射及两轴控制参数；未标定轴control_enabled保持false。
 * @return 参数有效且需要控制的轴注册成功时为true；仅启动阶段调用一次。
 */
bool Gimbal_Init(const Struct_Gimbal_Config& config = Gimbal_Default_Config());
/** @brief 返回应用状态，忽略配置为不参与控制的轴；仅供同一任务上下文读取。 */
Enum_Gimbal_Status Gimbal_GetStatus(void);

/** @brief ControlTask在更新前整体写入两轴调参值，不重置目标、不改变设备配置。
 * @param values Yaw后接Pitch；每轴为k_angle、k_speed、gravity_constant、gravity_linear、moving_friction。
 * @return 已初始化且全部值合法时写入并返回true，否则两轴均保持原值。
 */
bool Gimbal_SetTuningParameters(const float (&values)[10]);
/** @brief 在同一ControlTask中读取实际运行参数，顺序与写入相同。
 * @param values 输出两轴当前参数。 @return 应用是否已初始化。
 */
bool Gimbal_GetTuningParameters(float (&values)[10]);
#endif
