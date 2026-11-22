#ifndef GIMBAL_INPUT_H
#define GIMBAL_INPUT_H

#include <cstdint>

/** 输入标定：X正为右转，Y正为抬头；鼠标原始值每帧换算为rad/s。 */
struct Struct_Gimbal_Input_Config
{
    float pitch_rad_per_count = 0.0002f;
    float yaw_rad_per_count = 0.0002f;
    float mouse_frame_period_s = 1.0f / 30.0f;
    uint32_t vision_target_max_age_ms = 1000U;   ///< 0关闭目标超时；不改变无目标帧语义。
    uint32_t vision_feedback_max_age_ms = 1000U; ///< 0允许反馈旧姿态；保留原采样时刻。
    uint32_t vision_feedback_period_ms = 10U;    ///< 必须大于0，单位ms。
    float vision_bullet_speed_m_s = 16.0f;
};

/**
 * @brief 初始化上板来源状态、鼠标标定和视觉参数，上电键鼠关闭；仅在控制任务启动时调用。
 * @param config 输入与视觉配置；时间为ms，反馈周期须大于0，弹速为m/s。
 */
void GimbalInput_Init(const Struct_Gimbal_Input_Config& config = {});
/** @brief 控制任务中消费完整快照、处理Ctrl+S边沿并提交操作者和视觉输入，不直接发布机构命令。 */
void GimbalInput_Update();

/** @brief Shoot_Update之后调用，按本周期双轮达速结果发送左键动作；仅控制任务调用。 */
void GimbalInput_SendMouseCommand();

#endif
