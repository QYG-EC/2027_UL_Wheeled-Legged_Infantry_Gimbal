#ifndef RM_SOURCE_ARBITRATION_H
#define RM_SOURCE_ARBITRATION_H

#include "input_state.h"

struct InputDecision
{
    ChassisCmd chassis{};
    GimbalCmd gimbal{};
    ShootCmd shoot{};
    InputSource source = InputSource::Remote;
    bool armed = false;
};

/** 上板键鼠优先，显式请求且有有效视觉目标时覆盖绝对角；其他板型保留原有策略。 */
InputDecision SourceArbitration_Resolve(const InputState &state);

/**
 * @brief 目标有效、未过期且不早于切换边界；供仲裁与界面共用。
 * @param state 同一控制任务内的完整输入快照，包含当前毫秒时刻和配置时效。
 * @return 两轴有限且目标通过切换边界与时效检查时为true，时效0表示关闭超时。
 */
bool SourceArbitration_VisionReady(const InputState& state);

#endif
