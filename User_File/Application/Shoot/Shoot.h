#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief ControlTask 启动时调用一次，注册本板拥有的电机及发送组。
 * @return 全部设备注册与发送组绑定成功返回 true。
 */
bool Shoot_Init(void);
/**
 * @brief 每 1 ms 在 RobotCmd_Update 之后调用，消费连续目标与离散事件。
 * @note 反馈每 10 ms 发布；应用未检查命令年龄，上层负责持续刷新安全目标。
 */
void Shoot_Update(void);

/** @brief 同一控制任务读取双摩擦轮当周期达速许可，关闭或任一驱动未就绪返回false。 */
bool Shoot_FrictionReady(void);

#endif
