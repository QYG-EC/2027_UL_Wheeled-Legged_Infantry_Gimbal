#ifndef BOARD_TRANSPORT_H
#define BOARD_TRANSPORT_H

#include "message_types.h"

/** 仅由当前 CMake 板型选中的固定协议绑定实现；不是动态 Router。 */
bool BoardTransport_Init(void); ///< 回调或静态 Daemon 注册失败时返回 false。
bool BoardTransport_IsOnline(void); ///< 合法且及时的接收流活性，不代表业务 Topic 新鲜。
uint32_t BoardTransport_OfflineDurationMs(void);
void BoardTransport_Poll(void);
/**
 * @brief 每1ms从外置惯性Topic提交Yaw最新值；未发布或超过20ms不提交。
 * @return 是否更新发送槽；false表示未初始化、数据过期、编码或提交失败。
 * @note 仅由上板控制任务调用，成功不代表下板已收到。
 */
bool BoardTransport_SendGimbalYawState(void);
void BoardTransport_SendChassis(const ChassisCmd &command);

/** @brief 上板非阻塞提交左键0/1/2；返回是否入队，成功不等于下板执行。 */
bool BoardTransport_SendMouseCommand(uint8_t command);

#endif
