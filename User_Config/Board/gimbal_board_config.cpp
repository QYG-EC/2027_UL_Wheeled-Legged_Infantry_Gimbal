#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    // 外置惯性固定接485_2/P4，运行时921600，解析HI91反馈。
    static const BoardHardware hardware{
        &hfdcan2, &hfdcan1, nullptr, nullptr, &hfdcan1,
        true, true, true, true, true, true, &huart10, &huart2, &huart10, &huart1};
    return hardware;
}
