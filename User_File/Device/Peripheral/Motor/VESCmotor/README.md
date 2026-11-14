# VESC 电调驱动

`Class_VESCMotor` 封装官方 VESC CAN 的九种基本单帧控制与 STATUS/STATUS_4/STATUS_5。
复用现有 CAN BSP、Daemon 和 StatusTask，不绑定实际机构、不创建额外任务。

协议依据（核对日期 2026-11-14）：[官方文档](https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md)
及 [comm_can.c](https://github.com/vedderb/bldc/blob/master/comm/comm_can.c)。目标固件版本仍须实机核对。
不包含多帧命令转发、参数读写、固件更新或六字节 off-delay 控制变体。

## 1. 按总线启用

CMake 选项 `H7_CAN1_EXTENDED`、`H7_CAN2_EXTENDED`、`H7_CAN3_EXTENDED` 默认均为 `OFF`。
只为接入 VESC 的总线开启，例如当前配置为 SingleBoard 时：

```sh
cmake --preset SingleBoard -DH7_CAN1_EXTENDED=ON
cmake --build --preset SingleBoard
```

沿用实际选中的 preset，不为启用协议改板型或 Debug/Release 配置。开关写入 CMake cache，
之后重新 configure 会保留；需要关闭时显式设 `OFF`。CAN1 开启不会开启 CAN2/3。
现有标准帧仍可使用，完整收发键含总线、帧类型和 ID。详见 [BSP](../../../../Middleware/BSP/README.md#41-接收)。

本驱动 `Init()` 对未开启扩展帧的总线返回 false。不改变波特率或引脚；在 VESC Tool 设置
CAN Mode 为 VESC、唯一节点 ID（0～254）及与该总线一致的波特率。启用 STATUS 50 Hz，
STATUS_4/5 可设 10 Hz，保留电调通信超时（官方默认 0.5 s），控制目标至少以 50 Hz 周期发送。
按实物确认 CAN_H/CAN_L、参考地及总线两端终端电阻。

## 2. 接口与单位

构造函数不访问硬件；实例静态存活，`Init()` 在启动阶段调用。一个应用任务独占实例目标。
需要配置总线、节点、极对数及适合实物的输出上限；所有输出幅值上限默认零，位置模式默认禁用。
不得将极数填写为极对数。转矩常数、减速器换算和方向由拥有设备的应用处理。

| 接口 | 命令号 | 输入 | 报文缩放 |
| --- | --- | --- | --- |
| `SetDuty()` | 0 | 比例 [-1,1] | ×100000 |
| `SetCurrent()` | 1 | A | ×1000 |
| `SetBrakeCurrent()` | 2 | A | ×1000 |
| `SetSpeed()` | 3 | 电机轴机械 rad/s | ×60×极对数/(2π)，得到 ERPM |
| `SetPosition()` | 4 | VESC PID 坐标 rad，[0,2π] | 转 degree 后 ×1000000 |
| `SetCurrentRelative()` | 10 | 比例 [-1,1] | ×100000 |
| `SetBrakeCurrentRelative()` | 11 | 比例 [-1,1] | ×100000 |
| `SetHandbrakeCurrent()` | 12 | A | ×1000 |
| `SetHandbrakeCurrentRelative()` | 13 | 比例 [-1,1] | ×100000 |

也可使用 `SetTarget(mode, value)`。全部发送 4 字节大端有符号整数，ID 为 `(command << 8) | node`。
不增加上位机 PID，速度和位置闭环由 VESC 固件执行。位置不是累计多圈角或减速器输出角。

`current_limit_a` 约束绝对电流类命令，`relative_current_limit` 独立约束相对命令。
相对电流与占空比、位置模式仍依赖电调内设置的电流/速度/电压边界，不承诺受本地 A 限值约束。
`SetTarget()` 仅在 ready 时接受目标；非法值、非有限数、超限值及编码溢出返回 false，保留原目标。
目标设置与发送分开，调用方不能忽略设置失败后仍发送旧目标。

## 3. 调用与安全行为

```cpp
#include "vesc_motor.h"

static Class_VESCMotor motor;

bool Example_Init(FDCAN_HandleTypeDef *bus, uint16_t pole_pairs, float verified_current_limit_a)
{
    Struct_VESCMotor_Init_Config config{};
    config.hfdcan = bus; // 从 BoardConfig 的硬件绑定传入，不能在控制周期换总线。
    config.can_id = 23;
    config.pole_pairs = pole_pairs;
    config.current_limit_a = verified_current_limit_a;
    return motor.Init(config);
}

bool Example_Update(bool permitted, float current_a)
{
    if (!motor.RequestEnabled(permitted)) return false;
    if (!permitted) return true;
    if (!motor.GetSnapshot().ready) return motor.Send(); // 等待反馈，只发送零电流。
    if (!motor.SetCurrent(current_a))
    {
        (void)motor.RequestEnabled(false); // 安全提交失败由 ServiceAll 重试。
        return false;
    }
    return motor.Send();
}
```

顺序为初始化 → 请求使能 → 新鲜 STATUS → 设置目标并周期 Send，不能先等待 ready 才请求使能。
`ready` 只表示软件许可与运动反馈在线，不代表电调已使能、无故障或已执行目标。
返回 true 只表示请求/软件发送槽接受，不能作为 CAN 发送或电调执行确认。

每实例固定一个 owner 发送槽。切换模式覆盖完整 ID 和数据，不积累旧模式待发送帧；
BSP 1 ms 服务只发送发生更新的槽，不自行无限重发正常目标。已进硬件 FIFO 的帧无法撤回。

失能或 STATUS 超时（默认 100 ms）清除目标，以零电流释放命令覆盖槽。
StatusTask 每 10 ms 独立执行安全检查及失败补交；ISR 收到跨越超时窗口的恢复 STATUS 时也清除目标，
避免恢复反馈掩盖掉线。恢复后需重新设置目标。零电流表示释放，不是断电或保持制动。
初始化后也保持零电流安全默认；绝不等待应用继续运行才处理反馈超时。

没有 VESC 实例时不发送任何 VESC 帧；三路开关默认关闭。不会改动现有机构的电机绑定。

## 4. 反馈与资源

- STATUS（9，8 字节）：ERPM 转机械 rad/s、电流 A、占空比；只有该帧刷新 Daemon。
- STATUS_4（16，8 字节）：MOS/电机温度 °C、输入电流 A、PID 位置 rad。
- STATUS_5（27）：原始 tachometer、电源电压 V。文档描述 6 字节有效字段；当前上游发送 8 字节，
  后两字节保留，因此接受 6 或 8 字节，拒绝其他长度。原始计数不直接作为绝对位置。

快照通过短临界区一次复制，含各组时间戳。辅助反馈默认 500 ms 过期，分别看
`status4_valid`、`status5_valid`；旧数值保留供诊断，不能忽略有效标志。
这些状态消息没有完整故障码，因此不提供虚假的 `fault=false` 或实际使能确认。

每实例占 3 个接收注册项、1 个周期槽和 1 个 Daemon。BSP 当前总容量为 16 个接收项、
32 个周期槽，Daemon 为 32 项，与已有设备共享容量。相同总线重复节点注册失败；不同总线可同 ID。
初始化失败会回滚自身成功注册的接收项；发送槽不足时 Send 返回 false，不覆盖其他设备。

## 5. 验证

```sh
python3 Tests/CAN/run.py
```

测试编译真实 BSP、Daemon、VESC 源码，HAL/RTOS/时间采用主机 mock；覆盖开关、总线/帧类型隔离、
协议向量、队列容量、模式切换竞争、掉线恢复及安全重试。默认关闭、单路分别开启和三路开启
分别运行，不改固件 CMake cache。实机还需验证波特率、负载、传感器/PID 配置、方向和拔线保护。
