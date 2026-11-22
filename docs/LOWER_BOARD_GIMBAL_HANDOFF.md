# 下板Agent任务：接入云台目标速度与双板CAN通信

本文可直接交给下板工程的Agent执行。协议以2026-11-20上下板实现为准，包含CH8总使能下的CH7自瞄请求。

## 1. 任务与边界

1. 先阅读下板工程自己的AGENTS.md、说明、现有遥控输入和CAN发送代码，保留未提交修改。
2. 沿用下板已有框架和周期任务，实现向上板提交两轴云台目标角速度、使能标志与自瞄请求。
3. 本次不处理云台电机，不在下板做云台速度积分、位置闭环或惯性反馈控制。
4. 使能作为调用接口的布尔参数传入；按钮映射归下板现有输入模块，不要求上板知道按钮编号。
5. 遥控失效时沿用下板现有有效性判断，发送enabled=false及两轴零速度；不要给旧遥控输入不断重新盖时间戳。不要额外引入解锁条件、自动重连、限速或超时策略。
6. 不自行烧录、提交或合并；报告修改文件与真实验证结果。先保证上板对接报文正确，再由负责人安排实机验证。

上板已经实现：键鼠Ctrl+S总开关，上电关闭；开启后键鼠始终优先，独立于下板遥控许可；键鼠失效仍保持控制权。下板正常持续发送遥控状态即可，不根据鼠标是否运动抢占控制。

## 2. 文件交接

将下面两个独立文件复制到下板现有协议目录，并纳入已有构建：

- [gimbal_link_protocol.h](../User_File/System/Transport/gimbal_link_protocol.h)
- [gimbal_link_protocol.c](../User_File/System/Transport/gimbal_link_protocol.c)

它们兼容C99与C++，只依赖标准头文件及数学函数lroundf；工具链如要求，链接数学库m。
它们只编解码，不调用硬件、不发送数据、不分配动态内存、不维护周期或接收序号状态。
不要复制上板的HAL（Hardware Abstraction Layer，硬件抽象层）句柄、RTOS（Real-Time Operating System，实时操作系统）任务、MessageCenter或电机控制代码。

## 3. 下板需要提供的清单

### 云台遥控输入：必需，消息类型2

| 字段 | 含义 | 单位/方向 |
| --- | --- | --- |
| yaw_speed_rad_s | 操作者期望的Yaw角速度 | rad/s，正为向左转 |
| pitch_speed_rad_s | 操作者期望的Pitch角速度 | rad/s，正为抬头 |
| enabled | 本路遥控是否允许云台控制 | true允许，false失能该来源 |
| autoaim_requested | 遥控健康且CH8>1400、CH7>1400时为true | 仅请求，不独立授予使能 |
| sequence | 本类型独立发送序号 | 软件提交成功后加1，8位回绕 |

这是云台目标速度，不是底盘实测速度。下板不发送目标角度、LQR（Linear Quadratic Regulator，线性二次调节器）增益、按钮编号、IMU（Inertial Measurement Unit，惯性测量单元）数据或电机控制帧。

### 底盘反馈：保留现有功能，消息类型1

若下板已有底盘反馈，继续提供实测vx、vy、wz及enabled、online，并迁移到本文协议。其序号必须独立于云台输入；没有实测反馈时不要编造数据，不要为本任务新做底盘速度估计。上板云台控制不依赖这份底盘反馈才能运行。

## 4. 接线与CAN标识

CAN（Controller Area Network，控制器局域网）使用Classic CAN标准数据帧，1 Mbit/s，载荷8字节。

| 方向 | 外设 | 标准标识 |
| --- | --- | --- |
| 下板→上板 | 下板CAN3接上板CAN2 | **0x80** |
| 上板→下板 | 上板CAN2接下板CAN3 | **0x81** |

这是板间链路标识，不改其他总线的电机标识。旧0x141/0x222/0x223已由本协议替代；0x50/0x51不是当前协议。上下板必须同时采用新格式。

## 5. 固定8字节格式

| 字节 | 内容 |
| --- | --- |
| 0 | 本消息类型的8位序号 |
| 1 bit7–4 | 版本，固定1 |
| 1 bit3–2 | 类型：0底盘命令，1底盘反馈，2云台遥控输入；3左键动作 |
| 1 bit1–0 | 本类型的状态或模式 |
| 2–7 | 本类型的数据 |

### 类型2：下板→上板云台输入，0x80

| 字节 | 内容 |
| --- | --- |
| 0 | 云台输入序号 |
| 1 | 失能0x18，手动使能0x19，自瞄请求且使能0x1B；bit0使能、bit1自瞄请求 |
| 2–3 | Yaw目标角速度，有符号小端int16，0.001 rad/s |
| 4–5 | Pitch目标角速度，有符号小端int16，0.001 rad/s |
| 6–7 | 保留，必须填0 |

每10 ms发送一次，包括零速度和失能状态。上板每1 ms对目标速度积分。

编码范围−32.768～32.767 rad/s是int16乘1000后的可表示范围，**不是电机限速**。编码器按四舍五入（恰好半格远离零）取整；非有限数值或编码范围外数值返回false，不静默裁剪，不更改输出。下板既有摇杆到速度映射保持由下板负责。

### 类型1：下板→上板底盘反馈，0x80

字节1为0x14加状态位：bit0 enabled、bit1 online；字节2–3为vx、4–5为vy、6–7为wz。
均为有符号小端int16；vx/vy单位0.001 m/s，wz单位0.001 rad/s。已有反馈按约10 ms周期发送。

### 类型0：上板→下板底盘命令，0x81

字节1为0x10加模式：0 ZERO_FORCE、1 NO_FOLLOW、2 FOLLOW_GIMBAL_YAW、3 ROTATE。
字节2–7依次是目标vx/vy/wz，类型、字节序及单位同反馈。解码后交给下板已有命令入口，保留现有底盘模式实现，不另写一套底盘控制。

## 6. 最小发送示例

把Lower_CanSendFrame接到下板CAN3的已有非阻塞发送入口。它必须在返回成功前复制帧内容或将帧提交给硬件；不能保存下述栈数组指针。成功表示本地接受提交，不等于对端已经收到或执行。

```c
#include "gimbal_link_protocol.h"

/* 下板Agent实现：使用下板CAN3，标准数据帧，复制8字节后非阻塞提交。 */
extern bool Lower_CanSendFrame(uint16_t standard_id, const uint8_t bytes[8]);

/* 仅由一个已有周期任务调用，正常每10 ms调用一次。 */
bool Lower_SendGimbalInput(float yaw_rad_s, float pitch_rad_s, bool enabled, bool autoaim_requested)
{
    static uint8_t sequence = 0U;
    const GimbalLink_RateInput input = {yaw_rad_s, pitch_rad_s, enabled, enabled && autoaim_requested};
    uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE];
    if (!GimbalLink_EncodeRateInput(&input, sequence, bytes))
        return false;
    if (!Lower_CanSendFrame(GIMBAL_LINK_UPLINK_ID, bytes))
        return false;
    ++sequence;
    return true;
}

/* 若已有底盘反馈，每10 ms调用；序号与云台输入独立。 */
bool Lower_SendChassisFeedback(const GimbalLink_ChassisFeedback* feedback)
{
    static uint8_t sequence = 0U;
    uint8_t bytes[GIMBAL_LINK_PAYLOAD_SIZE];
    if (!GimbalLink_EncodeChassisFeedback(feedback, sequence, bytes))
        return false;
    if (!Lower_CanSendFrame(GIMBAL_LINK_UPLINK_ID, bytes))
        return false;
    ++sequence;
    return true;
}
```

**两类消息共用0x80，不能都写入同一个仅按CAN标识保留最新值的周期槽。**
若下板沿用本工程BSP（Board Support Package，板级支持包），使用CAN_Tx_Submit逐帧提交；不要同时用CAN_Tx_Perform提交这两类消息。复用现有发送任务，不新增任务或自动重试层。提交失败时保留本类型序号，下个既有周期使用最新业务数据再次提交即可，不积攒应用层旧目标。

## 7. 接收与时效约定

- 下板按0x81过滤，在接收回调中保存完整报文及本地接收时间，交已有任务调用GimbalLink_DecodeChassisCommand；编解码函数本身不校验CAN标识，调用方必须校验方向。
- 各消息类型独立跟踪序号。GimbalLink_SequenceNewer判断无符号差1～127为新帧，支持255→0；重复或回退帧不发布新业务命令，也不刷新业务时间戳。
- 沿用既有Transport规则：合法及时帧中断超过100 ms后重建本类型序号基准；连续合法重复帧只证明通信活性，不能反复刷新旧命令。
- 上板遥控输入业务时效为50 ms，键鼠为100 ms。云台输入停止发送或只重复旧序号，会使该遥控来源过期；底盘反馈不能给云台输入续期。
- 保留实际接收时间，不把下板未同步时间直接当上板时钟，不在每次轮询中给旧包重新盖时间戳。
- 接入下板命令时沿用下板既有掉线/过期处理，不新增未获用户同意的保护行为。

## 8. 对照测试帧

| 标识 | 8字节数据 | 解释 |
| --- | --- | --- |
| 0x80 | `01 19 E8 03 0C FE 00 00` | 类型2，序号1，使能；Yaw+1、Pitch−0.5 rad/s |
| 0x80 | `02 18 00 00 00 00 00 00` | 类型2，序号2，失能，两轴0 |
| 0x80 | `03 17 C8 00 9C FF F4 01` | 类型1，序号3，enabled/online均真；vx+0.2、vy−0.1 m/s、wz+0.5 rad/s |
| 0x81 | `04 11 F4 01 00 00 18 FC` | 类型0，序号4，NO_FOLLOW；vx+0.5、vy0 m/s、wz−1 rad/s |

## 9. Agent验收清单

- 按本工程当前配置完成构建，不擅自更换板型或工具链。
- 独立C文件不依赖上板硬件库，C99编译通过；上述四组字节与编码结果一致。
- 正负速度、使能切换、序号255→0、非法长度/版本/类型/保留位均验证。
- 两类0x80在同一周期提交时都能进入发送通道，不互相覆盖；各自序号独立。
- 遥控有效性丢失时发送false/零值，不在下板积分云台角度。
- 0x81解码结果进入已有底盘入口，不扰动原电机所有权和调度。
- 清楚列出改动文件、接口调用位置、构建/测试结果；软件提交不当作实机通信已验收。


## 图传左键接收扩展（待下板接入）

本轮仅修改上板。下板后续复制更新后的gimbal_link_protocol.c/.h，保持原有类型0/1/2业务，
新增0x81类型3接收入口。8字节为“独立序号、0x1C、动作值、0、0、0、0、0”。
示例 `01 1C 01 00 00 00 00 00` 表示单次动作1，动作2表示持续，0表示停止。
上板已按双摩擦轮达速许可过滤1/2，右键自瞄期间不发送1/2。
1只发送一次且为离散事件，2与0是持续状态，每10ms更新，状态变化立即提交。
同类型独立跟踪序号，重复帧不重复执行单次动作。
既有下板收帧缓存只处理底盘命令，需为类型3单独分发和保存，不能覆盖类型0。
下板须沿用自身既有过期/掉线处理，不能永久锁存旧的2。上板提交成功仅表示入队。
