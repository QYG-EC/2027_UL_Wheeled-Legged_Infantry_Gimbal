# Shoot 发射应用开发指南

当前应用复用 DJI Device 的闭环与发送组。正式上板只控制两台摩擦轮，其他配置保留双摩擦轮和拨弹盘。
应用拥有设备实例、连续目标和事件角目标，ControlTask 负责初始化顺序与 1 ms 调度。
本文说明当前实现和移植位置；代码片段是现有实现的阅读示例，不是另一套发射控制框架。

## 正式上板图传摩擦轮

正式上板沿用BoardConfig的图传绑定识别两摩擦轮配置。总线CAN1，节点1正转、节点2反转，
M3508/C620直驱，传动比1。拨弹由下板执行，上板不注册节点8。
图传F键经Input与RobotCmd发布摩擦轮命令，默认目标25 rad/s（约239转/分钟）。
调节速度修改Shoot_Config.h的default_friction_speed_rad_s；
调节达速容差修改friction_ready_tolerance_ratio，默认0.05表示±5%。
Shoot_FrictionReady()在每1ms更新后提供双轮许可，要求总开关和摩擦轮开启、
两台驱动均ready且各自有符号逻辑速度误差不超过目标乘容差。没有额外保持时间。
本许可仅约束图传左键下发，达速前按压取消、长按超差暂停及恢复见[Input指南](../Input/README.md)。

以下拨弹流程及三电机示例适用于保留本地拨弹盘的配置；正式上板反馈enabled/online只考虑双摩擦轮。

## 文件与框架接口

| 文件/接口 | 职责与调用约定 |
| --- | --- |
| [Shoot.h](Shoot.h) | 提供Init、Update与FrictionReady达速查询 |
| [Shoot.cpp](Shoot.cpp) | 私有设备、命令消费、模式处理和反馈发布 |
| [Shoot_Config.h](Shoot_Config.h) | 机构角度、默认速度、节点编号和 PID 参考值 |
| `bool Shoot_Init(void)` | ControlTask 启动时调用一次；本板拥有的设备与发送组全部初始化成功返回 true |
| `void Shoot_Update(void)` | 在 RobotCmd_Update 后每 1 ms 调用，反馈每 10 ms 发布 |
| `BoardConfig_Get().shoot_bus` | 提供硬件总线，机构参数不放入 BoardConfig |

CMake 仅在 `H7_APP_SHOOT=ON` 时加入 Shoot.cpp；SingleBoard 默认关闭，GimbalBoard 固定开启，
ChassisBoard 固定关闭。关闭时任务不包含头文件、不调用初始化与更新，也不发布发射反馈；
RobotCmd 拒绝射击事件。应用内部保留完整实现，不提供空入口或替代反馈文件。
编入时任务仍忽略 Shoot_Init 的返回值，应用通过 initialized 跳过失败的硬件路径。

依赖约定见 [Application 指南](../README.md)、[Message Center](../../System/MessageCenter/README.md)
和 [DJI 驱动](../../Device/Peripheral/Motor/DJImotor/dji_motor.md)。

## 连续命令与离散动作

`MessageCenter::Shoot_Command_Topic` 保存最新的 ShootCmd；RobotCmd 是命令发布者，
Shoot 是消费者。RobotCmd 对 Shoot 按变化发布，不周期刷新同一状态。
Subscriber 仅在序号变化时 Read 成功，读取失败时本应用继续使用缓存命令。
**本应用没有命令年龄检查**；当前安全命令依赖 RobotCmd 的输入许可仲裁及失去许可时发布 OFF。移植时不能把 Read 成功当作新鲜度判断。

| 命令字段 | 当前处理 |
| --- | --- |
| `shoot_mode` | OFF 禁止三台电机输出、取消事件保持并清除本周期开始时积压的事件；ON 执行其余模式 |
| `friction_mode` | OFF 使用零速度目标；ON 使用正的命令速度，否则用默认 25 rad/s |
| `friction_speed_rad_s` | 两侧同号逻辑速度；右摩擦轮 reverse=true 处理镜像安装 |
| `loader_mode=BURST` | 使用速度外环，取消事件角保持；非零 loader_speed 优先，否则用射速乘单弹角 |
| `loader_mode=REVERSE` | 使用速度外环；非零命令取负绝对值，否则用默认反转速度 |
| `loader_mode=STOP` | 无事件时为零速度；有事件时切角度外环，持续保持累加角目标 |
| `shoot_rate_hz` | BURST 中正值优先，否则默认 10 弹/s；乘单弹角得到拨弹盘 rad/s |

`STOP` 不是禁止事件发射；它是离散事件的消费模式。摩擦轮 OFF 也不禁止拨弹，
当前代码没有“摩擦轮达速后才拨弹”的条件。

`Shoot_Event_Queue` 容量为 8，保存 ShootOnce/ShootTriple，必须经 RobotCmd 的事件接口提交。
拨弹盘稳定 ready 后，STOP 每周期最多取一个事件，首次从反馈累计角起步，后续在目标上增加 1 或 3 个弹位。
事件被取出仅表示已累加逻辑目标，不表示电机到位或弹丸已发射；事件可以在前一发尚未到位时继续累加。
正常 ready 时 BURST/REVERSE 不消费队列，积压事件会留到再次进入 STOP。
OFF、初始化失败、拨弹盘未 ready，以及首次恢复 ready 的周期，均按队列长度有界清空事件，
取消事件角度保持并清除旧角目标。恢复首周期也清队列，以丢弃上次检查后、恢复前进入的请求。
其后的新单发/三连发事件从当前反馈角开始；不需要额外切换总开关，旧请求不会自动补执行。
首次上电和 OFF→ON 同样遵循此规则；生产者应在就绪后重新提交事件，不应自动重放被丢弃的请求。
`RobotCmd_PushShootEvent()` 成功仍只表示入队，不保证请求会被执行。

该恢复规则针对离散动作。BURST/REVERSE 仍是持续命令：驱动未 ready 时保护输出，
恢复后按仍有效的当前模式继续运行，不要求新的命令序号。摩擦轮的运行策略保持不变。

## 参数、单位与设备绑定

| 配置 | 默认参考与含义 |
| --- | --- |
| 摩擦轮编号 | 左 3、右 2，M3508/C620，右侧逻辑反向 |
| 拨弹编号 | 8，M3508，未指定 gear_ratio，使用驱动默认减速比 19 |
| friction_gear_ratio | 1，摩擦轮直接使用电机转子，没有原厂减速箱 |
| default_friction_speed_rad_s | 25 rad/s，摩擦轮输出轴角速度；不等于弹丸初速 |
| one_bullet_angle_rad | 36° 转为约 0.62832 rad，拨弹盘每弹位角度 |
| default_rate_hz | 10 弹/s，与单弹角相乘得到约 6.28319 rad/s |
| reverse_speed_rad_s | -360°/s 转为约 -6.28319 rad/s |
| loader_angle_pid.output_limit | 2π rad/s，角度环给速度环的目标上限 |
| PID 周期 | 0.001 s，与控制任务 1 kHz 对应 |

摩擦轮只配置速度闭环；拨弹盘配置角度、速度、电流环，通过 Set_Outer_Loop 选择角度或速度入口。
PID 的输入输出单位由环位置决定：角度环输入 rad、输出 rad/s；拨弹速度环输出电流环目标，
电流环和摩擦轮速度环最终输出协议指令。配置中的增益和输出上限没有可靠实车整定记录，移植时按机构重新核对。

两个摩擦轮组成一个发送组，拨弹盘单独一个发送组。必须检查同一总线上每个 CAN 组的全部槽位，
不能与其他应用重复占用设备编号，也不能让多个逻辑组各自覆盖同一物理发送组。
更换电机时同步核对型号、减速比、反馈单位、逻辑方向、环配置和停止接口。

## 控制周期与反馈边界

```text
读取新序号命令（无新命令则保留缓存）
  → initialized 时读取三台电机快照
  → OFF / 未 ready / 恢复首周期时有界清理事件并取消旧角目标
  → Shoot_ApplyCommand：输出许可 → 摩擦轮速度 → 拨弹模式/事件目标 → 统一选择外环并提交
  → Shoot_UpdateFeedback：使用本周期控制前的快照
  → 每十周期发布 Shoot_Feedback_Topic
```

反馈速度为输出轴 rad/s，拨弹角为输出轴累计 rad。online 需要三个电机都在线；
enabled 需要总开关 ON 且三个快照均 ready，不代表摩擦轮已达到目标速度。
快照早于本周期使能/失能请求，所以许可变化可能下一周期才反映。
离线速度字段可能保留驱动最后一次值，消费者必须检查 online。

驱动负责单设备反馈超时与安全输出，应用负责拨弹盘未就绪时取消旧离散动作，
仍没有任一轮掉线后统一停三台的互锁。
当前组 Control/RequestEnabled 的返回值被忽略，反馈 enabled 也不证明目标成功发送或执行。
当前没有有限值参数校验、热量/裁判互锁、堵转检测、自动回退或完整发射状态机。
这些是新业务的实现位置，不应把现有事件保持误当作完整发射流程。

## 重新开发时处理哪些函数

| 内部函数/状态 | 移植时应处理的内容 |
| --- | --- |
| ShootContext | 私有静态设备、快照、命令和动作状态；不把临时设备放在 Update 的局部栈中 |
| Shoot_MakePID | 校对周期和各环输出单位；调度改变后同步 D_T |
| Shoot_ApplyCommand | 替换发射策略的主要位置；明确总开关、事件接受时机、模式切换和目标选择 |
| Shoot_UpdateFeedback | 保持字段单位与含义；新增达速/卡弹/动作完成信息先扩展消息契约 |
| Shoot_Init | 注册新设备、绑定发送组、初始化控制状态；避免构造阶段访问硬件 |
| Shoot_Update | 串联读命令、保护、控制和反馈；保留每周期有界事件处理和反馈分频 |

若重新开发达速互锁、命令超时或故障恢复，可把相关运行许可集中在 PrepareControl 中，
Update 调用一次后再运行主控制逻辑。该函数不是当前源码已有接口；加入时需明确等待期间
是否消费事件、恢复后是否丢弃旧目标、三台设备如何统一停止，避免在多个分支重复维护安全状态。

算法仍通过 Device 提交目标，不在 Application 中解析 CAN、调用 HAL 发送或管理 DMA。
若现有 ShootCmd/ShootFeedback 足够表达业务，保留字段和单位；增加字段时同步 RobotCmd、
输入仲裁、消息生产者和消费者，板间字段还需核对 Transport 的编码/解码。
新增源文件和设备驱动在根 CMake 显式登记，并检查 SingleBoard/GimbalBoard 源码选择。

## 按函数阅读当前代码

以下片段沿用生产实现，分别折叠展示；修改应用时以同目录 Shoot.cpp 为准。

<details>
<summary>Shoot_MakePID()：参数转换</summary>

```cpp
static PID_InitTypeDef Shoot_MakePID(const ShootPidConfig& config)
{
    PID_InitTypeDef pid{};
    pid.K_P = config.kp;
    pid.K_I = config.ki;
    pid.K_D = config.kd;
    pid.I_Out_Max = config.integral_limit;
    pid.Out_Max = config.output_limit;
    pid.D_T = 0.001f;
    return pid;
}
```

</details>

<details>
<summary>Shoot_ApplyCommand()：模式与事件目标</summary>

```cpp
static void Shoot_ApplyCommand(void)
{
    /* ShootMode 是总使能；关闭后摩擦轮和拨弹盘都停止主动输出。 */
    const bool enabled = ctx.command.shoot_mode == ShootMode::ON;
    (void) ctx.friction_group.RequestEnabled(enabled);
    (void) ctx.loader_group.RequestEnabled(enabled);
    if (!enabled) // 总开关关闭，取消事件角度保持并禁止所有主动输出。
    {
        ctx.event_angle_active = false;
        return;
    }

    float friction_reference_rad_s = 0.0f; // Friction OFF 是使能状态下的零速度，不等于总开关 OFF。
    if (ctx.command.friction_mode == FrictionMode::ON) // 摩擦轮开启时采用正速度目标，否则保持零速度目标。
    {
        friction_reference_rad_s = ctx.command.friction_speed_rad_s > 0.0f
                                       ? ctx.command.friction_speed_rad_s
                                       : kShootConfig.default_friction_speed_rad_s;
    }
    ctx.friction_group.Control(friction_reference_rad_s, friction_reference_rad_s);

    float loader_speed_target_rad_s = 0.0f;
    switch (ctx.command.loader_mode)
    {
    case LoaderMode::BURST:
    {
        // 连发以角速度控制，退出之前的事件角度保持；射速乘单弹角得到 rad/s。
        ctx.event_angle_active = false;
        const float rate = ctx.command.shoot_rate_hz > 0.0f
                               ? ctx.command.shoot_rate_hz
                               : kShootConfig.default_rate_hz;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
                                        ? ctx.command.loader_speed_rad_s
                                        : rate * kShootConfig.one_bullet_angle_rad;
        break;
    }

    case LoaderMode::REVERSE:
        ctx.event_angle_active = false;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
                                        ? -std::fabs(ctx.command.loader_speed_rad_s)
                                        : kShootConfig.reverse_speed_rad_s;
        break;

    case LoaderMode::STOP:
    default:
    {
        ShootEvent event;
        /* 每个 1 ms 周期最多取一个逻辑请求并累加目标角，不等待前一发物理完成。 */
        if (MessageCenter::Shoot_Event_Queue.Pop(event)) // STOP 模式有排队动作时，每周期只取一个请求。
        {
            if (!ctx.event_angle_active) // 还没有事件目标时，从当前拨弹盘累计角起步。
            {
                // 首次动作从当前反馈角起步；后续动作继续累加，避免覆盖排队的弹位。
                ctx.loader_angle_target_rad =
                    ctx.loader_snapshot.output_total_angle;
            }
            const float bullet_count =
                event.type == ShootEventType::ShootTriple ? 3.0f : 1.0f;
            ctx.loader_angle_target_rad +=
                bullet_count * kShootConfig.one_bullet_angle_rad;
            ctx.event_angle_active = true;
        }
        break;
    }
    }

    // 应用只选择目标和外环；角度/速度/电流 PID 及 CAN 发布复用 DJI 电机组接口。
    ctx.loader.Set_Outer_Loop(ctx.event_angle_active ? DJI_MOTOR_ANGLE_LOOP : DJI_MOTOR_SPEED_LOOP);
    ctx.loader_group.Control(ctx.event_angle_active ? ctx.loader_angle_target_rad : loader_speed_target_rad_s);
}
```

</details>

<details>
<summary>Shoot_UpdateFeedback()：设备快照到应用反馈</summary>

```cpp
static void Shoot_UpdateFeedback(void)
{
    ctx.feedback.friction_left_speed_rad_s =
        ctx.friction_left_snapshot.output_speed;
    ctx.feedback.friction_right_speed_rad_s =
        ctx.friction_right_snapshot.output_speed;
    ctx.feedback.loader_angle_rad = ctx.loader_snapshot.output_total_angle;
    ctx.feedback.loader_speed_rad_s = ctx.loader_snapshot.output_speed;
    ctx.feedback.enabled = ctx.command.shoot_mode == ShootMode::ON &&
                           ctx.friction_left_snapshot.ready &&
                           ctx.friction_right_snapshot.ready && ctx.loader_snapshot.ready;
    ctx.feedback.online = ctx.friction_left_snapshot.online &&
                          ctx.friction_right_snapshot.online &&
                          ctx.loader_snapshot.online;
}
```

</details>

<details>
<summary>Shoot_Init()：注册设备与发送组</summary>

```cpp
bool Shoot_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

    Struct_DJIMotor_Init_Config friction_config{};
    friction_config.hfdcan = BoardConfig_Get().shoot_bus;
    friction_config.motor_type = Enum_DJIMotor_Type::M3508;
    friction_config.gear_ratio = kShootConfig.friction_gear_ratio;
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入为 rad/s；增益无可信实车标定依据，启用前需重新整定。
    friction_config.speed_pid = Shoot_MakePID(kShootConfig.friction_speed_pid);

    friction_config.can_id = kShootConfig.friction_left_id;
    const bool left_initialized = ctx.friction_left.Init(friction_config);
    friction_config.can_id = kShootConfig.friction_right_id;
    friction_config.reverse = true; // 镜像安装：两侧使用同号逻辑速度，右侧由驱动反向。
    const bool right_initialized = ctx.friction_right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_bus;
    loader_config.can_id = kShootConfig.loader_id;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508; // 未指定 gear_ratio，使用驱动默认 M3508 减速比 19。
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(kShootConfig.loader_current_pid);
    loader_config.speed_pid = Shoot_MakePID(kShootConfig.loader_speed_pid);
    // 角度环输出是 rad/s；原 360 deg/s 限幅转换为 2π rad/s。
    loader_config.angle_pid = Shoot_MakePID(kShootConfig.loader_angle_pid);
    const bool loader_initialized = ctx.loader.Init(loader_config);

    ctx.initialized = left_initialized && right_initialized && loader_initialized &&
                      ctx.friction_group.Init(&ctx.friction_left, &ctx.friction_right) &&
                      ctx.loader_group.Init(&ctx.loader);
    if (ctx.initialized) // 全部设备和发送组绑定成功后，建立禁止输出的初始请求。
    {
        (void) ctx.friction_group.RequestEnabled(false);
        (void) ctx.loader_group.RequestEnabled(false);
    }
    ctx.loader_was_ready = false;
    ctx.event_angle_active = false;
    ctx.loader_angle_target_rad = 0.0f;
    return ctx.initialized;
}
```

</details>

<details>
<summary>Shoot_Update()：任务周期入口</summary>

```cpp
void Shoot_Update(void)
{
    /* 每个控制周期读取最新命令；没有新消息时继续执行上一帧。 */
    ShootCmd command;
    if (ctx.command_subscriber.Read(command)) // Topic 有可读值时更新缓存，读取成功不代表命令仍新鲜。
    {
        ctx.command = command;
    }

    if (ctx.initialized)
    {
        ctx.friction_left_snapshot = ctx.friction_left.GetMotionSnapshot();
        ctx.friction_right_snapshot = ctx.friction_right.GetMotionSnapshot();
        ctx.loader_snapshot = ctx.loader.GetMotionSnapshot();
    }
    const bool loader_ready = ctx.initialized && ctx.command.shoot_mode == ShootMode::ON &&
                              ctx.loader_snapshot.ready;
    if (!loader_ready || !ctx.loader_was_ready)
    {
        // OFF、未就绪和恢复首周期均取消旧动作；新事件从恢复后的反馈角重新起步。
        ctx.event_angle_active = false;
        ctx.loader_angle_target_rad = 0.0f;
        // 清除本周期开始时已有的事件，避免重新使能后补射；按队列快照限制循环次数。
        ShootEvent discarded_event;
        size_t pending_events = MessageCenter::Shoot_Event_Queue.Size();
        while (pending_events-- > 0U &&
               MessageCenter::Shoot_Event_Queue.Pop(discarded_event))
        {
        }
    }
    ctx.loader_was_ready = loader_ready;

    if (ctx.initialized) // 初始化成功才访问设备并运行硬件控制路径。
    {
        Shoot_ApplyCommand();
        Shoot_UpdateFeedback();
    }

    /* 控制按 1 kHz 更新，应用层反馈降频到 100 Hz。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U) // 每十个 1 ms 周期发布一次反馈。
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}
```

</details>

## 命令例程：连发与单发

下面是 RobotCmd 侧的目标与事件组织方式。当前 RobotCmd_Update 会用输入仲裁结果覆盖缓存，
因此应接入实际选中的输入映射路径；不能另建任务并发调用这些接口争用命令所有权。

<details>
<summary>展开：连发命令</summary>

```cpp
ShootCmd command;
command.shoot_mode = ShootMode::ON; // 允许发射机构主动输出。
command.friction_mode = FrictionMode::ON;
command.friction_speed_rad_s = 25.0f; // 摩擦轮输出轴目标，不是弹速。
command.loader_mode = LoaderMode::BURST;
command.shoot_rate_hz = 5.0f; // 5 弹/s，零 loader_speed 时由单弹角换算拨弹速度。
command.loader_speed_rad_s = 0.0f;
RobotCmd_SetShoot(command);
```

</details>

<details>
<summary>展开：单发请求与队列返回值</summary>

```cpp
ShootCmd command;
command.shoot_mode = ShootMode::ON;
command.friction_mode = FrictionMode::ON;
command.loader_mode = LoaderMode::STOP; // 离散事件只在 STOP 模式中消费。
RobotCmd_SetShoot(command);
ShootEvent event;
event.type = ShootEventType::ShootOnce;
bool accepted = RobotCmd_PushShootEvent(event); // 在按钮边沿调用一次，不能每 1 ms 重复提交。
if (!accepted) // Shoot 未编入、输入未获许可或 FIFO 已满；由输入层决定提示或后续重试。
{
    // 在调用方记录请求失败；不要在控制周期阻塞等待队列腾出空间。
}
```

</details>

## 移植验证顺序

1. 核对总线、编号、发送组所有权、反馈时效、转向与减速比。
2. 验证 Init 失败和总开关 OFF 路径；检查上层失联能持续发布 OFF。
3. 单独验证摩擦轮速度与拨弹盘角度/速度方向，再整定控制器。
4. 验证 STOP 单发/三连发累加、BURST/REVERSE 切换、OFF 清队列以及队列满返回值；验证掉线取消未完成角目标、
   掉线期间和恢复首周期丢弃事件、恢复后新事件从当前反馈角开始。
5. 检查反馈 online/ready、控制前快照延迟及离线字段语义；按实车需求补充互锁和恢复策略。

命令所有权、发布时序和反馈读取例程见 [RobotCmd 指南](../RobotCmd/README.md)。

文档和注释变更检查差异、链接及示例；控制逻辑变更构建 SingleBoard 与 GimbalBoard。
启用 SingleBoard 的发射路径可在专用构建目录配置 `H7_APP_SHOOT=ON`；默认 OFF 构建不能覆盖设备路径。
实机验证仍需覆盖电机闭环、CAN 负载、机构弹位、卡弹与发射许可条件。
