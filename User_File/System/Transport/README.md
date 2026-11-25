# 双板固定通信协议

当前协议使用CAN（Controller Area Network，控制器局域网）标准数据帧，1 Mbit/s，固定8字节。
下板工程Agent的完整字段清单、独立C接口、发送示例和验收要求见
[下板对接任务](../../../docs/LOWER_BOARD_GIMBAL_HANDOFF.md)。两份源码
[gimbal_link_protocol.h](gimbal_link_protocol.h)/[gimbal_link_protocol.c](gimbal_link_protocol.c)
是上下板共用的C99/C++编解码实现，C++的TransportProtocol只做现有业务类型转换。

## 构建与板级接线

| 板型 | 本地应用 | 电机总线 | 板间总线 |
| --- | --- | --- | --- |
| GimbalBoard | RobotCmd、Gimbal、Shoot | Pitch CAN1、Yaw CAN2，双摩擦轮CAN1 | CAN2（与Yaw共用） |
| ChassisBoard | Chassis | 车轮CAN1，转向CAN2 | CAN3 |

沿用当前选中的构建目录验证；不为通信修改自动切换板型、应用或Debug/Release配置。
BoardConfig绑定硬件，TransportConfig绑定固定方向标识；Transport不是动态Router。

## 线上格式

- 下板→上板：标准标识 **0x80**。
- 上板→下板：标准标识 **0x81**。
- 旧0x141/0x222/0x223退出本链路，0x50/0x51不使用，双方必须同时升级。

| 字节 | 内容 |
| --- | --- |
| 0 | 本消息类型的8位序号 |
| 1 bit7–4 | 版本1 |
| 1 bit3–2 | 消息类型0、1、2或3 |
| 1 bit1–0 | 本类型的状态/模式 |
| 2–7 | 数据 |

| 类型 | 方向 | 状态/模式 | 数据 |
| --- | --- | --- | --- |
| 0 底盘命令 | 0x81 | 模式0～3 | vx/vy/wz各2字节 |
| 1 底盘反馈 | 0x80 | bit0 enabled，bit1 online | vx/vy/wz各2字节 |
| 2 云台遥控输入 | 0x80 | bit0 enabled，bit1 autoaim_requested | Yaw/Pitch各2字节，末2字节为0 |

所有速度编码为有符号小端int16，平移0.001 m/s、旋转0.001 rad/s。
Yaw正为左转，Pitch正为抬头；−32.768～32.767是编码容量，不是机构限速。
C接口拒绝非有限值和编码范围外的值，编码失败不修改输出。帧内不增加软件校验字段。
例如云台使能、序号1、Yaw+1/Pitch−0.5 rad/s：`01 19 E8 03 0C FE 00 00`。

## 收发与接口

上板仅向CAN2注册一个0x80回调，整数解析消息类型后分别复制两份完整帧与接收时间；
控制任务解码，再分别发布`Chassis_Feedback_Topic`与`Remote_Gimbal_Input_Topic`。
两类各自缓存、各自跟踪序号，连续收到的两类消息不能相互覆盖。
GimbalInput消费遥控速度，RobotCmd仍是命令唯一发布者，Gimbal按1ms积分目标。

上板底盘命令每10ms更新，使用现有CAN_Tx_Perform最新值槽，标识0x81。
下板底盘反馈与云台输入各约10ms逐帧提交，标识均为0x80；本仓库下板反馈改用
现有CAN_Tx_Submit，防止同标识两类帧在最新值槽中互相覆盖。软件接受提交后
本类型序号才递增；入队不等于硬件发送、对端接收或设备执行。下板遥控业务由下板Agent接入。

本协议只传连续目标/反馈，无确认应答、会话标识、额外心跳或分包，不承载不可丢失的离散事件。
C接口只编解码，调用方校验CAN标识/总线并负责发送周期、序号及接收时效。

## 链路在线与业务时效

保持原有100ms静态Daemon诊断。按实际接收时间仍及时、Decode合法的帧先Feed，再按
本类型独立序号判断是否Publish；合法重复帧可保持链路在线，但不能给业务数据续期。
合法及时流中断超过100ms后，本类型可重建序号基准；连续合法重复流不重建。
没有会话标识，不能宣称具备跨重启防重放能力。

Topic保留实际接收时间。Input的Remote业务时效继续为50ms，Keyboard为100ms，
底盘命令业务时效继续为100ms；这些不等同于链路Online。未知类型、错误版本/长度/保留位
不发布对应业务消息。未新加超时、锁存、自动恢复或重试策略。

## 验证

检查C99独立编码及C++业务包装的固定字节、往返、正负值、序号回绕和错误帧；
同一0x80连续到达两类报文后，两份Topic必须各保留正确数据与实际接收时间。
本机只构建当前选定的上板配置；下板业务与真实总线收发需在对端接入后联调。
`TransportTask.cpp`仍是USB遥测任务，板间轮询仍是ControlTask中的BoardTransport_Poll。

上下板主机集成验证可执行`python3 tests/run_gimbal_link.py`，默认使用同级
`2027_UL_Wheeled-Legged_Infantry_Chassis`，也可用`--lower <路径>`指定。
该验证编译两端真实Transport、下板遥控适配、上板输入/RobotCmd/Gimbal与24字节视觉源码，
硬件接口以桩替换；不能代替真实总线、电机与控制周期验收。

## CH8许可下的CH7自瞄请求

类型2状态bit1表示`autoaim_requested`，bit0保持`enabled`。
下板只有遥控健康且CH8原始值>1400、CH7原始值>1400时才置位请求；关闭CH8同时撤销两者。
状态0x18失能、0x19手动使能、0x1B使能并请求自瞄；其余字节及版本不变。
旧上板会拒绝bit1置位的帧，两端必须同步更新`gimbal_link_protocol.c/.h`与业务结构。


## 图传左键下发（类型3）

上板CAN2→下板CAN3，标准标识0x81，版本1。字节0为本类型独立序号，
字节1固定0x1C，字节2为0停止、1单次事件、2持续动作，字节3–7为0。
GimbalLink_EncodeMouseCommand/DecodeMouseCommand提供独立C接口。
BoardTransport_SendMouseCommand使用已有非阻塞发送队列，成功后递增序号。
输入层在动作变化时立即提交，0/2每10ms更新，1仅一次；提交失败不补发单次事件。
现有底盘命令仍用原周期槽，两类型不能共用仅按标识保存最新帧的槽。
下板当前尚未接入类型3，需按交接文档更新后才能消费本业务。

## 云台惯性Yaw下发（0x82）

为底盘跟随提供数据；本次只完成数据链路，未接入跟随控制器。
使用CAN（Controller Area Network，控制器局域网）标准数据帧、1 Mbit/s、8字节，
上板CAN2→下板CAN3，独立标识`0x82`，每1ms控制周期提交一次（目标1000Hz）。

| 字节位置 | 字段 | 编码 | 单位 | 范围/方向 |
| --- | --- | --- | --- | --- |
| 0–3 | yaw_rad，云台惯性Yaw角度 | 小端IEEE 754（Institute of Electrical and Electronics Engineers 754）32位浮点 | rad（弧度） | [-π, π]，左转为正 |
| 4–7 | yaw_speed_rad_s，惯性Z轴角速度 | 同上 | rad/s（弧度每秒） | 左转为正，不额外限幅 |

例如Yaw=1 rad、角速度=-2.5 rad/s时，载荷为`00 00 80 3F 00 00 20 C0`。
该标识直接承载两个浮点数，没有旧格式的版本、类型、序号或软件校验头；
0x80/0x81线上格式不变。独立C接口为`GimbalLink_EncodeYawState/DecodeYawState`。

上板`BoardTransport_SendGimbalYawState`读取外置IMU（Inertial Measurement Unit，惯性测量单元）
的`Gimbal_INS_State_Topic`，取同一快照的`yaw_rad`与`gyro_z_rad_s`，不取电机速度，
也不以板载姿态替代。角度归一化到[-π, π]。沿用20ms时效（恰好20ms仍有效），
未发布或过期不再提交，恢复新鲜数据后继续；与遥控使能及电机就绪状态无关。
每1ms发送可包含重复的仍新鲜样本，不等于传感器每次产生新样本。

发送使用`CAN_Tx_Perform`独立最新值槽，后来的快照覆盖尚未发送的旧版本，
不占用鼠标动作队列。周期槽只处理更新过的版本，成功发送后不会无限重发；
提交失败不额外补发，下一控制周期重新读取当前数据。
复用底层已有发送行为，不新增重试或超时撤回；已在软件槽或硬件中待发的一帧不会因源过期自动撤销。
软件提交不保证准确到达周期，真实频率、负载和延迟需实机验收。

下板独立回调缓存完整帧与实际接收时间，在`BoardTransport_Poll`中解码，
仅向`MessageCenter::Gimbal_Yaw_State_Topic`发布，不覆盖本板惯性姿态或云台电机反馈。
长度非8字节、非有限数不发布，也不刷新数据时间；保留最近一次有效值及时间元数据。
Topic时间是本板CAN接收时间，协议不携带远端采样时间，不能据此保证传感器端到端年龄。
下板不增加业务超时、控制回退或零位对齐；后续跟随控制器应明确消费时效及参考系。
现有链路Daemon诊断保持原职责，不由新Yaw帧刷新。
