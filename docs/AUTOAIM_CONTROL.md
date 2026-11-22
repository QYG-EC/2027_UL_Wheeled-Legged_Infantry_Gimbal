# 自瞄控制链路与参数调整

本说明对应正式上板的双向24字节协议及板间配套源码。上下板固件已构建并通过主机联合回归。
本次合入的固件尚未烧录，视觉链路、双轴闭环及摩擦轮仍待实机联调。协议依据用户提供的《没有加mpc的串口协议_电控对接.md》。

## 1. 整体数据流

```text
下板遥控CH1/CH2、CH8总使能、CH7自瞄开关
  → remote_gimbal.cpp（10ms读取遥控并映射）
  → 下板CAN3 → 上板CAN2，0x80类型2
  → Remote_Gimbal_Input_Topic
                                      视觉 → 串口1（24字节目标）
                                              → Vision_ReadLatest
  → GimbalInput_Update（两路输入适配、键鼠控制权与自瞄请求）
  → SourceArbitration_Resolve（先判断操作者许可，再决定手动/自瞄）
  → RobotCmd_Update（唯一命令发布者）
  → Gimbal_Command_Topic
  → Gimbal_Update（1ms：目标计算、双轴转矩控制）
  → Pitch/Yaw电机

外置惯性单元 → Gimbal_INS_State_Topic
  → GimbalInput_Update（默认每10ms回传姿态、弹速、视觉mode、原姿态时戳）
  → 串口1 → 视觉
```

CAN（Controller Area Network，控制器局域网）板间速度为rad/s；
串口UART（Universal Asynchronous Receiver/Transmitter，通用异步收发器）使用921600 bit/s、
8数据位、无校验、1停止位、无流控。上板串口1发送PA9、接收PA10，连接对端收发引脚并共地。
串口采用已有非阻塞中断收发，未新增任务、动态内存、自动重连或协议重试。

## 2. 谁拥有云台控制权

Ctrl+S键鼠总开关默认关闭。关闭时遥控拥有控制权：

| 遥控健康 | CH8原始值 | CH7原始值 | 结果 |
| --- | --- | --- | --- |
| 否 | 任意 | 任意 | 撤销遥控使能、自瞄请求，云台失能 |
| 是 | ≤1400 | 任意 | 云台失能，自瞄不生效 |
| 是 | >1400 | ≤1400 | 手动速度积分控制 |
| 是 | >1400 | >1400 | 请求自瞄，有效目标接管，否则仍为手动 |

这里的原始值是接收机线上0～2047值，比较前已恢复驱动减去的1024中位。
CH7不是另一个使能入口，关闭CH8会立即撤销遥控自瞄和使能。

Ctrl+S开启后，键鼠按原有规则独立拥有控制权，CH7不参与仲裁，CH8仍只约束遥控来源。
鼠标右键按满300ms请求自瞄。松开右键或键鼠数据过期时撤销请求。
回传的`mode`只选择视觉识别对象和颜色，不表示手动/自瞄控制权，也没有退出自瞄值。键鼠数据过期仍保留控制权并输出零手动速度。
鼠标两轴默认灵敏度为0.0002 rad/计数。X正对应Yaw右转，Y正对应Pitch抬头。

切换操作者或自瞄开关会建立新的接收时间边界，旧视觉帧不跨边界使用，需等待切换时刻及之后的新完整帧。

## 3. 自瞄与手动的控制区别

手动模式`RATE`：每1ms执行`目标角 += 操作者角速度 × 0.001`，摇杆或鼠标提供角速度。

自瞄模式`IMU`：直接令目标角等于视觉提供的绝对角，不与当前角叠加、不重复积分。
这里的IMU（Inertial Measurement Unit，惯性测量单元）是工程已有模式枚举名称。
Yaw和Pitch绝对角必须与回传给视觉的姿态属于同一参考系，约定为`odom → gimbal_link`。
电控不自行估计视觉参考系与惯性参考系之间的零偏。

两种目标共用LQR（Linear Quadratic Regulator，线性二次型调节器）转矩控制：

```text
角度误差 = 目标角 − 惯性实际角
反馈转矩 = torque_sign × [k_angle × 角度误差 + k_speed × (参考角速度 − 电机换算角速度)]
最终输出 = 反馈转矩 + 原有重力/摩擦补偿，经原有转矩限制后发送
```

新协议没有本机参考速度或加速度，自瞄参考速度固定为0。
`target_v_yaw`是敌方目标自转速度，绝不能填入本机云台速度前馈。
Yaw取最短角误差；Pitch继续应用原有机械位置目标限制和输出约束。
控制参数不因切换自瞄而改变，当前位置取惯性反馈，两轴速度取各自电机反馈除以传动比。
首次就绪、掉线恢复后即使目标数值相同，也会重新应用当前有效的绝对角目标。

## 4. 无目标、超时与反馈

- `target_state=1`、距离>0且四个浮点字段均有限时才有有效目标。
- 收到完整无目标帧立即撤销参考，恢复手动，不把无目标的零角当作回零命令。
- 默认目标年龄≤1000ms可用，>1000ms恢复手动；仍请求自瞄时，新有效目标可再次接管。
- 自瞄转手动时，以当时实测姿态作为手动积分起点，再响应当前摇杆/鼠标，避免继续追赶最后视觉角。
- 没有合法新帧时不刷新接收时刻；错误帧沿用最后完整快照，直到配置的目标时效到期。
- 首份姿态前不回传；默认姿态年龄>1000ms停止回传。重复同份姿态保留原时戳，忙时跳过，不排队。
- 将某一视觉时效设为0可关闭该项超时：接收静默会保留最后有效角，反馈会允许旧姿态，但仍保留其原时间戳。
- 视觉时效独立于现有遥控50ms、键鼠100ms、控制惯性20ms及电机保护。放宽视觉时效不会放宽电机控制条件。
- `fire_advice`仅作解析结果，本次不生成射击命令或射击事件。

## 5. 在哪里调参数

以下路径均相对各自工程根目录；参数改完重新编译并烧录相应板，运行时接口除外。

| 参数/行为 | 文件及位置 | 当前值/修改方式 |
| --- | --- | --- |
| 视觉目标时效 | 上板`User_File/Application/Input/gimbal_input.h`，`Struct_Gimbal_Input_Config::vision_target_max_age_ms` | 1000ms；0关闭 |
| 视觉反馈姿态时效 | 同结构`vision_feedback_max_age_ms` | 1000ms；0关闭 |
| 反馈发送周期 | 同结构`vision_feedback_period_ms` | 10ms，必须>0；由1ms任务检查，发送忙可能跳周期 |
| 固定弹速 | 同结构`vision_bullet_speed_m_s` | 16.0m/s，用户指定，不代表本次实测标定 |
| 串口1波特率 | 上板`User_File/Device/Peripheral/Vision/vision.h`，`VISION_BAUD_RATE` | 921600；与视觉端同步修改 |
| 串口与引脚绑定 | 上板`User_Config/Board/gimbal_board_config.cpp`及`Core/Src/usart.c` | `huart1`，PA9/PA10；运行期使用上述波特率 |
| 视觉mode选取 | 上板`User_File/Application/Input/gimbal_input.cpp`，`UpdateVisionFeedback` | 己方红色编号1～11选1蓝装甲，其余选0红装甲；2～5仅协议支持 |
| CH7自瞄通道/阈值 | 下板`User_File/Application/Input/remote_gimbal.cpp`，`kAutoAimChannel/kAutoAimThreshold` | 零基索引6、原始值>1400 |
| CH8总使能通道/阈值 | 下板同文件，`kEnableChannel/kEnableThreshold` | 零基索引7、原始值>1400 |
| 手动速度/鼠标灵敏度 | 下板同文件`kMaxRateRadS`、上板输入配置`pitch_rad_per_count/yaw_rad_per_count` | 摇杆满量程3 rad/s，鼠标两轴0.0002 rad/计数 |
| 两轴增益、前馈及原有机械/转矩约束 | 上板`User_File/Application/Gimbal/Gimbal_Config.h`，`Gimbal_Default_Config`及配置结构 | 沿用当前工作区调参值，非视觉协议参数 |
| 键鼠自瞄长按时长 | 上板`User_File/Application/Input/gimbal_input.cpp`，`BUTTON_HOLD_MS` | 300ms，与左键长按判定共用 |
| 24字节字段偏移、帧头尾、编码方式 | 上板`User_File/Device/Peripheral/Vision/vision.cpp` | 双方协议契约，不能当作单边调参项 |
| 板间自瞄标志 | 两端`User_File/System/Transport/gimbal_link_protocol.c/.h` | 类型2状态bit1，必须同步修改两端 |

当前任务调用`GimbalInput_Init()`使用头文件默认配置；需要明确覆盖时可在
`User_File/Task/Control_Task_Gimbal.cpp`初始化前构造`Struct_Gimbal_Input_Config`并传入。
界面与控制共用目标时效，不再叠加旧的100ms界面门限。

## 6. 串口线上字段速查

字节编号从1开始，多字节字段为大端序，float32直接为物理值，无×100缩放或结构体填充。
浮点数采用IEEE（Institute of Electrical and Electronics Engineers，电气电子工程师学会）754单精度。

视觉→电控：

| 字节 | 字段 |
| --- | --- |
| 1 / 24 | FF / 0D |
| 2 | 开火建议 |
| 3～6 / 7～10 | yaw / pitch绝对角，rad，左转正 / 下俯正 |
| 11～14 | 距离，m |
| 15 | 目标编号 |
| 16～19 | 敌方自转速度，rad/s |
| 20 | target_state |
| 21～22 / 23 | 保留 / 预留校验 |

电控→视觉：

| 字节 | 字段 |
| --- | --- |
| 1 / 24 | FF / 0D |
| 2 | mode：0红装甲、1蓝装甲、2/3红/蓝小能量机关、4/5红/蓝大能量机关 |
| 3～6 / 7～10 / 11～14 | roll / pitch / yaw，rad；pitch下俯正，yaw左转正 |
| 15～18 | 弹速，float32，m/s |
| 19～22 | 同份姿态uint32毫秒时间戳 |
| 23 | 0，预留校验 |

当前没有CRC（Cyclic Redundancy Check，循环冗余校验）或其他载荷校验，不自行增加线上校验。
线路pitch与应用pitch反号；roll独立保留惯性roll。反馈时间戳是惯性快照本地主控接收时间近似，
不是发送时间或传感器内部采样时刻，重复快照不续期。

## 7. 板间格式、验证和后续实机方向

下板CAN3到上板CAN2，标准标识0x80类型2，8字节：序号、状态、两轴小端int16速度、两个零保留字节。
状态为0x18失能、0x19手动使能、0x1B使能并请求自瞄；bit0是使能，bit1是自瞄请求。
速度分辨率0.001rad/s，10ms周期及序号语义沿用原实现。
旧上板拒绝bit1置位的帧，因此上下板必须同步升级；视觉端也必须使用本次24字节版本。
图传鼠标动作的0x81类型3由上板发送。下板本地`main`已具备0x80类型2遥控上报，
类型3的接收和动作执行仍待接入，详见[下板交接](LOWER_BOARD_GIMBAL_HANDOFF.md)。

主机验证执行`python3 tests/run_gimbal_link.py`，运行真实两端协议、下板遥控适配及上板仲裁/控制，
硬件接口由桩替代；不能代替实际电气收发和运动验收。
当前配置分别执行上板`cmake --build build/GimbalBoard`和下板`cmake --build build/ChassisBoard`。

后续实机先在CH8关闭时核对24字节双向帧、反馈姿态/弹速/时戳和mode；
再核对CH7/CH8传输及视觉绝对角参考系，最后由操作者控制使能，验证小角度双轴跟踪、退出与丢目标回手动。
实机验收以新版本实际收帧和反馈记录为准。
