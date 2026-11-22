# INS 状态发布与在线监控

`System_IMU_Configure()` 设置整机 VQF 参数并注册静态 INS Daemon，必须在 BMI088 Init 前调用；注册失败返回 false，System_Init 记录 BMI088/INS 链路不可用并禁止启动采集。

`System_IMU_Publish_State()` 只在 BMI088 已初始化且 Yaw/Pitch/Roll、三轴机体系角速度全部 finite 时 Feed 并发布 INS_State。NaN/Inf 不发布，也不刷新在线时间；原始 SPI/DMA chunk 不 Feed。因此 `System_IMU_IsOnline()` 监控整个采集、解算到有效 INS 输出链路。

监控门限为 30 ms，用于输出链路诊断；BMI088Task 按 FIFO 队列批量解算并在每批完成后发布一次，发布频率不等于陀螺仪采样频率；Gimbal 仍以 `INS_State_Topic.ReadFresh(..., 20000)` 判断用户确认的20 ms实时控制时效。二者不合并。StatusTask 100 Hz 统一 CheckAll，Daemon 不控制云台模式或整车安全策略；实现不使用动态内存。参见 [Daemon 说明](../Daemon/README.md)。


## 上板外置随动惯性单元

正式接口使用串口2，即485_2/P4，BoardConfig绑定`&huart2`，运行时921600、HI91。
串口3（485_1/P5）不承担正式云台外置惯性接收。

GimbalBoard调用`System_Gimbal_IMU_Init/Update`。Device层
[HIPNUC解析器](../../Device/Peripheral/IMU/hipnuc.h)从BoardConfig指定的串口2接收：
传感器RS232经转换模块接RS485，上板在接收启动前将CubeMX通用1M初值覆盖为921600，
保持原有8N1、RS485硬件方向和接收DMA配置，不发送设备配置/保存命令，不改写传感器参数。

用户已确认当前惯性单元输出HI91。解析器支持HI91固定76字节载荷与HI83位图载荷，共用5A A5帧封装；
CRC（Cyclic Redundancy Check，循环冗余校验）使用XMODEM，不能复用裁判协议CRC。
HI91角度/角速度为deg、deg/s；HI83角度为deg、角速度为rad/s，Device统一转为rad和rad/s。
HI83必须在同一帧提供角速度与欧拉角，未提供的帧不与上一帧拼接成新姿态。

System在控制任务中只消费新完整快照，并沿用既有六字段有限性发布条件，
以实际串口接收时间发布唯一静态`Gimbal_INS_State_Topic`；坐标轴符号留给Gimbal配置。
板载BMI088继续发布自己的`INS_State_Topic`，两者互不覆盖。
外置链路复用30ms诊断约定，Gimbal以用户确认的20ms业务时效读取；没有外置有效反馈时不会使用板载姿态代替。

协议依据：[厂商串行二进制协议](https://download.hipnuc.com/products/imu/cum/serial.html)。
当前输出类型已确认为HI91；串口2实测800ms内收到800帧有效姿态，期间串口错误和恢复计数不增长。
该结果验证接收链路，云台闭环运动仍需单独验收。
传感器和转换模块需匹配921600，当前代码不向设备发送改波特率或输出频率的命令。
