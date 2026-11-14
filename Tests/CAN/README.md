# CAN / VESC 主机回归

在仓库根运行 `python3 Tests/CAN/run.py`，需要 Python 3、支持 UBSan 的 `cc` / `c++`。
产物位于 `build/host-can/`；脚本不改变固件板型、应用开关或 CMake cache。

编译真实 `bsp_can.c`、`daemon.cpp`、`vesc_motor.cpp`，仅模拟 HAL、CMSIS 队列、中断屏蔽和时间。
五组开关配置（全关、单独开启各路、全开）各执行路由、设备、容量和竞争场景。
覆盖标准帧兼容、同 ID 跨总线/帧类型隔离、关闭路拒绝扩展帧、九种命令、状态帧、失能/掉线、
部分初始化回滚、队列/槽满、HAL 失败重试及消费快照后目标被替换的竞争。

这不能替代目标板 CAN 电气、仲裁、时序、总线负载、电调反馈及电机安全验收。
