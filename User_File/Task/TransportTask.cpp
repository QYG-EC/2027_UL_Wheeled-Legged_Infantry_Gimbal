/**
 * @file    TransportTask.cpp
 * @brief   Normal 优先级、约 1 ms 的 USB CDC 调试通信任务。
 * @details 初始化 USB，正常上板执行双轴调参通信，其余路径沿用原遥测；osDelay(1) 允许阻塞。
 *          不负责 System/Transport 的板间 CAN，也不解析遥控/裁判协议。
 * @todo    文件名与板间 Transport 易混淆，后续独立更名，本次保留任务符号。
 * @author  zzm
 * @version 1.2
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "usb_device.h"
#include "user_task.h"
#if H7_GIMBAL_RATE_INPUT
#include "gimbal_tuning.h"
#endif


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
#if H7_GIMBAL_RATE_INPUT
    GimbalTuning_Init();
#endif
    MX_USB_DEVICE_Init();
#if !H7_GIMBAL_RATE_INPUT
    EricTool_USB.Set_Data(3, (int) &Debug_IMU_Data.Euler_Yaw_rad,
                         (int) &Debug_IMU_Data.Euler_Pitch_rad,
                         (int) &Debug_IMU_Data.Euler_Roll_rad);
#endif
    for (;;)
    {
#if H7_GIMBAL_RATE_INPUT
        GimbalTuning_TransportUpdate();
#else
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
#endif
        osDelay(1);
    }
}
