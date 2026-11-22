#ifndef GIMBAL_TUNING_H
#define GIMBAL_TUNING_H

#include <cstdint>

/** USB调参数据：每轴依次为角度增益、速度增益、重力常数、重力斜率、运动摩擦。 */
struct Struct_Gimbal_Tuning_Request
{
    uint32_t sequence;
    uint8_t operation; // 1写入、2查询；不表达电机使能或目标。
    float values[10];  // Yaw在前，Pitch在后；线上为小端binary32。
};

/** @brief USB设备启动前绑定唯一接收回调，仅由TransportTask调用一次。 */
void GimbalTuning_Init();
/** @brief TransportTask每1ms调用：解析接收流，非阻塞发送一次应答；不自动重发。 */
void GimbalTuning_TransportUpdate();
/** @brief ControlTask取出完整请求；同一主机只允许一个在途请求。
 * @param request 接收请求的副本。 @return 是否取到请求。
 */
bool GimbalTuning_TakeRequest(Struct_Gimbal_Tuning_Request& request);
/** @brief ControlTask提交读回值；0成功、1参数拒绝、2应用未初始化。
 * @param request 原请求及对应序号。 @param status 执行结果。
 * @param values 当前两轴实际参数，共10个float。
 */
void GimbalTuning_Reply(const Struct_Gimbal_Tuning_Request& request, uint8_t status, const float* values);

#endif
