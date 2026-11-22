#include "vision.h"

#include "bsp_uart.h"

#include <cmath>
#include <cstring>

namespace
{
constexpr uint16_t RECEIVE_SIZE = 24U;
uint8_t buffer[RECEIVE_SIZE];
uint16_t buffered;
Struct_Vision_Reference latest;
bool received;
UART_HandleTypeDef* vision_uart;

float ReadFloat(const uint8_t* bytes)
{
    const uint32_t bits = static_cast<uint32_t>(bytes[0]) << 24U |
                          static_cast<uint32_t>(bytes[1]) << 16U |
                          static_cast<uint32_t>(bytes[2]) << 8U | bytes[3];
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void WriteUint32(uint8_t* bytes, uint32_t value)
{
    bytes[0] = static_cast<uint8_t>(value >> 24U);
    bytes[1] = static_cast<uint8_t>(value >> 16U);
    bytes[2] = static_cast<uint8_t>(value >> 8U);
    bytes[3] = static_cast<uint8_t>(value);
}

void WriteFloat(uint8_t* bytes, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    WriteUint32(bytes, bits);
}

bool Decode(Struct_Vision_Reference& reference)
{
    if (buffer[0] != 0xFFU || buffer[23] != 0x0DU)
    {
        return false;
    }
    reference.yaw_rad = ReadFloat(buffer + 2U);
    reference.pitch_rad = -ReadFloat(buffer + 6U);
    reference.distance_m = ReadFloat(buffer + 10U);
    reference.target_id = buffer[14];
    reference.target_spin_rad_s = ReadFloat(buffer + 15U);
    if (!std::isfinite(reference.yaw_rad) || !std::isfinite(reference.pitch_rad) ||
        !std::isfinite(reference.distance_m) || !std::isfinite(reference.target_spin_rad_s))
    {
        return false;
    }
    reference.valid = buffer[19] == 1U && reference.distance_m > 0.0f;
    reference.fire_advice = reference.valid && buffer[1] == 1U;
    if (!reference.valid)
    {
        // 无目标必须发布撤销状态，不能继续暴露上一帧运动参考或开火建议。
        reference = {};
        reference.distance_m = -1.0f;
    }
    reference.received_ms = HAL_GetTick();
    reference.sequence = latest.sequence + 1U;
    return true;
}
} // namespace

bool Vision_Init(UART_HandleTypeDef* uart)
{
    vision_uart = nullptr;
    buffered = 0U;
    received = false;
    latest = {};
    if (uart == nullptr)
    {
        return false;
    }
    uart->Init.BaudRate = VISION_BAUD_RATE;
    if (HAL_UART_Init(uart) != HAL_OK)
    {
        return false;
    }
    UART_Init(uart, nullptr);
    if (!UART_InitInterruptRx(uart, Vision_Receive))
    {
        return false;
    }
    vision_uart = uart;
    return true;
}

void Vision_Receive(uint8_t* bytes, uint16_t size)
{
    for (uint16_t index = 0U; index < size; ++index)
    {
        if (buffered == 0U && bytes[index] != 0xFFU)
        {
            continue;
        }
        buffer[buffered++] = bytes[index];
        if (buffered != RECEIVE_SIZE)
        {
            continue;
        }
        Struct_Vision_Reference reference;
        if (Decode(reference))
        {
            latest = reference;
            received = true;
            buffered = 0U;
        }
        else
        {
            // 只有整帧失败后才搜索下一候选头，合法载荷中的FF/0D不截断帧。
            uint16_t start = 1U;
            while (start < RECEIVE_SIZE && buffer[start] != 0xFFU)
            {
                ++start;
            }
            buffered = RECEIVE_SIZE - start;
            std::memmove(buffer, buffer + start, buffered);
        }
    }
}

bool Vision_ReadLatest(Struct_Vision_Reference& reference)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool available = received;
    if (available)
    {
        reference = latest;
    }
    __DMB();
    __set_PRIMASK(mask);
    return available;
}

uint8_t Vision_SendFeedback(const Struct_Vision_Feedback& feedback)
{
    if (vision_uart == nullptr)
    {
        return HAL_ERROR;
    }
    if (feedback.mode > 5U || !std::isfinite(feedback.roll_rad) ||
        !std::isfinite(feedback.pitch_rad) || !std::isfinite(feedback.yaw_rad) ||
        !std::isfinite(feedback.bullet_speed_m_s))
    {
        return HAL_ERROR;
    }
    uint8_t bytes[24]{};
    bytes[0] = 0xFFU;
    bytes[1] = feedback.mode;
    WriteFloat(bytes + 2U, feedback.roll_rad);
    WriteFloat(bytes + 6U, -feedback.pitch_rad);
    WriteFloat(bytes + 10U, feedback.yaw_rad);
    WriteFloat(bytes + 14U, feedback.bullet_speed_m_s);
    WriteUint32(bytes + 18U, feedback.sampled_ms);
    bytes[23] = 0x0DU;
    return UART_Transmit_Nonblocking(vision_uart, bytes, sizeof(bytes));
}
