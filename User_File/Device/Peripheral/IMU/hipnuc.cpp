#include "hipnuc.h"

#include "bsp_uart.h"
#include "sys_timestamp.h"
#include <cstring>

namespace
{
constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr uint16_t MAX_PAYLOAD = 512U;
uint8_t buffer[MAX_PAYLOAD + 6U];
uint16_t buffered;
Struct_Hipnuc_State latest;
bool received;

// HIPNUC 使用 XMODEM：初值0、多项式0x1021；不能使用裁判协议的CRC。
uint16_t Crc(uint16_t crc, const uint8_t* bytes, uint16_t size)
{
    for (uint16_t index = 0U; index < size; ++index)
    {
        crc ^= static_cast<uint16_t>(bytes[index]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U ? (crc << 1U) ^ 0x1021U : crc << 1U;
        }
    }
    return crc;
}

float FloatAt(const uint8_t* bytes)
{
    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

// 每次仅解码同一帧同时提供的角速度和姿态，避免拼接两个采样时刻。
bool Decode(const uint8_t* payload, uint16_t size, Struct_Hipnuc_State& state)
{
    if (payload[0] == 0x91U && size == 76U)
    {
        for (unsigned axis = 0U; axis < 3U; ++axis)
        {
            state.gyro_rad_s[axis] = FloatAt(payload + 24U + axis * 4U) * DEG_TO_RAD;
            state.angle_rad[axis] = FloatAt(payload + 48U + axis * 4U) * DEG_TO_RAD;
        }
    }
    else if (payload[0] == 0x83U && size >= 8U)
    {
        uint32_t bitmap;
        std::memcpy(&bitmap, payload + 4U, sizeof(bitmap));
        constexpr uint32_t KNOWN_FIELDS = 0x00200FFFU;
        constexpr uint32_t REQUIRED_FIELDS = (1U << 1U) | (1U << 3U);
        if ((bitmap & ~KNOWN_FIELDS) != 0U || (bitmap & REQUIRED_FIELDS) != REQUIRED_FIELDS)
        {
            return false;
        }
        constexpr uint8_t lengths[] = {12U, 12U, 12U, 12U, 16U, 8U, 8U, 4U, 4U, 12U, 12U, 12U};
        uint16_t offset = 8U;
        for (unsigned bit = 0U; bit <= 21U; ++bit)
        {
            if ((bitmap & (1U << bit)) == 0U)
            {
                continue;
            }
            const uint8_t length = bit == 21U ? 12U : lengths[bit];
            if (offset + length > size)
            {
                return false;
            }
            if (bit == 1U || bit == 3U)
            {
                for (unsigned axis = 0U; axis < 3U; ++axis)
                {
                    const float value = FloatAt(payload + offset + axis * 4U);
                    if (bit == 1U)
                    {
                        state.gyro_rad_s[axis] = value;
                    }
                    else
                    {
                        state.angle_rad[axis] = value * DEG_TO_RAD;
                    }
                }
            }
            offset += length;
        }
        if (offset != size)
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    state.status = static_cast<uint16_t>(payload[1]) | (static_cast<uint16_t>(payload[2]) << 8U);
    state.tag = payload[0];
    return true;
}

void Drop(uint16_t count)
{
    buffered -= count;
    std::memmove(buffer, buffer + count, buffered);
}
} // namespace

bool Hipnuc_Init(UART_HandleTypeDef* uart)
{
    if (uart == nullptr || uart->hdmarx == nullptr)
    {
        return false;
    }
    buffered = 0U;
    received = false;
    latest = {};
    UART_Init(uart, Hipnuc_Receive);
    return true;
}

void Hipnuc_Receive(uint8_t* bytes, uint16_t size)
{
    for (uint16_t index = 0U; index < size; ++index)
    {
        buffer[buffered++] = bytes[index];
        // 每轮要么等待剩余字节，要么丢弃至少1字节；工作量由帧长和本次chunk限定。
        while (buffered >= 6U)
        {
            const uint16_t length = static_cast<uint16_t>(buffer[2]) |
                                    (static_cast<uint16_t>(buffer[3]) << 8U);
            if (buffer[0] != 0x5AU || buffer[1] != 0xA5U || length == 0U || length > MAX_PAYLOAD)
            {
                Drop(1U);
                continue;
            }
            const uint16_t total = length + 6U;
            if (buffered < total)
            {
                break;
            }
            const uint16_t crc = Crc(Crc(0U, buffer, 4U), buffer + 6U, length);
            const uint16_t expected = static_cast<uint16_t>(buffer[4]) |
                                      (static_cast<uint16_t>(buffer[5]) << 8U);
            if (crc != expected)
            {
                Drop(1U);
                continue;
            }
            Struct_Hipnuc_State state{};
            if (Decode(buffer + 6U, length, state))
            {
                state.received_us = SYS_Timestamp_Get_Microsecond();
                state.sequence = latest.sequence + 1U;
                latest = state;
                received = true;
            }
            Drop(total);
        }
    }
}

bool Hipnuc_ReadLatest(Struct_Hipnuc_State& state)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool available = received;
    if (available)
    {
        state = latest;
    }
    __DMB();
    __set_PRIMASK(primask);
    return available;
}
