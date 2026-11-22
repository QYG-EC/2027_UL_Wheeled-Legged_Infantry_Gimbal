#include "gimbal_tuning.h"

#include "bsp_usb.h"
#include <cstring>

namespace
{
constexpr uint16_t FRAME_SIZE = 56U;
constexpr uint8_t HEADER[] = {'G', 'T', '0', '1'};
uint8_t received_bytes[USB_BUFFER_SIZE];
uint16_t received_size;
uint8_t frame[FRAME_SIZE];
uint16_t frame_size;
Struct_Gimbal_Tuning_Request pending_request{};
bool request_pending;
uint8_t pending_response[FRAME_SIZE];
bool response_pending;
uint32_t transmit_failures;

uint32_t ReadU32(const uint8_t* bytes)
{
    return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8U) |
           (uint32_t(bytes[2]) << 16U) | (uint32_t(bytes[3]) << 24U);
}

void WriteU32(uint8_t* bytes, uint32_t value)
{
    for (unsigned index = 0U; index < 4U; ++index)
    {
        bytes[index] = uint8_t(value >> (8U * index));
    }
}

// CRC（Cyclic Redundancy Check，循环冗余校验）32/ISO-HDLC，初值与末异或均为全1。
uint32_t Checksum(const uint8_t* bytes, uint16_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t index = 0U; index < size; ++index)
    {
        crc ^= bytes[index];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

// 中断只复制本次字节；解析与校验留在普通优先级任务，不保留USB回调指针。
void Receive(uint8_t* bytes, uint16_t size)
{
    const uint16_t available = USB_BUFFER_SIZE - received_size;
    const uint16_t count = size < available ? size : available;
    std::memcpy(received_bytes + received_size, bytes, count);
    received_size += count;
}

void Feed(uint8_t byte)
{
    frame[frame_size++] = byte;
    while (frame_size >= sizeof(HEADER))
    {
        if (std::memcmp(frame, HEADER, sizeof(HEADER)) != 0)
        {
            --frame_size;
            std::memmove(frame, frame + 1U, frame_size);
            continue;
        }
        if (frame_size < FRAME_SIZE)
        {
            return;
        }
        if ((frame[4] == 1U || frame[4] == 2U) && frame[5] == 0U &&
            frame[6] == 0U && frame[7] == 0U &&
            ReadU32(frame + 52U) == Checksum(frame, 52U))
        {
            Struct_Gimbal_Tuning_Request request{};
            request.sequence = ReadU32(frame + 8U);
            request.operation = frame[4];
            std::memcpy(request.values, frame + 12U, sizeof(request.values));
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            pending_request = request;
            request_pending = true;
            __DMB();
            __set_PRIMASK(primask);
            frame_size = 0U;
            return;
        }
        --frame_size;
        std::memmove(frame, frame + 1U, frame_size);
    }
}
} // namespace

void GimbalTuning_Init()
{
    received_size = frame_size = 0U;
    request_pending = response_pending = false;
    transmit_failures = 0U;
    USB_Init(Receive);
}

void GimbalTuning_TransportUpdate()
{
    uint8_t bytes[USB_BUFFER_SIZE];
    uint8_t response[FRAME_SIZE];
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint16_t count = received_size;
    std::memcpy(bytes, received_bytes, count);
    received_size = 0U;
    const bool send_response = response_pending;
    if (send_response)
    {
        std::memcpy(response, pending_response, FRAME_SIZE);
        response_pending = false;
    }
    __DMB();
    __set_PRIMASK(primask);
    for (uint16_t index = 0U; index < count; ++index)
    {
        Feed(bytes[index]);
    }
    if (send_response && USB_Transmit_Data(response, FRAME_SIZE) != USBD_OK)
    {
        // 参数可能已生效；应答失败只记录，不重发、不回滚，主机随后显式查询。
        ++transmit_failures;
    }
}

bool GimbalTuning_TakeRequest(Struct_Gimbal_Tuning_Request& request)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool available = request_pending;
    if (available)
    {
        request = pending_request;
        request_pending = false;
    }
    __DMB();
    __set_PRIMASK(primask);
    return available;
}

void GimbalTuning_Reply(const Struct_Gimbal_Tuning_Request& request, uint8_t status, const float* values)
{
    uint8_t response[FRAME_SIZE]{};
    std::memcpy(response, HEADER, sizeof(HEADER));
    response[4] = request.operation | 0x80U;
    response[5] = status;
    WriteU32(response + 8U, request.sequence);
    std::memcpy(response + 12U, values, 10U * sizeof(float));
    WriteU32(response + 52U, Checksum(response, 52U));
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    std::memcpy(pending_response, response, FRAME_SIZE);
    response_pending = true;
    __DMB();
    __set_PRIMASK(primask);
}
