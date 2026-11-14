#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
    typedef void* osMessageQueueId_t;
    typedef struct
    {
        void* cb_mem;
        uint32_t cb_size;
        void* mq_mem;
        uint32_t mq_size;
    } osMessageQueueAttr_t;
    typedef enum
    {
        osOK,
        osErrorResource
    } osStatus_t;
    osMessageQueueId_t osMessageQueueNew(uint32_t, uint32_t, const osMessageQueueAttr_t*);
    osStatus_t osMessageQueuePut(osMessageQueueId_t, const void*, uint8_t, uint32_t);
    osStatus_t osMessageQueueGet(osMessageQueueId_t, void*, uint8_t*, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
