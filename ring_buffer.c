#include <assert.h>
#include "ring_buffer.h"


bool RingBuffer_Init(RingBuffer *ringBuffer, char *dataBuffer, size_t dataBufferSize) 
{
    assert(ringBuffer);
    assert(dataBuffer);
    assert(dataBufferSize > 0);

    if ((ringBuffer) && (dataBuffer) && (dataBufferSize > 0))
    {
        ringBuffer->dataBufferSize = dataBufferSize;
        ringBuffer->dataBuffer = dataBuffer;
        ringBuffer->dataBufferLength = 0;
        ringBuffer->dataBufferisEmpty = true;
        ringBuffer->dataBufferHead = 0;
        ringBuffer->dataBufferTail = 0;
        return true;
    }
    return false;
}

bool RingBuffer_Clear(RingBuffer *ringBuffer)
{
    assert(ringBuffer);

    if (ringBuffer)
    {
        ringBuffer->dataBufferisEmpty = true;
        ringBuffer->dataBufferLength = 0;
        ringBuffer->dataBufferHead = 0;
        ringBuffer->dataBufferTail = 0;
        return true;
    }
    return false;
}

bool RingBuffer_IsEmpty(const RingBuffer *ringBuffer)
{
    assert(ringBuffer);

    return ringBuffer->dataBufferisEmpty;
}

size_t RingBuffer_GetLen(const RingBuffer *ringBuffer)
{
    assert(ringBuffer);

    if (ringBuffer)
    {
        return ringBuffer->dataBufferLength;
    }
    return 0;
}

size_t RingBuffer_GetCapacity(const RingBuffer *ringBuffer)
{
    assert(ringBuffer);

    if (ringBuffer)
    {
        return ringBuffer->dataBufferSize;
    }
    return 0;
}


bool RingBuffer_PutChar(RingBuffer *ringBuffer, char c)
{
    assert(ringBuffer);

    if (ringBuffer && (ringBuffer->dataBufferLength < ringBuffer->dataBufferSize))
    {
        ringBuffer->dataBuffer[ringBuffer->dataBufferHead] = c;
        ringBuffer->dataBufferLength++;
        ringBuffer->dataBufferHead++;
        ringBuffer->dataBufferisEmpty = false;
        if(ringBuffer->dataBufferHead == ringBuffer->dataBufferSize)
        {
            ringBuffer->dataBufferHead = 0;
        }
        return true;
    }
    return false;
}

bool RingBuffer_GetChar(RingBuffer *ringBuffer, char *c)
{
    assert(ringBuffer);
    assert(c);

    if ((ringBuffer) && (c) && (!ringBuffer->dataBufferisEmpty))
    {
        *c = ringBuffer->dataBuffer[ringBuffer->dataBufferTail];
        ringBuffer->dataBufferLength--;
        ringBuffer->dataBufferTail++;
        if(ringBuffer->dataBufferLength == 0)
        {
            ringBuffer->dataBufferisEmpty = true;
        }
        if(ringBuffer->dataBufferTail == ringBuffer->dataBufferSize)
        {
            ringBuffer->dataBufferTail = 0;
        }
        return true;
    }
    return false;
}
