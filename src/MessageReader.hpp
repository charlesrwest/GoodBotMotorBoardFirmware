#include "CircularBuffer.hpp"
#include "cmp.h"
#include "Messages.hpp"
#include "Framing.hpp"

//#define RUNNING_TEST_ON_PC

#ifndef RUNNING_TEST_ON_PC
#include "ch.h"
#include "hal.h"
#else
    #include<istream>
#endif

namespace GoodBot
{

template<size_t BufferSize>
class MessageReader
{
public:
inline MessageReader();

/**
This function performs a blocking read with a circular buffer, making any messages found available for reading.
@param serialDriver: The serial driver to read from
@return: The number of frames decoded
*/
inline int64_t BlockingRead(SerialDriver& serialDriver, int32_t millisecondsToTimeout = -1);

/**
See if any message is available and if there is one, which type.
@return: The type of the next available message (INVALID if none)
*/
inline MessageType PeekNextMessageType() const;

/**
This function retrieves the next message from the internal buffer if it matches the given type and advances the buffer.
@param buffer: The buffer to store the message into
@return: true if it was able to retrieve the given message type and false otherwise
*/
inline bool GetMessage(SetPWMMessage& buffer);

/**
This function retrieves the next message from the internal buffer if it matches the given type and advances the buffer.
@param buffer: The buffer to store the message into
@return: true if it was able to retrieve the given message type and false otherwise
*/
inline bool GetMessage(SetTargetVelocityMessage& buffer);

protected:
void UpdateNextMessageType();

int32_t FeedDecoderAndForwardFrames(const char* data, int32_t dataSize);

SerialDriver* serialDriver;
PPPDecoder<BufferSize> FrameDecoder;
CircularBuffer<BufferSize> DecodedDataBuffer;
cmp_ctx_t Context;
MessageType NextMessageType = MessageType::INVALID;
};

template<size_t BufferSize>
bool CircleReader(cmp_ctx_t *inputContext, void* inputBufferToWriteTo, size_t inputMaxNumberOfBytes)
{
    return ((CircularBuffer<BufferSize>*) inputContext->buf)->Read((char *) inputBufferToWriteTo, inputMaxNumberOfBytes) == ((int32_t) inputMaxNumberOfBytes);
}

template<size_t BufferSize>
MessageReader<BufferSize>::MessageReader()
{
    cmp_init(&Context, &DecodedDataBuffer, CircleReader<BufferSize>, nullptr, nullptr);
}

template<size_t BufferSize>
int64_t MessageReader<BufferSize>::BlockingRead(SerialDriver& serialDriver, int32_t millisecondsToTimeout)
{
    int64_t number_of_bytes_read = 0;
    int64_t number_of_frames_decoded = 0;
    std::array<char, 8> read_buffer;    

    //Do blocking read until data is available
    int timeout_flag = millisecondsToTimeout < 0 ? TIME_INFINITE : TIME_MS2I(millisecondsToTimeout);
    number_of_bytes_read = chnReadTimeout(&serialDriver, (uint8_t*) read_buffer.data(), read_buffer.size(), timeout_flag);

    if(number_of_bytes_read <= 0)
    {
        return number_of_bytes_read;
    }

    number_of_frames_decoded += FeedDecoderAndForwardFrames(read_buffer.data(), number_of_bytes_read);
    
    //Do non-blocking reads until all data has been transferred to the appropriate buffers
    while(number_of_bytes_read > 0)
    {
        number_of_bytes_read = chnReadTimeout(&serialDriver, (uint8_t*) read_buffer.data(), read_buffer.size(), TIME_IMMEDIATE);

        if(number_of_bytes_read <= 0)
        {
            break;
        }

        number_of_frames_decoded += FeedDecoderAndForwardFrames(read_buffer.data(), number_of_bytes_read);
    }

    //Peek next integer in circular buffer if possible
    if(number_of_frames_decoded > 0)
    {
        UpdateNextMessageType();
    }

    return number_of_frames_decoded;
}

template<size_t BufferSize>
MessageType MessageReader<BufferSize>::PeekNextMessageType() const
{
    return NextMessageType;
}


template<size_t BufferSize>
bool MessageReader<BufferSize>::GetMessage(SetPWMMessage& buffer)
{
    if(PeekNextMessageType() != MessageType::SET_PWM)
    {
        return false;
    }

    buffer.Type = PeekNextMessageType();

    if(!cmp_read_long(&Context, &buffer.Version))
    {
        UpdateNextMessageType();
        return false;
    }

    if(!cmp_read_long(&Context, &buffer.Channel))
    {
        UpdateNextMessageType();
        return false;
    }

    if(!cmp_read_long(&Context, &buffer.OnTime))
    {
        UpdateNextMessageType();
        return false;
    }

    UpdateNextMessageType();
    return true;
}

template<size_t BufferSize>
bool MessageReader<BufferSize>::GetMessage(SetTargetVelocityMessage& buffer)
{
    if(PeekNextMessageType() != MessageType::SET_TARGET_VELOCITY)
    {
        return false;
    }

    buffer.Type = PeekNextMessageType();

    if(!cmp_read_long(&Context, &buffer.Version))
    {
        UpdateNextMessageType();
        return false;
    }

    if(!cmp_read_decimal(&Context, &buffer.Velocity))
    {
        UpdateNextMessageType();
        return false;
    }

    UpdateNextMessageType();
    return true;
}

template<size_t BufferSize>
void MessageReader<BufferSize>::UpdateNextMessageType()
{
    int64_t NextInteger = -1;
    NextMessageType = MessageType::INVALID;
    while(DecodedDataBuffer.GetNumberOfBytesToBeRead() > 0)
    {
        if(cmp_read_long(&Context, &NextInteger))
        {
            if((NextInteger != 0) && (NextInteger != 12))
            {
                continue;
            }
            else
            {
                NextMessageType = (MessageType) NextInteger;
                break;
            }
        }
    }
}

template<size_t BufferSize>
int32_t MessageReader<BufferSize>::FeedDecoderAndForwardFrames(const char* data, int32_t dataSize)
{
    int32_t frames_decoded = 0;
    for(int32_t data_index = 0; data_index < dataSize; data_index++)
    {
        if(FrameDecoder.DecodeByte(data[data_index]))
        {
            //A frame has been decoded, so forward it to the msgpack buffer if there is space
            if(DecodedDataBuffer.GetNumberOfBytesBeforeWrapAround() > FrameDecoder.DecodedDataLength())
            {
                DecodedDataBuffer.Write(FrameDecoder.DecodedData(), FrameDecoder.DecodedDataLength());
                frames_decoded++;
            }
            FrameDecoder.ClearDecodedData();  //Clear decoded frame after it's been processed
        }
    }
    return frames_decoded;
}

}
