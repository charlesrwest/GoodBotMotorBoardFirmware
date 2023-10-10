#pragma once

#include<cstddef>
#include<cstdint>
#include<cstring>
#include "cmp.h"
#include "Messages.hpp"

namespace GoodBot
{

class MessageWriter
{
public:
/**
Initializes writer to write to target buffer without exceeding the size of the buffer.  Repeated writes are put into the buffer sequentially until clear is called.  The total number of bytes written since the last call of clear can be gotten from GetNumberOfBytesWritten().
@param dataBuffer: The buffer to write the msgpack bytes to
@param dataBufferSize: The size of the buffer to write to
*/
MessageWriter(char *dataBuffer, size_t dataBufferSize);

/**
This function returns the total number of bytes written to the buffer (from the start) since the last clear.
@return: The number of byte written in the buffer (and offset in the buffer to the next byte to be written)
*/
int32_t GetNumberOfBytesWritten();

/**
This function resets the number of bytes written to 0 and has the next write be at the beginning of the buffer again.
*/
void Clear();

/**
This function writes a msgpack integer to the buffer.
@param input: The integer to write
@return: true if it was successfully written and false otherwise
*/
bool Write(int64_t input);
bool Write(double input);

bool Write(const int64_t* intArray, int32_t arraySize);
bool Write(const double* doubleArray, int32_t arraySize);
bool Write(const float* floatArray, int32_t arraySize);
bool WriteString(const char *data, uint32_t size);
bool WriteBinary(const char *data, uint32_t size);


/**
This function writes the expected preface to send a message over USB to the main computer.
@param type: The type of message which is expected to follow
@param version: The version of the message which is expected to follow
@return: true if it was successfully written and false otherwise
*/
bool WriteMessageHeader(MessageType type, int64_t version = 0);

friend size_t WriterFunction(cmp_ctx_t *inputContext, const void* dataBuffer, size_t dataSize);

protected:
cmp_ctx_t Context;
char *Buffer;
size_t BufferSize;
int32_t NumberOfBytesWritten = 0;
};

}
