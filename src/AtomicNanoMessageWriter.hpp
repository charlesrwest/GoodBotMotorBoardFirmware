#pragma once
#include "ch.h"
#include<array>
#include "MessageWriter.hpp"
#include "MessageQueue.hpp"

namespace GoodBot
{
const uint32_t OUTGOING_NANO_MESSAGE_QUEUE_MAX_MESSAGE_COUNT = 32;
const uint32_t MAX_OUTGOING_NANO_MESSAGE_BUFFER_MESSAGE_SIZE = 64;

//These global variables provide a mutex protected buffer to put things into to be sent over USB
extern MessageQueue<OUTGOING_NANO_MESSAGE_QUEUE_MAX_MESSAGE_COUNT, MAX_OUTGOING_NANO_MESSAGE_BUFFER_MESSAGE_SIZE> *GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE;

/**
This buffer performs a thread safe write into the global UART write buffer.
@param data: The data to write
@param dataSize: How many bytes to write (either all of it will be or none of it).
@return: If the write to the buffer completed successfully
*/
bool AtomicOutgoingNanoBufferWrite(const char *data, int32_t dataSize);

class AtomicNanoMessageWriter
{
public:
AtomicNanoMessageWriter();

void BeginTransaction(); //Clears any previously written data that has not been committed
bool CommitTransaction(); //Sends data in the buffer to the UART buffer after framing it

bool Write(int input);
bool Write(int64_t input);
bool Write(double input);
bool Write(const int64_t* intArray, int32_t arraySize);
bool Write(const double* doubleArray, int32_t arraySize);
bool Write(const float* floatArray, int32_t arraySize);
bool WriteString(const char *data, uint32_t size);
bool WriteBinary(const char *data, uint32_t size);
bool WriteMessageHeader(MessageType type, int64_t version = 0);

std::array<char, MAX_OUTGOING_NANO_MESSAGE_BUFFER_MESSAGE_SIZE> MsgpackBuffer;
MessageWriter Writer;
};

}


