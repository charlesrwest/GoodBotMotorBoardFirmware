#include<cstddef>
#include<cstdint>
#include<cstring>

#include "AtomicNanoMessageWriter.hpp"

using namespace GoodBot;

namespace GoodBot
{

MessageQueue<OUTGOING_NANO_MESSAGE_QUEUE_MAX_MESSAGE_COUNT, MAX_OUTGOING_NANO_MESSAGE_BUFFER_MESSAGE_SIZE> *GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE = nullptr; //Initialized in the USB write thread

}

bool GoodBot::AtomicOutgoingNanoBufferWrite(const char *data, int32_t dataSize)
{
    if(GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE == nullptr)
    {
        return false; //Buffer not initialized yet
    }

    return GLOBAL_OUTGOING_NANO_MESSAGE_QUEUE->InsertMessage((const uint8_t*) data, dataSize);
}

AtomicNanoMessageWriter::AtomicNanoMessageWriter() : MsgpackBuffer(), Writer(MsgpackBuffer.data(), MsgpackBuffer.size())
{
}

void AtomicNanoMessageWriter::BeginTransaction()
{
    Writer.Clear();
}

bool AtomicNanoMessageWriter::CommitTransaction()
{
    return AtomicOutgoingNanoBufferWrite(MsgpackBuffer.data(), Writer.GetNumberOfBytesWritten());
}

bool AtomicNanoMessageWriter::Write(int input)
{
    return Write((int64_t) input);
}

bool AtomicNanoMessageWriter::Write(int64_t input)
{
    return Writer.Write(input);
}

bool AtomicNanoMessageWriter::Write(double input)
{
    return Writer.Write(input);
}

bool AtomicNanoMessageWriter::Write(const int64_t* intArray, int32_t arraySize)
{
    return Writer.Write(intArray, arraySize);
}

bool AtomicNanoMessageWriter::Write(const double* doubleArray, int32_t arraySize)
{
    return Writer.Write(doubleArray, arraySize);
}

bool AtomicNanoMessageWriter::Write(const float* floatArray, int32_t arraySize)
{
    return Writer.Write(floatArray, arraySize);
}

bool AtomicNanoMessageWriter::WriteMessageHeader(MessageType type, int64_t version)
{
    return Writer.WriteMessageHeader(type, version);
}

bool AtomicNanoMessageWriter::WriteString(const char *data, uint32_t size)
{
    return Writer.WriteString(data, size);
}

bool AtomicNanoMessageWriter::WriteBinary(const char *data, uint32_t size)
{
    return Writer.WriteBinary(data, size);
}

