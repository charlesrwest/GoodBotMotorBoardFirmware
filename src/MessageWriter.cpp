#include "MessageWriter.hpp"

using namespace GoodBot;

namespace GoodBot
{

size_t WriterFunction(cmp_ctx_t *inputContext, const void* dataBuffer, size_t dataSize)
{
    MessageWriter& writer = *((MessageWriter*) inputContext->buf);

    if((writer.GetNumberOfBytesWritten() + dataSize) > writer.BufferSize)
    {
        return 0;
    }

    memcpy((writer.Buffer+writer.GetNumberOfBytesWritten()), (char *) dataBuffer, dataSize);
    writer.NumberOfBytesWritten += dataSize;
    
    return dataSize;
}

}

MessageWriter::MessageWriter(char *dataBuffer, size_t dataBufferSize) : Buffer(dataBuffer), BufferSize(dataBufferSize)
{
    cmp_init(&Context, this, nullptr, nullptr, GoodBot::WriterFunction);
}

int32_t MessageWriter::GetNumberOfBytesWritten()
{
    return NumberOfBytesWritten;
}

void MessageWriter::Clear()
{
    NumberOfBytesWritten = 0;
}

bool MessageWriter::Write(int64_t input)
{
    return cmp_write_integer(&Context, input);
}

bool MessageWriter::Write(double input)
{
    return cmp_write_decimal(&Context, input);
}

bool MessageWriter::Write(const int64_t* intArray, int32_t arraySize)
{
    bool write_successful = cmp_write_array(&Context, arraySize);
    if(!write_successful)
    {
        return false;
    }
    for(int32_t int_index = 0; int_index < arraySize; int_index++)
    {
        write_successful = write_successful && Write(intArray[int_index]);
    }
    return write_successful;
}

bool MessageWriter::Write(const double* doubleArray, int32_t arraySize)
{
    bool write_successful = cmp_write_array(&Context, arraySize);
    if(!write_successful)
    {
        return false;
    }
    for(int32_t float_index = 0; float_index < arraySize; float_index++)
    {
        write_successful = write_successful && Write(doubleArray[float_index]);
    }
    return write_successful;
}

bool MessageWriter::Write(const float* floatArray, int32_t arraySize)
{
    bool write_successful = cmp_write_array(&Context, arraySize);
    if(!write_successful)
    {
        return false;
    }
    for(int32_t float_index = 0; float_index < arraySize; float_index++)
    {
        write_successful = write_successful && Write((double) floatArray[float_index]);
    }
    return write_successful;
}

bool MessageWriter::WriteString(const char *data, uint32_t size)
{
    return cmp_write_str_v4(&Context, data, size);
}

bool MessageWriter::WriteBinary(const char *data, uint32_t size)
{
    return cmp_write_bin(&Context, data, size);
}

bool MessageWriter::WriteMessageHeader(MessageType type, int64_t version)
{
    bool write_worked = true;
    write_worked = write_worked && Write((int64_t) type);
    write_worked = write_worked && Write((int64_t) version);
    return write_worked;
}
