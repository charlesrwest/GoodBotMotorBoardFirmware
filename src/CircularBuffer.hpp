#pragma once

#include<array>

template<size_t BufferSize>
class CircularBuffer
{
public:
    /**
    This function reads up to inputMaxNumberOfCharactersToRead from the circular buffer and increments the internal index so that the same data will not be read again.
    @param inputDataBuffer: The buffer to write the data from this buffer to
    @param inputMaxNumberOfCharactersToRead: The maximum number of bytes that the destination can hold right now
    @return: How many bytes were transferred
    */
    int32_t Read(char *inputDataBuffer, int32_t inputMaxNumberOfCharactersToRead)
    {
        int32_t offset = 0;

        inputMaxNumberOfCharactersToRead = std::min<int32_t>(inputMaxNumberOfCharactersToRead, GetNumberOfBytesToBeRead());

        for(; offset < inputMaxNumberOfCharactersToRead; offset++)
        {
            inputDataBuffer[offset] = serializationReadWriteBuffer[readIndex];
            IncrementIndex(readIndex); //Go up and then wrap around
        }

        return offset;
    }

    /**
    This function reads up to inputMaxNumberOfCharactersToRead from the circular buffer but does not increment the internal read index so that the same data will be read again.
    @param inputDataBuffer: The buffer to write the data from this buffer to
    @param inputMaxNumberOfCharactersToRead: The maximum number of bytes that the destination can hold right now
    @return: How many bytes were transferred
    */
    int32_t StationaryRead(char *inputDataBuffer, int32_t inputMaxNumberOfCharactersToRead)
    {
        int32_t offset = 0;

        inputMaxNumberOfCharactersToRead = std::min<int32_t>(inputMaxNumberOfCharactersToRead, GetNumberOfBytesToBeRead());

        int32_t readIndexBuffer = readIndex;

        for(; offset < inputMaxNumberOfCharactersToRead; offset++)
        {
            inputDataBuffer[offset] = serializationReadWriteBuffer[readIndexBuffer];
            IncrementIndex(readIndexBuffer); //Go up and then wrap around
        }

        return offset;
    }

    /**
    This function throws away any data that is in the buffer to be read.
    */
    void ClearReadBuffer()
    {
        readIndex = writeIndex;
    }

    //Returns number written (can lap read index)
    int32_t Write(const char *inputDataBuffer, int32_t inputNumberOfCharactersToWrite)
    {
        int32_t offset = 0;

        inputNumberOfCharactersToWrite = std::min<int32_t>(inputNumberOfCharactersToWrite, size());

        for(; offset < inputNumberOfCharactersToWrite; offset++)
        {
            serializationReadWriteBuffer[writeIndex] = inputDataBuffer[offset];
            IncrementIndex(writeIndex); //Go up and then wrap around
        }

        return offset;
    }

    int32_t GetNumberOfBytesToBeRead() const
    {
        return writeIndex < readIndex ? (size() - readIndex) + writeIndex : writeIndex - readIndex;
    }

    int32_t GetNumberOfBytesBeforeWrapAround() const
    {
        return size() - GetNumberOfBytesToBeRead(); 
    }



    int32_t size() const
    {
        return serializationReadWriteBuffer.size();
    }

    protected:
    //Returns number read
    static void IncrementIndex(int32_t& inputIndexToIncrement)
    {
        inputIndexToIncrement = (inputIndexToIncrement + 1) % BufferSize;
    }

    int32_t writeIndex = 0; //current location to write to
    int32_t readIndex = 0; //current byte to be read to 
    std::array<char, BufferSize> serializationReadWriteBuffer;
}; 
