#pragma once

#include<cstddef>
#include<cstdint>
#include<cstring>
#include<array>
#include "ch.h"

namespace GoodBot
{

//Queue messages are stored internally as a segment of memory from the memory pool with a uint32_t proceeding the message in the memory segment to indicate the message size.  A mailbox with msg_t objects casted as pointers transfers ownership of the memory from the producer to the consumer
template<size_t MaxNumberOfMessages, size_t MaxMessageSize>
class MessageQueue
{
public:
inline MessageQueue();

/**
This function inserts the given data into the queue.
@param data: The data to be placed into the queue
@param size: The size of the message to be placed in the queue
@return: true if the message could be inserted into the queue
*/
inline bool InsertMessage(const uint8_t* data, uint32_t size);

/**
This message does a blocking read on the message queue until the queue has a message available.
@param buffer: The buffer to place the read message into
@param bufferSize: The size of the buffer (max message size that can be placed)
@param timeoutInMicroseconds: How long to wait before giving up
@return: The size of the retrieved message or -1 on timeout
*/
inline int32_t RetrieveMessage(uint8_t* buffer, uint32_t bufferSize, uint32_t timeoutInMicroseconds);

/**
This non-blocking function how many messages are currently waiting to be processed.
@return: Number of messages stored in queue
*/
inline int32_t NumberOfMessagesAvailable();

protected:
const uint32_t MessageMemoryChunkSize;
std::array<uint8_t, MaxNumberOfMessages*(sizeof(uint32_t) + MaxMessageSize)> MemoryPoolBuffer;
memory_pool_t MessageMemoryPool;

std::array<msg_t, MaxNumberOfMessages> MemoryForMailbox;
mailbox_t Mailbox;
};

template<size_t MaxNumberOfMessages, size_t MaxMessageSize>
MessageQueue<MaxNumberOfMessages, MaxMessageSize>::MessageQueue() : MessageMemoryChunkSize((sizeof(uint32_t) + MaxMessageSize))
{
//Initialize memory pool to hold memory chunks from the associated array.  Memory pool is not allowed to grow after initialization
chPoolObjectInit(&MessageMemoryPool, MessageMemoryChunkSize, nullptr); 
chPoolLoadArray(&MessageMemoryPool, MemoryPoolBuffer.data(), MaxNumberOfMessages);

//Initialize mailbox to hold memory 
chMBObjectInit(&Mailbox, MemoryForMailbox.data(), MaxNumberOfMessages);
}

template<size_t MaxNumberOfMessages, size_t MaxMessageSize>
bool MessageQueue<MaxNumberOfMessages, MaxMessageSize>::InsertMessage(const uint8_t* data, uint32_t size)
{
    if(size > MaxMessageSize)
    {
        return false;
    }

    //Allocate memory for message
    uint8_t* message_memory = (uint8_t*) chPoolAlloc(&MessageMemoryPool);

    if(message_memory == nullptr)
    {
        return false;
    }

    //Copy message into allocated memory
    memcpy((char *) message_memory, &size, sizeof(uint32_t));
    memcpy((char *) message_memory+sizeof(uint32_t), data, size);

    bool post_succeeded = chMBPostTimeout(&Mailbox, (msg_t) message_memory, TIME_US2I(1000)) == MSG_OK;
    if(!post_succeeded)
    {
        chPoolFree(&MessageMemoryPool, (void *) message_memory);
    }

    //Put a pointer to the message in the mailbox
    return post_succeeded;
}

template<size_t MaxNumberOfMessages, size_t MaxMessageSize>
int32_t MessageQueue<MaxNumberOfMessages, MaxMessageSize>::RetrieveMessage(uint8_t* buffer, uint32_t bufferSize, uint32_t timeoutInMicroseconds)
{
    if(bufferSize < MaxMessageSize)
    {
        return -1; //Buffer too small
    }

    uint8_t* message_memory_pointer = nullptr;
    if(chMBFetchTimeout(&Mailbox, (msg_t *) &message_memory_pointer, TIME_US2I(timeoutInMicroseconds)) != MSG_OK)
    {
        return -2; //Read timed out or otherwise failed
    }
    
    uint32_t message_size = ((uint32_t*) message_memory_pointer)[0];
    if(message_size > MaxMessageSize)
    {
        chPoolFree(&MessageMemoryPool, (void *) message_memory_pointer);
        return -3; //Invalid message
    }

    memcpy((char *) buffer, message_memory_pointer + sizeof(uint32_t), message_size);
    chPoolFree(&MessageMemoryPool, (void *) message_memory_pointer);

    return (int32_t) message_size;
}

template<size_t MaxNumberOfMessages, size_t MaxMessageSize>
int32_t MessageQueue<MaxNumberOfMessages, MaxMessageSize>::NumberOfMessagesAvailable()
{
    return chMBGetUsedCountI(&Mailbox);
}

}
