#include "Framing.hpp"

using namespace GoodBot;

uint32_t GoodBot::htonl(uint32_t inputInteger)
{
    constexpr uint32_t TEST_INTEGER = 0x1;

    if(((char*) &TEST_INTEGER)[0] == 1)
    {
        //It's little endian, so reverse
        uint32_t buffer = ((inputInteger << 8) & 0xFF00FF00 ) | ((inputInteger >> 8) & 0xFF00FF ); 
        return (buffer << 16) | (buffer >> 16);
    }
    else
    {
        return inputInteger;
    }
}

uint32_t GoodBot::ntohl(uint32_t inputInteger)
{
    constexpr uint32_t TEST_INTEGER = 0x1;

    if(((char*) &TEST_INTEGER)[0] == 1)
    {
        //It's little endian, so reverse
        uint32_t buffer = ((inputInteger << 8) & 0xFF00FF00 ) | ((inputInteger >> 8) & 0xFF00FF ); 
        return (buffer << 16) | (buffer >> 16);
    }
    else
    {
        return inputInteger;
    }
}

/**
This function escapes a character as needed, storing in the provided buffer and returning the number of characters in the buffer to use.
@param byte: The byte to encode
@param encodedBuffer: The encoded version of the byte (one or two bytes)
@return: The number of bytes in the buffer to use
*/
int8_t EscapeCharacter(char byte, std::array<char, 2>& encodedBuffer)
{
    if((byte == FlagByte) || (byte == EscapeByte))
    {
        encodedBuffer[0] = EscapeByte;
        encodedBuffer[1] = byte ^ XORByte;
        return 2;
    }

    encodedBuffer[0] = byte;
    return 1;    
}

char PPPEncoder::StartFrame()
{
    ResetCRC();
    ReturnedBytesSize = 1;
    return FlagByte;
}

std::array<char, 2> PPPEncoder::EncodeCharacter(char byte)
{
    ReturnedBytesSize = 0;
    std::array<char, 2> return_bytes;
    UpdateCRC(byte);
    ReturnedBytesSize = EscapeCharacter(byte, return_bytes);

    return return_bytes;
}

int8_t PPPEncoder::NumberOfEncodedCharactersToUse() const
{
    return ReturnedBytesSize;
}

std::array<char, MaxEndFrameLength> PPPEncoder::EndFrame()
{
    ReturnedBytesSize = 0;
    std::array<char, MaxEndFrameLength> return_bytes;

    //Add CRC, then flag byte
    FinalizeCRC();
    //Make sure byte order is right
    CRC_VALUE = htonl(CRC_VALUE);
    //Add CRC bytes after escaping
    std::array<char, 2> encode_buffer;
    for(int8_t crc_byte_index = 0; crc_byte_index < CRCSize; crc_byte_index++)
    {
        int8_t number_of_bytes = EscapeCharacter(((char *) &CRC_VALUE)[crc_byte_index], encode_buffer);
        for(int8_t byte_index = 0; byte_index < number_of_bytes; byte_index++)
        {
            return_bytes[ReturnedBytesSize] = encode_buffer[byte_index];
            ReturnedBytesSize++;
        }
    }

    return_bytes[ReturnedBytesSize] = FlagByte;
    ReturnedBytesSize++;

    ResetCRC();

    return return_bytes;
}

void PPPEncoder::ResetCRC()
{
    CRC_VALUE = crc_init();
}

void PPPEncoder::UpdateCRC(char frameByte)
{
    CRC_VALUE = crc_update(CRC_VALUE, &frameByte, sizeof(frameByte));
}

void PPPEncoder::FinalizeCRC()
{
    CRC_VALUE = crc_finalize(CRC_VALUE);
}
