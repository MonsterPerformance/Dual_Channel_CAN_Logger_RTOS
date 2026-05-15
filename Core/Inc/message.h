#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <stdint.h>
#include <stdbool.h>

const uint8_t BUFFERSIZE{64U};

typedef struct
{
    uint32_t timestamp;
    uint32_t ID;
        bool isExtended;
     uint8_t channelID;
     uint8_t filterID;
     uint8_t length;
     uint8_t payload[BUFFERSIZE];
} CMessage;

#endif
