#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <stdint.h>
#include <stdbool.h>

#define payloadMaxSize 8
typedef struct
{
    uint32_t timestamp;
    uint32_t ID;
        bool isExtended;
     uint8_t channelID;
     uint8_t filterID;
     uint8_t length;
     uint8_t payload[payloadMaxSize];
} CMessage;

#endif
