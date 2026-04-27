#ifndef CONSOL_LOGGER
#define CONSOL_LOGGER

#include <stdio.h>

extern "C"
{
    int __io_putchar(int ch)
    {
        HAL_UART_Transmit(&hlpuart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
        return ch;
    }
}

template <typename... Ts>
void log(const char *value, Ts... values)
{
    //printf(value, values...);
}

#endif
