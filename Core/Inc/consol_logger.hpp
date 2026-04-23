#ifndef CONSOL_LOGGER
#define CONSOL_LOGGER

#include <stdio.h>

template <typename... Ts>
void log(const char *value, Ts... values)
{
    //printf(value, values...);
}

#endif
