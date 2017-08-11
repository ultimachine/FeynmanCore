#ifndef main_h
#define main_h

// Includes Atmel CMSIS
#include <sam.h>

extern void loop();
extern void setup();
void mdelay(uint32_t ul_dly_ticks);

#endif