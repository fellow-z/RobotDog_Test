#ifndef RX_28_H
#define RX_28_H

#include "stm32f4xx.h"

void snycWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint16_t *nDat);
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
uint8_t readBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen);
void alarmSet(uint8_t ID,uint8_t dat);

#endif
