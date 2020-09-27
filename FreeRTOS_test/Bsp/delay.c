#include "delay.h"

#include "main.h"

void delay_ms(u16 nms)
{
	HAL_Delay(nms);
}
