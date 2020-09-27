#include "sys.h"



int fputc(int ch,FILE *f)
{
	uint8_t str[1];
	str[0]=(uint8_t)ch;
	HAL_UART_Transmit(&huart1,str,sizeof(str),10);
	return ch;
}

