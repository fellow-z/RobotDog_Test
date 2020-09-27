#include "RX_28.h"
#include "usart.h"

void snycWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, uint16_t *nDat)
{
	uint8_t i;
	uint8_t nLen = 2;
	uint8_t tmp_position[2];
	uint8_t mesLen = ((nLen + 1)*IDN + 4);
	uint8_t checkSum = 0;
	uint8_t bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = 0x83;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	HAL_UART_Transmit(&huart2, bBuf, 7, 100);

	checkSum = 0xfe + mesLen + 0x83 + MemAddr + nLen;
	for (i = 0; i < IDN; i++) {
		HAL_UART_Transmit(&huart2, ID + i, 1, 100);
		tmp_position[0] = (nDat[i]>>8)&0xff;
		tmp_position[1] = nDat[i]&0xff;
		HAL_UART_Transmit(&huart2, tmp_position, nLen, 100);
		checkSum += ID[i];
		checkSum += tmp_position[0];
		checkSum += tmp_position[1];
	}
	checkSum = ~checkSum;
	HAL_UART_Transmit(&huart2, &checkSum, 1, 100);
}

void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t tmp[10];
	uint8_t msgLen = 2;
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if (nDat) {
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		HAL_UART_Transmit(&huart2, bBuf, 6, 10);

	}
	else {
		bBuf[3] = msgLen;
		HAL_UART_Transmit(&huart2, bBuf, 5, 10);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	if (nDat) {
		for (i = 0; i < nLen; i++) {
			CheckSum += nDat[i];
		}
		for (i = 0; i < nLen; i++) {
			tmp[nLen - 1 - i] = nDat[i];
		}
		HAL_UART_Transmit(&huart2, tmp, nLen, 10);
	}
	CheckSum = ~CheckSum;
	HAL_UART_Transmit(&huart2, &CheckSum, 1, 10);
}

uint8_t readBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
	uint8_t temp[10]={0};
	writeBuf(ID, MemAddr,&nLen , 1, 0x02);		//发送数据
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);		//接收使能
	HAL_UART_Receive(&huart2, temp, nLen+7, 50);		//接收数据
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);			//发送使能
	
	for(int i=0;i<nLen;i++)
	{
		nDat[i]=temp[5+nLen-i];
	}
	
	return 0;
}

void alarmSet(uint8_t ID,uint8_t dat)
{
	writeBuf(ID,0x12,&dat,1,0x03);
}

