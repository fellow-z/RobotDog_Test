#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "main.h"
#include "mpu6050.h"
#include "usart.h"
#include "delay.h"

#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "dmpmap.h"
#include "dmpKey.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


void myget_ms(unsigned long *time);
int fputc(int ch,FILE *f);
