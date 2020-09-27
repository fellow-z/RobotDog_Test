#include "ioi2c.h"
#include "sys.h"
#include "delay.h"

#define IIC_SDA_H() HAL_GPIO_WritePin(IIC_GPIO, SDA, GPIO_PIN_SET)
#define IIC_SDA_L() HAL_GPIO_WritePin(IIC_GPIO, SDA, GPIO_PIN_RESET)
#define IIC_SCL_H() HAL_GPIO_WritePin(IIC_GPIO, SCL, GPIO_PIN_SET)
#define IIC_SCL_L() HAL_GPIO_WritePin(IIC_GPIO, SCL, GPIO_PIN_RESET)
#define IIC_SDA_Read() HAL_GPIO_ReadPin(IIC_GPIO, SDA)

void IIC_Delay(uint16_t time)
{
	uint16_t count;
	count = time * 6;
	while(count --);
}

void IIC_SDA_Out(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IIC_GPIO, &GPIO_InitStruct);
}

void IIC_SDA_In(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = SDA;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_GPIO, &GPIO_InitStruct);
}

/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{			
	
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
int IIC_Start(void)
{
	IIC_SDA_Out();
	IIC_SDA_H();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SDA_L();
	IIC_Delay(1);
	IIC_SCL_L();
	
	return 1;
}
/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
	IIC_SDA_Out();
	IIC_SCL_L();
	IIC_SDA_L();
	IIC_Delay(1);
	IIC_SCL_H();
	IIC_SDA_H();
	IIC_Delay(1);
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
int IIC_Wait_Ack(void)
{
	uint16_t Out_Time=1000;
    
    IIC_SDA_H();
	IIC_SDA_In();
	IIC_Delay(1);
	IIC_SCL_H();
	IIC_Delay(1);
	while(IIC_SDA_Read())
	{
		if(--Out_Time)
		{
			IIC_Stop();
            return 0xff;
		}
	}
	IIC_SCL_L();
    return 0;
}
/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SDA_Out();
	IIC_SDA_L();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SCL_L();
	IIC_Delay(1);
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SDA_Out();
	IIC_SDA_H();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SCL_L();
	IIC_Delay(1);
}
/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(uint8_t Temp)
{
	uint8_t i;
	IIC_SDA_Out();
	IIC_SCL_L();
	for(i=0;i<8;i++)
	{
		if(Temp&0x80)
		{
			IIC_SDA_H();
		}
		else
		{
			IIC_SDA_L();
		}
		Temp<<=1;
		IIC_Delay(1);
		IIC_SCL_H();
		IIC_Delay(1);
		IIC_SCL_L();
	}
} 	 
  
/**************************实现函数********************************************
*函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		
*******************************************************************************/
int i2cWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	IIC_Start();
    
	IIC_Send_Byte(dev_addr << 1);
	if(IIC_Wait_Ack() == 0xff)
    {
        return 0xff;
    }
    
	IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack() == 0xff)
    {
        return 0xff;
    }

	for(int i=0;i<len;i++)
	{
		IIC_Send_Byte(data[i]);
		if(IIC_Wait_Ack() == 0xff)
		{
			return 0xff;
		}
	}
	IIC_Stop();
    return 0;
}
/**************************实现函数********************************************
*函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		
*******************************************************************************/
int i2cRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t count, uint8_t *pdata)
{
	uint8_t i;
	IIC_Start();
	
    IIC_Send_Byte(dev_addr << 1);
	if(IIC_Wait_Ack() == 0xff)
    {
        return 0xff;
    }
    
    IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack() == 0xff)
    {
        return 0xff;
    }
	
    IIC_Start();
    
    IIC_Send_Byte((dev_addr << 1)+ 1);
	if(IIC_Wait_Ack() == 0xff)
    {
        return 0xff;
    }
    
    for(i=0;i<(count-1);i++)
    {
        *pdata=IIC_Read_Byte(1);
        pdata++;
    }

    *pdata=IIC_Read_Byte(0);
    IIC_Stop(); 
    
    return 0;    
}



/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	uint8_t i,Temp=0;
	IIC_SDA_In();
	for(i=0;i<8;i++)
	{
		IIC_SCL_L();
		IIC_Delay(1);
		IIC_SCL_H();
		Temp<<=1;
		if(IIC_SDA_Read())
		   Temp++;
		IIC_Delay(1);
	}
	IIC_SCL_L();
					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return Temp;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //发送写命令
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //发送地址
	IIC_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//产生一个停止条件

	return res;
}


/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //进入接收模式	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
		reg	   寄存器地址
		*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
