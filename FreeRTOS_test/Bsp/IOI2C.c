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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
void IIC_Init(void)
{			
	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
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
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
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
  
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
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



/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
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
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return Temp;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
