#include "bps_SW_i2c.h"
#include "bps_mpu6050.h"
#include "bps_systick.h"

extern void Delay_ms(int n);//��ʱ����


void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

	/* ʹ���� I2C �йص�ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	//��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
    
  /* I2C_SCL��I2C_SDA*/
  GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // ��©���
  GPIO_Init(MPU6050_I2C_SCL_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // ��©���
  GPIO_Init(MPU6050_I2C_SDA_PORT, &GPIO_InitStructure);	
	
}

void start(void)
{
	SDA(1);
	SCL(1);
	Delay_ms(1);
	SDA(0);
	Delay_ms(1);
	SCL(0);
}

void stop(void)
{
	SDA(0)
	SCL(1);
	Delay_ms(1);
	SDA(1);
}


void I2C_sentdata(uint8_t byte)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		if((byte&(0x80>>i))!=0)
		{
			SDA(1);
		}
		else
		{
			SDA(0);
		}
		SCL(1);
		Delay_ms(1);
		SCL(0);
	}
	SDA(1);//�ͷ�����
}

uint8_t I2C_receivedata(void)
{
	
	uint8_t Data=0x00;
	uint8_t i=0;
	SDA(1);//�ͷ�����
	Delay_ms(2);
	for(i=0;i<8;i++)
	{
		SCL(1);
		if(GPIO_ReadInputDataBit(MPU6050_I2C_SDA_PORT,MPU6050_I2C_SDA_PIN)==1)
		{
			Data=Data|(0x80>>i);
		}
		Delay_ms(1);
		SCL(0);
		Delay_ms(1);
	}
	return Data;
	
//	uint8_t i;
//	uint8_t value;

//	/* ������1��bitΪ���ݵ�bit7 */
//	value = 0;
//	for (i = 0; i < 8; i++)
//	{
//		value <<= 1;
//		SCL(1);
//		Delay_ms(1);
//		if ((GPIOB->IDR&GPIO_Pin_7))
//		{
//			value++;
//		}
//		SCL(0);
//		Delay_ms(1);
//	}
//	return value;

	
}

void I2C_sentACK(uint8_t ACK)
{
	if(ACK==1)
	{
		SDA(0);
	}
	else
	{
		SDA(1);
	}
	SCL(1);
	Delay_ms(2);
	SCL(0);
	SDA(1);//Ӧ��֮����Ҫ�ͷ�SDA����
}


uint8_t I2C_receiveACK(void)//����ֵΪ0��ʾ�յ�Ӧ��
{
	SDA(1);                    //�ȴ�Ӧ��������Ҫ�ͷ�SDA���ߣ��ɴӻ�����Ӧ��
	Delay_ms(1);
	SCL(1);
	Delay_ms(1);
	if((GPIOB->IDR&0x0080)!=0)
	{
		Delay_ms(1);
		SCL(0);
		return 1;
	}
	Delay_ms(1);
	SCL(0);
	return 0;
}

void I2C_Wbyte(uint8_t addr,uint8_t byte)
{
	start();
	I2C_sentdata(MPU6050_SLAVE_ADDRESS);//д���豸��ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_sentdata(addr);//д����Ҫд��ĵ�ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_sentdata(byte);//д������
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	stop();
	
}

//��ȡһ������
uint8_t I2C_Rbyte(uint8_t addr)
{
	uint8_t Data;
	start();
	I2C_sentdata(0XA0);//д���豸��ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_sentdata(addr);//д����Ҫ��ȡ�ĵ�ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	start();
	
	I2C_sentdata(0XA0+1);//д���豸��ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_receivedata();
	Data=I2C_receivedata();
	
	I2C_sentACK(0);
	stop();
	return Data;
}


//��ȡ�����ֽڵ�����
uint8_t I2C_R_Page(uint8_t addr,uint8_t *Data,uint8_t n)
{
	uint8_t i=0;
	start();
	I2C_sentdata(MPU6050_SLAVE_ADDRESS);//д���豸��ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_sentdata(addr);//д����Ҫ��ȡ�ĵ�ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	start();//�ڶ��ο�ʼ
	I2C_sentdata(MPU6050_SLAVE_ADDRESS+1);//д���豸��ַ������
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	
	//ѭ����ȡ����
	for(i=0;i<n;i++)
	{
		if(i==n-1)
		{
			Data[i]=I2C_receivedata();
			I2C_sentACK(0);
		}
		else
		{
			Data[i]=I2C_receivedata();
			I2C_sentACK(1);
		}
		
	}
	stop();
	return 1;//ִ�гɹ�
}
/****************************** ���º���ΪDMP�ٷ��⺯��ʹ�� **********************************************/
static unsigned short RETRY_IN_MLSEC  = 55;
/**
  * @brief  ��ȡ���õ�iic����ʱ��
  * @param  none
  * @retval none
  */
unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}



int ST_Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data)
{
	uint8_t i=0;
	start();
	I2C_sentdata(slave_addr<<1|I2C_Direction_Transmitter);//д���豸��ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_sentdata(reg_addr);//д����Ҫд��ĵ�ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	//ѭ��д������
	for(i=0;i<length;i++)
	{
		I2C_sentdata(data[i]);
		while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	}
	
	stop();
	return 0;
}

int ST_Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data)
{

	start();
	I2C_sentdata(slave_addr<<1|I2C_Direction_Transmitter);//д���豸��ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	I2C_sentdata(reg_addr);//д����Ҫ��ȡ�ĵ�ַ
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	start();//�ڶ��ο�ʼ
	I2C_sentdata(slave_addr<<1|I2C_Direction_Receiver);//д���豸��ַ������
	while(I2C_receiveACK());//�ȴ��ӻ�����Ӧ��
	
	
	//ѭ����ȡ����
	while (length)
	{
		if (length==1)
		{
			*data =I2C_receivedata();
			I2C_sentACK(0);
		} 
		else 
		{
			*data =I2C_receivedata();
			I2C_sentACK(1);
			Delay_ms(1);
		}
		data++;
		length--;
	}
	stop();
	return 0;//ִ�гɹ�
}

/**
  * @brief  ��IIC�豸�ļĴ�������д�����ݣ�����ʱ�������ã���mpu�ӿڵ���
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫд�����ݵĳ���
  * @param  RegisterValue: Ҫָ��д�����ݵ�ָ��
  * @retval 0��������0�쳣
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain://�������ʧ�ܻ��ظ������Ĵ�
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;
}


/**
  * @brief  ��IIC�豸�ļĴ���������������,����ʱ�������ã���mpu�ӿڵ���
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫ��ȡ�����ݳ���
  * @param  RegisterValue: ָ��洢�������ݵ�ָ��
  * @retval 0��������0�쳣
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain://�������ʧ�ܻ��ظ������Ĵ�
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}