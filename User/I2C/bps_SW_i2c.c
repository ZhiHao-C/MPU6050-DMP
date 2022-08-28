#include "bps_SW_i2c.h"
#include "bps_mpu6050.h"
#include "bps_systick.h"

extern void Delay_ms(int n);//延时函数


void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能与 I2C 有关的时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	//打开GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
    
  /* I2C_SCL、I2C_SDA*/
  GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // 开漏输出
  GPIO_Init(MPU6050_I2C_SCL_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	       // 开漏输出
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
	SDA(1);//释放总线
}

uint8_t I2C_receivedata(void)
{
	
	uint8_t Data=0x00;
	uint8_t i=0;
	SDA(1);//释放总线
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

//	/* 读到第1个bit为数据的bit7 */
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
	SDA(1);//应答之后需要释放SDA总线
}


uint8_t I2C_receiveACK(void)//返回值为0表示收到应答
{
	SDA(1);                    //等待应答，主机需要释放SDA总线，由从机产生应答
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
	I2C_sentdata(MPU6050_SLAVE_ADDRESS);//写入设备地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_sentdata(addr);//写入需要写入的地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_sentdata(byte);//写入数据
	while(I2C_receiveACK());//等待从机发回应答
	
	stop();
	
}

//读取一个数据
uint8_t I2C_Rbyte(uint8_t addr)
{
	uint8_t Data;
	start();
	I2C_sentdata(0XA0);//写入设备地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_sentdata(addr);//写入需要读取的地址
	while(I2C_receiveACK());//等待从机发回应答
	
	start();
	
	I2C_sentdata(0XA0+1);//写入设备地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_receivedata();
	Data=I2C_receivedata();
	
	I2C_sentACK(0);
	stop();
	return Data;
}


//读取任意字节的数据
uint8_t I2C_R_Page(uint8_t addr,uint8_t *Data,uint8_t n)
{
	uint8_t i=0;
	start();
	I2C_sentdata(MPU6050_SLAVE_ADDRESS);//写入设备地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_sentdata(addr);//写入需要读取的地址
	while(I2C_receiveACK());//等待从机发回应答
	
	start();//第二次开始
	I2C_sentdata(MPU6050_SLAVE_ADDRESS+1);//写入设备地址（读）
	while(I2C_receiveACK());//等待从机发回应答
	
	
	//循环读取数据
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
	return 1;//执行成功
}
/****************************** 以下函数为DMP官方库函数使用 **********************************************/
static unsigned short RETRY_IN_MLSEC  = 55;
/**
  * @brief  获取设置的iic重试时间
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
	I2C_sentdata(slave_addr<<1|I2C_Direction_Transmitter);//写入设备地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_sentdata(reg_addr);//写入需要写入的地址
	while(I2C_receiveACK());//等待从机发回应答
	
	//循环写入数据
	for(i=0;i<length;i++)
	{
		I2C_sentdata(data[i]);
		while(I2C_receiveACK());//等待从机发回应答
	}
	
	stop();
	return 0;
}

int ST_Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data)
{

	start();
	I2C_sentdata(slave_addr<<1|I2C_Direction_Transmitter);//写入设备地址
	while(I2C_receiveACK());//等待从机发回应答
	
	I2C_sentdata(reg_addr);//写入需要读取的地址
	while(I2C_receiveACK());//等待从机发回应答
	
	start();//第二次开始
	I2C_sentdata(slave_addr<<1|I2C_Direction_Receiver);//写入设备地址（读）
	while(I2C_receiveACK());//等待从机发回应答
	
	
	//循环读取数据
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
	return 0;//执行成功
}

/**
  * @brief  向IIC设备的寄存器连续写入数据，带超时重试设置，供mpu接口调用
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要写入数据的长度
  * @param  RegisterValue: 要指向写入数据的指针
  * @retval 0正常，非0异常
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain://如果发送失败会重复发送四次
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
  * @brief  向IIC设备的寄存器连续读出数据,带超时重试设置，供mpu接口调用
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要读取的数据长度
  * @param  RegisterValue: 指向存储读出数据的指针
  * @retval 0正常，非0异常
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain://如果接收失败会重复接收四次
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
