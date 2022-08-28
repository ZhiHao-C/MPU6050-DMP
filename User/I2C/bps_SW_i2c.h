#ifndef __BPS_SW_I2C_H__
#define __BPS_SW_I2C_H__

#include "stm32f10x.h" 

/**************************I2C�������壬I2C1��I2C2********************************/
#define             MPU6050_I2Cx                                I2C1
#define             MPU6050_I2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             MPU6050_I2C_CLK                             RCC_APB1Periph_I2C1
#define             MPU6050_I2C_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             MPU6050_I2C_GPIO_CLK                        RCC_APB2Periph_GPIOB     
#define             MPU6050_I2C_SCL_PORT                        GPIOB   
#define             MPU6050_I2C_SCL_PIN                         GPIO_Pin_6
#define             MPU6050_I2C_SDA_PORT                        GPIOB 
#define             MPU6050_I2C_SDA_PIN                         GPIO_Pin_7

//MPU6050��AD0���Ž�GNDʱ����ַΪ0x68 ,��3.3Vʱ����ַΪ0x69��ע��Ҫƥ�䡣
#define MPU6050_ADDR   0x68

/*�ȴ���ʱʱ��*/
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))


/*��Ϣ���*/
#define MPU_DEBUG_ON         0
#define MPU_DEBUG_FUNC_ON    0

#define MPU6050_INFO(fmt,arg...)           printf("<<-MPU6050-INFO->> "fmt"\n",##arg)
#define MPU6050_ERROR(fmt,arg...)          printf("<<-MPU6050-ERROR->> "fmt"\n",##arg)
#define MPU6050_DEBUG(fmt,arg...)          do{\
                                          if(MPU6050_DEBUG_ON)\
                                          printf("<<-MPU6050-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

#define MPU_DEBUG_FUNC()               do{\
                                         if(MPU_DEBUG_FUNC_ON)\
                                         printf("<<-MPU-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)


/* 
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */

/* MPU6050 Addresses defines */
#define MPU6050_Block0_ADDRESS 0xA0   /* E2 = 0 */
//#define MPU6050_Block1_ADDRESS 0xA2 /* E2 = 0 */
//#define MPU6050_Block2_ADDRESS 0xA4 /* E2 = 0 */
//#define MPU6050_Block3_ADDRESS 0xA6 /* E2 = 0 */


void I2C_EE_Init(void);
void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite);
uint32_t I2C_EE_ByteWrite(u8* pBuffer, u8 WriteAddr);
uint32_t I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite);
uint32_t I2C_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void I2C_EE_WaitEepromStandbyState(void);
																			
																																				
																					
																					
																					
#define SDA(a)               if(a)\
                             {GPIO_SetBits(MPU6050_I2C_SDA_PORT,MPU6050_I2C_SDA_PIN );}\
											 else\
                             {GPIO_ResetBits(MPU6050_I2C_SDA_PORT,MPU6050_I2C_SDA_PIN);} 
#define SCL(a)               if(a)\
                             {GPIO_SetBits(MPU6050_I2C_SCL_PORT,MPU6050_I2C_SCL_PIN );}\
											 else\
                             {GPIO_ResetBits(MPU6050_I2C_SCL_PORT,MPU6050_I2C_SCL_PIN);} 

													
																					
																				
void I2C_config(void);																				
void start(void);
void stop(void);
void I2C_sentdata(uint8_t byte);
uint8_t I2C_receivedata(void);
void I2C_sentACK(uint8_t ACK);
uint8_t I2C_receiveACK(void);		
void I2C_GPIO_Config(void);														 
void Delay_ms(int n);
uint8_t I2C_Rbyte(uint8_t addr);
void I2C_Wbyte(uint8_t addr,uint8_t byte);
uint8_t I2C_R_Page(uint8_t addr,uint8_t *Data,uint8_t n);

														 
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr);
#endif
