#ifndef _I2C_H_
  #define _I2C_H_

  #include "main.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)



#define TRUE  0
#define FALSE -1

#define SCL_H  nop()//HAL_GPIO_WritePin(SOFT_SCL_GPIO_Port, SOFT_SCL_Pin, GPIO_PIN_SET)		/* SCL = 1 */
#define SCL_L  nop()//HAL_GPIO_WritePin(SOFT_SCL_GPIO_Port, SOFT_SCL_Pin, GPIO_PIN_RESET)		/* SCL = 0 */

#define SDA_H  nop()//HAL_GPIO_WritePin(SOFT_SDA_GPIO_Port, SOFT_SDA_Pin, GPIO_PIN_SET)		/* SDA = 1 */
#define SDA_L  nop()//HAL_GPIO_WritePin(SOFT_SDA_GPIO_Port, SOFT_SDA_Pin, GPIO_PIN_RESET)		/* SDA = 0 */

#define SDA_read  0//HAL_GPIO_ReadPin(SOFT_SDA_GPIO_Port, SOFT_SDA_Pin)	/* ��SDA����״̬ */



#define	I2C_Direction_Trans   0

#define	I2C_Direction_Rec      1	 
/*====================================================================================================*/
/*====================================================================================================*/
//PB6 SCL
//PB7 SDA
//return 0:success   1:failed
//----------------------------------------------------------------------
extern int8_t IIC_Write_One_Byte(uint8_t addr,uint8_t reg,uint8_t data);
extern int8_t IIC_Read_One_Byte(uint8_t addr,uint8_t reg);	 
extern int8_t IIC_Write_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len);
extern int8_t IIC_read_Bytes(uint8_t addr,uint8_t reg,uint8_t *data,uint8_t len);
//----------------------------------------------------------------------f


/*====================================================================================================*/
/*====================================================================================================*/

#endif
