#include "i2c_lcd.h"


uint8_t I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t lastevent = 0;
	  uint32_t flag1 = 0, flag2 = 0;
	  uint8_t status = I2C_Error;

	  /* Check the parameters */
//	  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
//	  assert_param(IS_I2C_EVENT(I2C_EVENT));

	  /* Read the I2Cx status register */
	  flag1 = I2Cx->SR1;
	  flag2 = I2Cx->SR2;
	  flag2 = flag2 << 16;

	  /* Get the last event value from I2C status register */
	  lastevent = (flag1 | flag2) & I2C_FlagMask;

	  /* Check whether the last event contains the I2C_EVENT */
	  if ((lastevent & I2C_EVENT) == I2C_EVENT)
	  {
	    /* SUCCESS: last event is equal to I2C_EVENT */
	    status = I2C_Success;
	  }
	  else
	  {
	    /* ERROR: last event is different from I2C_EVENT */
	    status = I2C_Error;
	  }
	  /* Return status */
	  return status;
}

void lcd_Write_byte(char data)
{
    /* Send START condition */
	LL_I2C_GenerateStartCondition(I2C_Channel);
    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C_Channel, I2C_EVENT_MASTER_MODE_SELECT));
    /* Send PCF8574A address for write */
    LL_I2C_TransmitData8(I2C_Channel, PCF8574A_Address);
	/* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C_Channel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    /* Send the byte to be written */
    LL_I2C_TransmitData8(I2C_Channel, data);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C_Channel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    /* Send STOP condition */
    LL_I2C_GenerateStopCondition(I2C_Channel);
}
void lcd_init (void)
{
	/* Set 4-bits interface */
	lcd_Control_Write(0x33);
	LL_mDelay(10);
	lcd_Control_Write(0x32);
	LL_mDelay(50);
	/* Start to set LCD function */
	lcd_Control_Write(0x28);
	LL_mDelay(50);
	/* clear LCD */
	lcd_Control_Write(0x01);
	LL_mDelay(50);
	/* wait 60ms */

	/* set entry mode */
	lcd_Control_Write(0x06);	LL_mDelay(50);;
	/* set display to on */
	lcd_Control_Write(0x0C);	LL_mDelay(50);;
	/* move cursor to home and set data address to 0 */
	lcd_Control_Write(0x02);	LL_mDelay(50);
}
void lcd_Data_Write(char data)
{
	char data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x0d;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0d;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
	     lcd_Write_byte(data_t[i]);
	}

}
void lcd_Control_Write(char data)
{
    char data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x04;  //en=1, rs=0
	data_t[1] = data_u;  //en=0, rs=0
	data_t[2] = data_l|0x04;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
	     lcd_Write_byte(data_t[i]);
	}

}
void lcd_send_string (char *str)
{
	while (*str) lcd_Data_Write (*str++);
}
void Delete_LCD(void)
{
	lcd_Control_Write(0x01);
	LL_mDelay(10);
}
