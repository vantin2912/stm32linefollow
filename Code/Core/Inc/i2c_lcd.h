#ifndef __I2C_LCD_H
#define __I2C_LCD_H


	 
#include "main.h"

#define PCF8574A_Address      	0x7E
#define I2C_Channel      		I2C1

#define I2C_Success 1
#define I2C_Error 0
#define I2C_FlagMask ((uint32_t)0x00FFFFFF)

/* --EV5 */
#define  I2C_EVENT_MASTER_MODE_SELECT		((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */
/* --EV6 */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED			((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED			((uint32_t)0x00030002)  /* BUSY, MSL and ADDR flags */
/* --EV8_2 */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */



void lcd_Write_byte(char data);
void lcd_init (void);
void lcd_Data_Write(char data);
void lcd_Control_Write(char data);
void lcd_send_string (char *str);
void Delete_LCD(void);

uint8_t I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
//extern void Delay_msST(uint16_t nTime);
#endif
