#ifndef hal_tftlcd
#define hal_tftlcd


#define LCD_CMD  0	//写命令
#define LCD_DATA 1	//写数据
//LCD引脚在PA,PB,PC口
#define LCD_PORTA 		GPIOA
#define LCD_PORTB 		GPIOB
#define LCD_PORTC 		GPIOC
//时钟线
#define LCD_SLK_PORT    LCD_PORTB
#define LCD_SLK_PIN		GPIO_Pin_3
//数据线
#define LCD_DO_PORT		LCD_PORTB
#define LCD_DO_PIN		GPIO_Pin_5
//复位脚
#define LCD_RES_PORT	LCD_PORTA
#define LCD_RES_PIN		GPIO_Pin_15
//控制脚
#define LCD_CMD_PORT	LCD_PORTB
#define LCD_CMD_PIN 	GPIO_Pin_4
//片选脚
#define LCD_CS_PORT		LCD_PORTB
#define LCD_CS_PIN 		GPIO_Pin_6
//使能脚
#define LCD_EN_PORT		LCD_PORTC
#define LCD_EN_PIN 		GPIO_Pin_10
//-----------------LCD端口定义---------------- 

#define LCD_SCL_Clr() GPIO_ResetBits(LCD_SLK_PORT,LCD_SLK_PIN)//SCL
#define LCD_SCL_Set() GPIO_SetBits(LCD_SLK_PORT,LCD_SLK_PIN)

#define LCD_SDA_Clr() GPIO_ResetBits(LCD_DO_PORT,LCD_DO_PIN)//SDA MISO
#define LCD_SDA_Set() GPIO_SetBits(LCD_DO_PORT,LCD_DO_PIN)

#define LCD_RES_Clr() GPIO_ResetBits(LCD_RES_PORT,LCD_RES_PIN)//RES
#define LCD_RES_Set() GPIO_SetBits(LCD_RES_PORT,LCD_RES_PIN)

#define LCD_DC_Clr()  GPIO_ResetBits(LCD_CMD_PORT,LCD_CMD_PIN)//DC
#define LCD_DC_Set()  GPIO_SetBits(LCD_CMD_PORT,LCD_CMD_PIN)
	     
#define LCD_CS_Clr()  GPIO_ResetBits(LCD_CS_PORT,LCD_CS_PIN)//CS
#define LCD_CS_Set()  GPIO_SetBits(LCD_CS_PORT,LCD_CS_PIN)

#define LCD_EN_Clr()  GPIO_ResetBits(LCD_EN_PORT,LCD_EN_PIN)//EN
#define LCD_EN_Set()  GPIO_SetBits(LCD_EN_PORT,LCD_EN_PIN)

#include "all.h"

void hal_TFTLCD_Config(void);
void LCD_Writ_Bus(uint8_t dat); 
void hal_LCD_Display_on(void);
void hal_LCD_Display_off(void);
void hal_LCD_ResetL(void);
void hal_LCD_ResetH(void);
void LCD_WR_DATA8(uint8_t dat);
void LCD_WR_DATA(uint16_t dat);
void LCD_WR_REG(uint8_t dat);
void DMA_SPI3_TX(uint8_t* buffer,uint16_t len);

#endif /*hal_tftlcd*/
