#ifndef ____HAL_TFTLCD_H_
#define ____HAL_TFTLCD_H_
// AC Check Pin
#define LCD_SCLK__PORT       GPIOB
#define LCD_SCLK__PIN        GPIO_Pin_3

//-----------------LCD¶Ë¿Ú¶¨Òå---------------- 
#define LCD_SCLK_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_3)//SCL=SCLK
#define LCD_SCLK_Set() GPIO_SetBits(GPIOB,GPIO_Pin_3)

#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_5)//SDA=MOSI
#define LCD_MOSI_Set() GPIO_SetBits(GPIOB,GPIO_Pin_5)

#define LCD_DC_Clr()   GPIO_ResetBits(GPIOB,GPIO_Pin_4)//DC
#define LCD_DC_Set()   GPIO_SetBits(GPIOB,GPIO_Pin_4)

#define LCD_CS_Clr()   GPIO_ResetBits(GPIOB,GPIO_Pin_6)//CS
#define LCD_CS_Set()   GPIO_SetBits(GPIOB,GPIO_Pin_6)

#define LCD_RES_Clr()  GPIO_ResetBits(GPIOA,GPIO_Pin_15)//RES
#define LCD_RES_Set()  GPIO_SetBits(GPIOA,GPIO_Pin_15)

#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOC,GPIO_Pin_10)//BLK
#define LCD_BLK_Set()  GPIO_SetBits(GPIOC,GPIO_Pin_10)
void hal_tftlcdConfig(void);

void LCD_WR_REG(unsigned char dat);
void LCD_WR_DATA8(unsigned char dat);
void LCD_WR_DATA(unsigned short dat);
void DMA_SPI3_TX(unsigned char *buffer,unsigned short len);

void hal_Oled_Display_on(void);
void hal_Oled_Display_off(void);
void hal_oled_RestH(void);
void hal_oled_RestL(void);


#endif




