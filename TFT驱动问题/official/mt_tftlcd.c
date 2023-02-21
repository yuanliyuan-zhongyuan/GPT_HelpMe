#include "mt_Tftlcd.h"
#include "hal_tftlcd.h"

static void hal_tftlcd_Delay(unsigned int de);
unsigned char ColorBuf[640];

void mt_tftlcd_init(void)
{
  	hal_tftlcdConfig();//初始化GPIO
		hal_tftlcd_Delay(10000);
		hal_oled_RestL();//复位
		hal_tftlcd_Delay(10000);
		hal_oled_RestH();
		hal_tftlcd_Delay(100);
	//************* Start Initial Sequence **********//
		LCD_WR_REG(0x11);
		hal_tftlcd_Delay(10000);//delay_ms(100); //Delay 120ms
		LCD_WR_REG(0X36);// Memory Access Control
		if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
		else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
		else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
		else LCD_WR_DATA8(0xA0);
		LCD_WR_REG(0X3A);
	 // LCD_WR_DATA8(0X03);   //12bit
		LCD_WR_DATA8(0X05);  
		//--------------------------------ST7789S Frame rate setting-------------------------

		LCD_WR_REG(0xb2);
		LCD_WR_DATA8(0x0c);
		LCD_WR_DATA8(0x0c);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x33);
		LCD_WR_DATA8(0x33);

		LCD_WR_REG(0xb7);
		LCD_WR_DATA8(0x35);
		//---------------------------------ST7789S Power setting-----------------------------

		LCD_WR_REG(0xbb);
		LCD_WR_DATA8(0x35);

		LCD_WR_REG(0xc0);
		LCD_WR_DATA8(0x2c);

		LCD_WR_REG(0xc2);
		LCD_WR_DATA8(0x01);

		LCD_WR_REG(0xc3);
		LCD_WR_DATA8(0x13);

		LCD_WR_REG(0xc4);
		LCD_WR_DATA8(0x20);

		LCD_WR_REG(0xc6);
		LCD_WR_DATA8(0x0f);

		LCD_WR_REG(0xca);
		LCD_WR_DATA8(0x0f);

		LCD_WR_REG(0xc8);
		LCD_WR_DATA8(0x08);

		LCD_WR_REG(0x55);
		LCD_WR_DATA8(0x90);

		LCD_WR_REG(0xd0);
		LCD_WR_DATA8(0xa4);
		LCD_WR_DATA8(0xa1);
		//--------------------------------ST7789S gamma setting------------------------------
		LCD_WR_REG(0xe0);
		LCD_WR_DATA8(0xd0);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x06);
		LCD_WR_DATA8(0x09);
		LCD_WR_DATA8(0x0b);
		LCD_WR_DATA8(0x2a);
		LCD_WR_DATA8(0x3c);
		LCD_WR_DATA8(0x55);
		LCD_WR_DATA8(0x4b);
		LCD_WR_DATA8(0x08);
		LCD_WR_DATA8(0x16);
		LCD_WR_DATA8(0x14);
		LCD_WR_DATA8(0x19);
		LCD_WR_DATA8(0x20);
		LCD_WR_REG(0xe1);
		LCD_WR_DATA8(0xd0);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x06);
		LCD_WR_DATA8(0x09);
		LCD_WR_DATA8(0x0b);
		LCD_WR_DATA8(0x29);
		LCD_WR_DATA8(0x36);
		LCD_WR_DATA8(0x54);
		LCD_WR_DATA8(0x4b);
		LCD_WR_DATA8(0x0d);
		LCD_WR_DATA8(0x16);
		LCD_WR_DATA8(0x14);
		LCD_WR_DATA8(0x21);
		LCD_WR_DATA8(0x20);
		LCD_WR_REG(0x29);
		hal_Oled_Display_on();//打开背光

		LCD_Fill(0,0,LCD_W,LCD_H,YELLOW);
} 

/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(unsigned short x1,unsigned short y1,unsigned short x2,unsigned short y2)
{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+2);
		LCD_WR_DATA(x2+2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);//储存器写
}

static void hal_tftlcd_Delay(unsigned int de)
{
	while(de--);
}
/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/

void LCD_Fill(unsigned short xsta,unsigned short ysta,unsigned short xend,unsigned short yend,unsigned short color)
{          
	unsigned short i; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=0;i<xend;i++)
  {
		ColorBuf[i++] = color>>8;
		ColorBuf[i] = color;
	}
	for(i=ysta;i<yend*2;i++)
	{		
		 DMA_SPI3_TX(ColorBuf,xend);
	}	
}

