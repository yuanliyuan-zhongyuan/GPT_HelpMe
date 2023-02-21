#include "hal_tftlcd.h"

//TFTLCD配置
void hal_TFTLCD_Config(void)
{
	//创建三个结构体  spi，gpio和dma的
	SPI_InitTypeDef   SPI_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStruct;
	DMA_InitTypeDef	  DMA_InitStruct;
	//开GPIOA,B,C和SPI3,DMA2,复用口的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
						  |RCC_APB2Periph_GPIOB
						  |RCC_APB2Periph_GPIOC
						  |RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
	//引脚重映射，PB4,PB5
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	//RES-PA15
	GPIO_InitStruct.GPIO_Pin = LCD_RES_PIN;	 
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //速度50MHz
 	GPIO_Init(LCD_RES_PORT, &GPIO_InitStruct);	  		 //初始化GPIOA
	 //LCD脚配置的时候都拉高是为了后面更好的初始化，处在一个稳定态
	GPIO_SetBits(LCD_RES_PORT,LCD_RES_PIN);		
	
	//CMD-PB4
	//CS-PB6
	GPIO_InitStruct.GPIO_Pin = LCD_CS_PIN|LCD_CMD_PIN;	 
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //速度50MHz
 	GPIO_Init(LCD_PORTB, &GPIO_InitStruct);	  			 //初始化GPIOB	
	GPIO_SetBits(LCD_PORTB,LCD_CS_PIN|LCD_CMD_PIN);
	
	//EN-PC10
	GPIO_InitStruct.GPIO_Pin = LCD_EN_PIN;	 
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //速度50MHz
 	GPIO_Init(LCD_PORTC, &GPIO_InitStruct);	  			 //初始化GPIOC		
	GPIO_ResetBits(LCD_PORTC,LCD_EN_PIN);
	
	//CLK-PB3
	//MOSI-PB5
	GPIO_InitStruct.GPIO_Pin =  LCD_SLK_PIN |LCD_DO_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //速度50MHz
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;		 //复用推挽输出	
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	//SPI3配置
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx; 			//SPI1设置为单线
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;	                    //设置SPI1为主模式
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                  //SPI发送接收8位帧结构
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;	 		            //串行时钟在不操作时，时钟为高电平
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;		                //第二个时钟沿开始采样数据
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;			                //NSS信号由软件（使用SSI位）管理
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //定义波特率预分频的值:波特率预分频值为2
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;				    //数据传输从MSB位开始
	SPI_InitStruct.SPI_CRCPolynomial = 7;						    //CRC值计算的多项式
	SPI_Init(SPI3, &SPI_InitStruct);
	
	//DMA2配置
	DMA_DeInit(DMA2_Channel2); 											 //因为SPI3发送是在DMA2通道2上面
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&SPI3->DR; 			 //数据传输目标地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST; 					 //数据传输方向，从内存读取发送到外设
	DMA_InitStruct.DMA_BufferSize = 1024;           					 //发送Buff数据大小
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 		 //设置外设地址是否递增
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  				 //设置内存地址是否递增
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度为8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 //内存数据宽度为8位	
	DMA_InitStruct.DMA_Mode =   DMA_Mode_Normal;                         //普通缓存模式
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;                   //高优先级
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;            				 //禁止DMA2个内存相互访问
	DMA_Init(DMA2_Channel2, &DMA_InitStruct);        					 //初始化DMA,SPI在DMA1的通道2
	
	//使能SPI2 DMA的发送功能
	SPI_I2S_DMACmd(SPI3,SPI_I2S_DMAReq_Tx,ENABLE); 
	//使能SPI3
	SPI_Cmd(SPI3, ENABLE);
}

//SPI3 DMA 数据发送函数
void DMA_SPI3_TX(uint8_t* buffer,uint16_t len)
{
	DMA2->IFCR |=(0xf<<4);   				 //清除通道2上面所有的标志位
	DMA2_Channel2->CNDTR=len; 		 		 //设置要传输的数据长度
	DMA2_Channel2->CMAR=(u32)buffer;		 //设置RAM缓冲区地址
	DMA2_Channel2->CCR|=0x1;   				 //启动DMA
        while(!(DMA2->ISR&(1<<5))) ; 		 //等待数据数据传输完成
	DMA2_Channel2->CCR &=(uint32_t)~0x1;	 //关闭DMA
}
//LCD写总线
void LCD_Writ_Bus(uint8_t dat) 
{	
	//片选拉低后才能发送数据
	LCD_CS_Clr();
	//发送一个字节
	DMA_SPI3_TX(&dat,1);
}
//LCD开
void hal_LCD_Display_on(void)
{
	LCD_EN_Set();
}
//LCD关
void hal_LCD_Display_off(void)
{
	LCD_EN_Clr();
}
//LCD复位脚拉高
void hal_LCD_ResetH(void)
{
	LCD_RES_Set();
}
//LCD复位脚拉低
void hal_LCD_ResetL(void)
{
	LCD_RES_Clr();
}
/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
	uint8_t d[2];
	d[0] = dat>>8;
	d[1] = dat;
	DMA_SPI3_TX(&d[0],2);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}















