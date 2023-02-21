#include "hal_tftlcd.h"

//TFTLCD����
void hal_TFTLCD_Config(void)
{
	//���������ṹ��  spi��gpio��dma��
	SPI_InitTypeDef   SPI_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStruct;
	DMA_InitTypeDef	  DMA_InitStruct;
	//��GPIOA,B,C��SPI3,DMA2,���ÿڵ�ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
						  |RCC_APB2Periph_GPIOB
						  |RCC_APB2Periph_GPIOC
						  |RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
	//������ӳ�䣬PB4,PB5
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	//RES-PA15
	GPIO_InitStruct.GPIO_Pin = LCD_RES_PIN;	 
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //�ٶ�50MHz
 	GPIO_Init(LCD_RES_PORT, &GPIO_InitStruct);	  		 //��ʼ��GPIOA
	 //LCD�����õ�ʱ��������Ϊ�˺�����õĳ�ʼ��������һ���ȶ�̬
	GPIO_SetBits(LCD_RES_PORT,LCD_RES_PIN);		
	
	//CMD-PB4
	//CS-PB6
	GPIO_InitStruct.GPIO_Pin = LCD_CS_PIN|LCD_CMD_PIN;	 
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //�ٶ�50MHz
 	GPIO_Init(LCD_PORTB, &GPIO_InitStruct);	  			 //��ʼ��GPIOB	
	GPIO_SetBits(LCD_PORTB,LCD_CS_PIN|LCD_CMD_PIN);
	
	//EN-PC10
	GPIO_InitStruct.GPIO_Pin = LCD_EN_PIN;	 
 	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //�ٶ�50MHz
 	GPIO_Init(LCD_PORTC, &GPIO_InitStruct);	  			 //��ʼ��GPIOC		
	GPIO_ResetBits(LCD_PORTC,LCD_EN_PIN);
	
	//CLK-PB3
	//MOSI-PB5
	GPIO_InitStruct.GPIO_Pin =  LCD_SLK_PIN |LCD_DO_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		 //�ٶ�50MHz
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;		 //�����������	
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	//SPI3����
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx; 			//SPI1����Ϊ����
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;	                    //����SPI1Ϊ��ģʽ
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                  //SPI���ͽ���8λ֡�ṹ
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;	 		            //����ʱ���ڲ�����ʱ��ʱ��Ϊ�ߵ�ƽ
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;		                //�ڶ���ʱ���ؿ�ʼ��������
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;			                //NSS�ź��������ʹ��SSIλ������
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ2
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;				    //���ݴ����MSBλ��ʼ
	SPI_InitStruct.SPI_CRCPolynomial = 7;						    //CRCֵ����Ķ���ʽ
	SPI_Init(SPI3, &SPI_InitStruct);
	
	//DMA2����
	DMA_DeInit(DMA2_Channel2); 											 //��ΪSPI3��������DMA2ͨ��2����
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&SPI3->DR; 			 //���ݴ���Ŀ���ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST; 					 //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStruct.DMA_BufferSize = 1024;           					 //����Buff���ݴ�С
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 		 //���������ַ�Ƿ����
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  				 //�����ڴ��ַ�Ƿ����
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ��Ϊ8λ
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 //�ڴ����ݿ��Ϊ8λ	
	DMA_InitStruct.DMA_Mode =   DMA_Mode_Normal;                         //��ͨ����ģʽ
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;                   //�����ȼ�
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;            				 //��ֹDMA2���ڴ��໥����
	DMA_Init(DMA2_Channel2, &DMA_InitStruct);        					 //��ʼ��DMA,SPI��DMA1��ͨ��2
	
	//ʹ��SPI2 DMA�ķ��͹���
	SPI_I2S_DMACmd(SPI3,SPI_I2S_DMAReq_Tx,ENABLE); 
	//ʹ��SPI3
	SPI_Cmd(SPI3, ENABLE);
}

//SPI3 DMA ���ݷ��ͺ���
void DMA_SPI3_TX(uint8_t* buffer,uint16_t len)
{
	DMA2->IFCR |=(0xf<<4);   				 //���ͨ��2�������еı�־λ
	DMA2_Channel2->CNDTR=len; 		 		 //����Ҫ��������ݳ���
	DMA2_Channel2->CMAR=(u32)buffer;		 //����RAM��������ַ
	DMA2_Channel2->CCR|=0x1;   				 //����DMA
        while(!(DMA2->ISR&(1<<5))) ; 		 //�ȴ��������ݴ������
	DMA2_Channel2->CCR &=(uint32_t)~0x1;	 //�ر�DMA
}
//LCDд����
void LCD_Writ_Bus(uint8_t dat) 
{	
	//Ƭѡ���ͺ���ܷ�������
	LCD_CS_Clr();
	//����һ���ֽ�
	DMA_SPI3_TX(&dat,1);
}
//LCD��
void hal_LCD_Display_on(void)
{
	LCD_EN_Set();
}
//LCD��
void hal_LCD_Display_off(void)
{
	LCD_EN_Clr();
}
//LCD��λ������
void hal_LCD_ResetH(void)
{
	LCD_RES_Set();
}
//LCD��λ������
void hal_LCD_ResetL(void)
{
	LCD_RES_Clr();
}
/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
	uint8_t d[2];
	d[0] = dat>>8;
	d[1] = dat;
	DMA_SPI3_TX(&d[0],2);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
	LCD_DC_Clr();//д����
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//д����
}















