/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author   
  * @version  
  * @date    
  * @brief   USARTӦ�ú����ӿ�
  ******************************************************************************
  */
  
#include "bsp_usart.h"

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h" 
#endif

static USART_TypeDef* Using_USART = USART2;

char USART2_buff[USART2_BUFFLEN];
uint8_t USART2_Pointer = 0;

/**
  * @brief  USART1���ٳ�ʼ��
  * @param  USART_BaudRate 	����������
  * @retval None
  * @Using 	���ڽ���DR16���ջ�Dbus����
  */
	
void USART1_QuickInit(uint32_t USART_BaudRate){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(USART1_TX_GPIO_CLK | USART1_RX_GPIO_CLK,ENABLE);/* ʹ�� GPIO ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                 /* ʹ�� USART1 ʱ�� */
  
	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	/* ����Tx����  */
	GPIO_InitStructure.GPIO_Pin = USART1_TX_Pin;  
	GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx���� */
	GPIO_InitStructure.GPIO_Pin = USART1_RX_Pin;
	GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(USART1_TX_GPIO_PORT, USART1_TX_PINSOURCE, GPIO_AF_USART1);

	/*  ���� PXx �� USARTx_Rx*/
	GPIO_PinAFConfig(USART1_RX_GPIO_PORT, USART1_RX_PINSOURCE, GPIO_AF_USART1);
  
	/* ���ô�USART ģʽ */
	USART_InitStructure.USART_BaudRate = USART_BaudRate;       /* ���������ã�USART_BAUDRATE */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;/* ���ݳ���(����λ+У��λ)��8 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;     /* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_Parity = USART_Parity_No;        /* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	
	USART_Init(USART1, &USART_InitStructure); /* ���USART��ʼ������ */

	NVIC_Config(USART1_IRQn, 0, 2);/* �����ж����ȼ����� */
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);/* ʹ�ܴ��ڿ����ж� */
	
	USART_Cmd(USART1, ENABLE);/* ʹ�ܴ��� */
	
}


/**
  * @brief  USART1 RX DMA ����    ���赽�ڴ�(USART1->DR)
  * @param  USART1_DMABuff_addr		DMAĿ��洢��ַ
  * @retval ��
  */
void USART1_RXDMA_Config(uint32_t USART1_DMABuff_addr, uint32_t buffsize){
	
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	/*����DMA2ʱ��*/
	  
	DMA_DeInit(USART1_RX_DMA_STREAM);/* ��λ��ʼ��DMA������ */

  /* ȷ��DMA��������λ��� */
	while(DMA_GetCmdStatus(USART1_RX_DMA_STREAM) != DISABLE){}
		
	/*usart1 rx��Ӧdma2��ͨ��4��������2*/	
	DMA_InitStructure.DMA_Channel = USART1_RX_DMA_CHANNEL;  
	/*����DMAԴ���������ݼĴ�����ַ*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_RX_ADDR;	 
	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
	DMA_InitStructure.DMA_Memory0BaseAddr = USART1_DMABuff_addr;
	/*���򣺴����赽�ڴ�*/		
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
	DMA_InitStructure.DMA_BufferSize = buffsize;
	/*�����ַ����*/	    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	/*�ڴ��ַ����*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	/*�������ݵ�λ*/	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/*�ڴ����ݵ�λ 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
	/*DMAģʽ������ѭ��*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/*���ȼ�����*/	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
	/*����FIFO*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;    
	/*�洢��ͻ������ 16 ������*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
	/*����ͻ������ 1 ������*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
	
	/*����DMA2��������*/		   
	DMA_Init(USART1_RX_DMA_STREAM, &DMA_InitStructure);
	/*ʹ��DMA*/
	DMA_Cmd(USART1_RX_DMA_STREAM, ENABLE);
	
	/* �ȴ�DMA��������Ч*/
	while(DMA_GetCmdStatus(USART1_RX_DMA_STREAM) != ENABLE){}
  
	//ʹ��DMA����  
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); 
  
}


/**
  * @brief  USART2���ٳ�ʼ��
  * @param  USART_BaudRate 	����������
  * @retval None
  * @Using 	����pc�˷��ͣ����յ�����Ϣ
  */
void USART2_QuickInit(uint32_t USART_BaudRate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	/* ʹ�� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(USART2_TX_GPIO_CLK | USART2_RX_GPIO_CLK,ENABLE);

	/* ʹ�� USART2 ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	/* ����Tx����  */
	GPIO_InitStructure.GPIO_Pin = USART2_TX_Pin;  
	GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx���� */
	GPIO_InitStructure.GPIO_Pin = USART2_RX_Pin;
	GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(USART2_TX_GPIO_PORT, USART2_TX_PINSOURCE, GPIO_AF_USART2);

	/*  ���� PXx �� USARTx_Rx*/
	GPIO_PinAFConfig(USART2_RX_GPIO_PORT, USART2_RX_PINSOURCE, GPIO_AF_USART2);
  
	/* ���ô�USART ģʽ */
	/* ���������ã�USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = USART_BaudRate;
	/* ���ݳ���(����λ+У��λ)��8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* ���USART��ʼ������ */
	USART_Init(USART2, &USART_InitStructure); 
	
	/* �����ж����ȼ����� */
	NVIC_Config(USART2_IRQn, 0, 3);
  
	/* ʹ�ܴ��ڽ����ж� */
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	/* ʹ�ܴ��ڿ����ж� */
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	
	/* ʹ�ܴ��� */
	USART_Cmd(USART2, ENABLE);
}


/**
  * @brief  USART2 RX DMA ���ã����赽�ڴ�(USART2->DR)
  * @param  USART2_DMABuff_addr		DMAĿ��洢��ַ
  * @retval ��
  */
void USART2_RXDMA_Config(uint32_t USART2_DMABuff_addr, uint32_t buffsize)
{
	DMA_InitTypeDef DMA_InitStructure;

	/*����DMA1ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	  
	/* ��λ��ʼ��DMA������ */
	DMA_DeInit(USART2_RX_DMA_STREAM);

	/* ȷ��DMA��������λ��� */
	while(DMA_GetCmdStatus(USART2_RX_DMA_STREAM) != DISABLE){
	}

	/*usart2 rx��Ӧdma1��ͨ��4��������5*/	
	DMA_InitStructure.DMA_Channel = USART2_RX_DMA_CHANNEL;  
	/*����DMAԴ���������ݼĴ�����ַ*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_RX_ADDR;	 
	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
	DMA_InitStructure.DMA_Memory0BaseAddr = USART2_DMABuff_addr;
	/*���򣺴����赽�ڴ�*/		
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
	DMA_InitStructure.DMA_BufferSize = buffsize;
	/*�����ַ����*/	    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	/*�ڴ��ַ����*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	/*�������ݵ�λ*/	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/*�ڴ����ݵ�λ 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
	/*DMAģʽ������ѭ��*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/*���ȼ�����*/	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
	/*����FIFO*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;    
	/*�洢��ͻ������ 16 ������*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
	/*����ͻ������ 1 ������*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
	/*����DMA2��������*/		   
	DMA_Init(USART2_RX_DMA_STREAM, &DMA_InitStructure);
	  
	/*ʹ��DMA*/
	DMA_Cmd(USART2_RX_DMA_STREAM, ENABLE);
  
	/* �ȴ�DMA��������Ч*/
	while(DMA_GetCmdStatus(USART2_RX_DMA_STREAM) != ENABLE){
	}
  
	//ʹ��DMA����  
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE); 
  
}


/**
  * @brief  USART3���ٳ�ʼ��
  * @param  USART_BaudRate 	����������
  * @retval None
  * @Using 	����pc�˷��ͣ����յ�����Ϣ
  */
void USART3_QuickInit(uint32_t USART_BaudRate){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ʹ�� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(USART3_TX_GPIO_CLK | USART3_RX_GPIO_CLK,ENABLE);

	/* ʹ�� USART2 ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	/* ����Tx����  */
	GPIO_InitStructure.GPIO_Pin = USART3_TX_Pin;  
	GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx���� */
	GPIO_InitStructure.GPIO_Pin = USART3_RX_Pin;
	GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(USART3_TX_GPIO_PORT, USART3_TX_PINSOURCE, GPIO_AF_USART2);

	/*  ���� PXx �� USARTx_Rx*/
	GPIO_PinAFConfig(USART3_RX_GPIO_PORT, USART3_RX_PINSOURCE, GPIO_AF_USART2);
  
	/* ���ô�USART ģʽ */
	/* ���������ã�USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = USART_BaudRate;
	/* ���ݳ���(����λ+У��λ)��8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* ���USART��ʼ������ */
	USART_Init(USART3, &USART_InitStructure); 
	
	/* �����ж����ȼ����� */
	NVIC_Config(USART3_IRQn, 0, 3);
  
	/* ʹ�ܴ��ڽ����ж� */
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	/* ʹ�ܴ��ڿ����ж� */
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	/* ʹ�ܴ��� */
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  USART3 RX DMA ���ã����赽�ڴ�(USART3->DR)
  * @param  USART3_DMABuff_addr		DMAĿ��洢��ַ
  * @retval ��
  */
void USART3_RXDMA_Config(uint32_t USART3_DMABuff_addr, uint32_t buffsize)
{
	DMA_InitTypeDef DMA_InitStructure;

	/*����DMA1ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	  
	/* ��λ��ʼ��DMA������ */
	DMA_DeInit(USART3_RX_DMA_STREAM);

	/* ȷ��DMA��������λ��� */
	while(DMA_GetCmdStatus(USART3_RX_DMA_STREAM) != DISABLE){
	}

	/*USART3 rx��Ӧdma1��ͨ��4��������5*/	
	DMA_InitStructure.DMA_Channel = USART3_RX_DMA_CHANNEL;  
	/*����DMAԴ���������ݼĴ�����ַ*/
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_RX_ADDR;	 
	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
	DMA_InitStructure.DMA_Memory0BaseAddr = USART3_DMABuff_addr;
	/*���򣺴����赽�ڴ�*/		
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
	DMA_InitStructure.DMA_BufferSize = buffsize;
	/*�����ַ����*/	    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	/*�ڴ��ַ����*/
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	/*�������ݵ�λ*/	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	/*�ڴ����ݵ�λ 8bit*/
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
	/*DMAģʽ������ѭ��*/
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	/*���ȼ�����*/	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
	/*����FIFO*/
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;    
	/*�洢��ͻ������ 16 ������*/
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
	/*����ͻ������ 1 ������*/
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
	/*����DMA2��������*/		   
	DMA_Init(USART3_RX_DMA_STREAM, &DMA_InitStructure);
	  
	/*ʹ��DMA*/
	DMA_Cmd(USART3_RX_DMA_STREAM, ENABLE);
  
	/* �ȴ�DMA��������Ч*/
	while(DMA_GetCmdStatus(USART3_RX_DMA_STREAM) != ENABLE){
	}
  
	//ʹ��DMA����  
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 
  
}

/**
  * @brief  USART����һ���ַ�
  * @param  USARTx 	USART����
  *		      ch   		�ַ�
  * @retval None
  */
void USART_sendChar(USART_TypeDef* USARTx, char ch){
	
	/* ����һ���ֽ����ݵ����� */
	USART_SendData(USARTx, (uint8_t) ch);
	
	/* �ȴ�������� */
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);	
	
}


/**
  * @brief  USARTʹ��printf��scanf�˿�����
  * @param  USARTx 	USART����
  * @retval None
  */
void USART_setPort(USART_TypeDef* USARTx){
	Using_USART = USARTx;
}


///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(Using_USART, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(Using_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}


///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(Using_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(Using_USART);
}
