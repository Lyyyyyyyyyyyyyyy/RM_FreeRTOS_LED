/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  
  * @version
  * @date    
  * @brief   CANӦ�ú����ӿ�
  ******************************************************************************
  */
  
  
#include "bsp_can.h"

/**
  * @brief  CAN1���ٳ�ʼ��
  * @param  None
  * @retval None
  */
	
void CAN1_QuickInit(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	RCC_AHB1PeriphClockCmd(CAN1_RX_GPIO_CLK | CAN1_TX_GPIO_CLK, ENABLE);    /* ʹ�� GPIO ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	                  /* ʹ��CAN1ʱ�� */
	
	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	
	/* ����Tx����  */
	GPIO_InitStructure.GPIO_Pin = CAN1_TX_Pin;  
	GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx���� */
	GPIO_InitStructure.GPIO_Pin = CAN1_RX_Pin;
	GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_TX_PINSOURCE, GPIO_AF_CAN1); 	/*  ���� IO �� CAN1_Tx*/
	GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_RX_PINSOURCE, GPIO_AF_CAN1); 	/*  ���� IO �� CAN1_Rx*/
	
	/*************************************CAN ģʽ����*********************************************/
	CAN_InitStructure.CAN_TTCM = DISABLE;			   //MCR-TTCM  ʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_ABOM = ENABLE;			   //MCR-ABOM  �Զ����߹��� 
	CAN_InitStructure.CAN_AWUM = ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART = DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM = DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP = DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ��
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;//Mode      ����ģʽ
	
	/* ss=1 bs1=5 bs2=3 λʱ����Ϊ(1+3+5) �����ʼ�Ϊʱ������tq*(1+3+5)  */
	CAN_InitStructure.CAN_SJW = CAN1_SJW;		   //BTR-SJW ����ͬ����Ծ���2��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS1 = CAN1_BS1;		   //BTR-TS1 ʱ���1 ռ����4��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2 = CAN1_BS2;		   //BTR-TS1 ʱ���2 ռ����6��ʱ�䵥Ԫ	
	
	/* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 45 MHz) */
	CAN_InitStructure.CAN_Prescaler = CAN1_Prescaler;		   //BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 45/(1+3+5)/5=1 Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
	
	/*********************CANɸѡ����ʼ��***********************************************/
	CAN_FilterInitStructure.CAN_FilterNumber = 0;									          //ɸѡ����0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;					//������ID����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;				//ɸѡ��λ��Ϊ����32λ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;		//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;			            //ʹ��ɸѡ��
	
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
	
	/* �����й��� */
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;				//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000; 				//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;		//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;			//ɸѡ����16λÿλ����ƥ��
	
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	NVIC_Config(CAN1_RX0_IRQn, 1, 1);
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);       /* FIFO0 message pending interrupt */
	
}

/**
  * @brief  CAN��������
  * @param  CANx 		  CAN���
  * 	    	id_type ��	id���� CAN_ID_STD����׼���� CAN_ID_EXT����չ��
  *			    id			  id��
  * 	    	data[8]		8������
  * @retval None
  */

void CAN_SendData(CAN_TypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8]){	  
	
	CanTxMsg TxMessage;
	
	if(id_type == CAN_Id_Standard){
		
		TxMessage.StdId = id;					 
		
	}
	else{
		
		TxMessage.ExtId = id;					 //ID��
		
	}
	
	TxMessage.IDE = id_type;				 //ID����
	TxMessage.RTR = CAN_RTR_DATA;		 //���͵�Ϊ����
	TxMessage.DLC = 0x08;						 //���ݳ���Ϊ8�ֽ�
	
	/*����Ҫ���͵�����*/
	for (uint8_t i = 0; i < 8; i++)
	{
		TxMessage.Data[i] = data[i];
	}
  
	CAN_Transmit(CANx, &TxMessage);
	
}

