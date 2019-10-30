#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include <stdbool.h>
#include <stdint.h>
#include "user_common.h"

#pragma anon_unions

/* PID���� */
typedef struct{
	float Target; 			        //�趨Ŀ��ֵ
	float Measured; 				    //����ֵ
	float err; 						      //����ƫ��ֵ
	float err_last; 				    //��һ��ƫ��
	float err_beforeLast; 			//���ϴ�ƫ��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float pwm; 						      //pwm���
	uint32_t MaxOutput;				  //����޷�
  uint32_t IntegralLimit;			//�����޷� 
}incrementalpid_t;

typedef struct{
	float Target; 					    //�趨Ŀ��ֵ
	float Measured; 				    //����ֵ
	float err; 						      //����ƫ��ֵ
	float err_last; 				    //��һ��ƫ��
	float integral_err; 			  //����ƫ��֮��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float pwm; 						      //pwm���
	uint32_t MaxOutput;				  //����޷�
  uint32_t IntegralLimit;			//�����޷� 
}positionpid_t;

/* ���̵�� */
typedef struct{
	uint16_t realAngle;			    //�������Ļ�е�Ƕ�
	int16_t  realSpeed;			    //���������ٶ�
	int16_t  realCurrent;		    //��������ʵ�ʵ���
	
	uint8_t  temperture;        //�������ĵ���¶�
	
	int16_t  targetCurrent;			//Ŀ�����
	int16_t  targetSpeed;			  //Ŀ���ٶ�
	uint16_t targetAngle;			  //Ŀ��Ƕ�
	
	int16_t  outCurrent;				//�������
	
	incrementalpid_t pid;		    //���pid
	
	uint8_t  M3508InfoUpdateFlag;		  //��Ϣ��ȡ���±�־
	uint16_t M3508InfoUpdateFrame;	  //֡��
	uint8_t  M3508OffLineFlag;		    //�豸���߱�־
}M3508s_t;

/* �����˶� */
typedef struct{
	float targetXRaw;		//����x��ԭʼ����
	float targetYRaw;		//����y��ԭʼ����
	float targetZRaw;		//����z��ԭʼ����
	
	float LpfAttFactor;	//�����˲�ϵ��
	
	float targetXLPF;		//����x���˲�������
	float targetYLPF;		//����y���˲�������
	float targetZLPF;		//����z���˲�������
	
	float speedLimit;		//�����ٶ�����
	
	float FollowtargetYawRaw;	              //����Ŀ��Yaw�������̨ԭʼ����
	float FollowtargetYawLPF;              	//����Yaw�������̨�˲�������
	
	positionpid_t FollowYawAttitude_pid;		//����Yaw�������̨pid
	
	uint8_t mode;							              //���̿���ģʽ
	uint8_t swingFlag;				              //Ť����־λ
	float swingSpeed;					              //Ť���ٶ�
	uint8_t PowerOverflowFlag;				    	//�����ʱ�־λ
}Chassis_t;

/* ��̨��� */
typedef struct{
	
	uint16_t realAngle;			      //�������Ļ�е�Ƕ�
	int16_t  realSpeed;			      //���������ٶ�
	int16_t  realTorque;		      //��������ʵ��ת��
	int16_t  temperture;          //���������¶�
	
	int16_t  targetCurrent;		    //Ŀ�����	
	int16_t  targetAngle;		      //Ŀ��Ƕ�
	int32_t  last_targetAngle;		//�ϴ�Ŀ��Ƕ�
	int16_t  countAngle;          //�����Ƕ�
	int32_t  targetSpeed;			    //Ŀ���ٶ�
	
	int16_t  outCurrent;				  //�������
  int16_t  inneroutCurrent;
	
	uint16_t lastAngle;		        //�ϴεĽǶ�
	int16_t  turnCount;			      //ת����Ȧ��
	int32_t  totalAngle;			    //�Ƕ��ۼ�ֵ
	
	positionpid_t pid_Speed;			//��̨pid
	positionpid_t pid_Angle;
	
	uint8_t  M6020InfoUpdateFlag;	//��Ϣ��ȡ���±�־
	uint16_t M6020InfoUpdateFrame;//֡��
	uint8_t  M6020OffLineFlag;		//�豸���߱�־

}M6020s_t;

/* ��̨�˶�*/
typedef struct {
	float LpfAttFactor;			   //��̨�˲�ϵ��
	
	float targetYawRaw;			   //��̨Ŀ��yaw��ԭʼ����
	float targetPitchRaw;		   //��̨Ŀ��pitch��ԭʼ����
	
	float IMUtargetYawRaw;		 //��������̨Ŀ��yaw��ԭʼ����	
	float IMUtargetPitchRaw;	 //��������̨Ŀ��pitch��ԭʼ����

	float targetYawLPF;			   //��̨yaw���˲�������
	float targetPitchLPF;		   //��̨pitch���˲�������
	
	float IMUtargetYawLPF;		 //��̨yaw���˲�������
	float last_IMUtargetYawLPF;//��������̨yaw���˲�������
	float IMUtargetPitchLPF;   //��̨pitch���˲�������
	
	positionpid_t YawAttitude_pid;			//��̨yaw����̬pid
	positionpid_t YawSpeed_pid;			    //��̨yaw���ٶ�pid
	positionpid_t PitchAttitude_pid;	  //��̨Pitch����̬pid
	positionpid_t PitchSpeed_pid;			  //��̨Pitch���ٶ�pid

	float IMUYawAngleMax;		  //��̨IMU���Ƕ�(��)
	float IMUYawAngleMin;		  //��̨IMU��С�Ƕ�(��)
	float IMUPitchAngleMax;		//��̨IMU���Ƕ�(��)
	float IMUPitchAngleMin;		//��̨IMU��С�Ƕ�(��)

	

	uint8_t Mode;					            	//��̨����ģʽ
}Cloud_t;

/* ������ */
typedef struct {
		float x;                 //������pitch��ķ�������
		float y;                 //������Y�᷽������
		float z;                 //������yaw���ת���ٶ�
}Vector_t;

typedef struct {
	float Roll;                 //ROLL�᷽�򣬵�ǰ�ĽǶ�ֵ
	float Pitch;                //PITCH�᷽��
	float Yaw;                  //YAW�᷽��
}Eular_t;


typedef struct {
	Vector_t Gyro;              //�������ٶ�ֵ����          
	Eular_t  Eular;             //ŷ��������     
	float    lastYaw;           //��һ��YAW������    
	float    targetYaw;
	float    totalYaw;
	int16_t  turnCount;

	uint8_t  ImuDevStatus;
	uint8_t  ImuInfoUpdateFlag;
	uint16_t ImuInfoUpdateFrame;
	uint8_t  ImuOffLineFlag;
}GY_IMU_t;

/* ң�� */
typedef struct {
	struct{
		int16_t ch0;   
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		uint8_t s_left;
		uint8_t s_right;
	}rc;
	
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t keyLeft;
		uint8_t keyRight;
		
	}mouse;
	
	union {
		uint16_t key_code;
		struct{
			bool press_W:1;
			bool press_S:1;
			bool press_A:1;
			bool press_D:1;
			bool press_Shift:1;
			bool press_Ctrl:1;
			bool press_Q:1;
			bool press_E:1;
			
			bool press_R:1;
			bool press_F:1;
			bool press_G:1;
			bool press_Z:1;
			bool press_X:1;
			bool press_C:1;
			bool press_V:1;
			bool press_B:1;
		};
	}keyBoard;
	
	uint16_t DR16InfoUpdateFrame;	//֡��
	uint8_t DR16OffLineFlag;	  	//�豸���߱�־
}DR16_DBUS_DATA_t;

typedef enum{
	ControlMode_CL = 3,
	ControlMode_HL = 2,
	ControlMode_OFF = 1,
}ControlMode_e;

typedef enum{
	ControlMode_ATTI1 = 3,
	ControlMode_GPS = 1,
	ControlMode_ATTI2 = 2,
}Mode_e;

typedef enum{
	SpecialMode_Normal = 0,
	SpecialMode_BigSymbol = 2,	//���ģʽ
	SpecialMode_UPBridge = 3,	  //����ģʽ
	SpecialMode_Supply = 4, 	  //����վģʽ
}SpecialMode_t;

typedef struct{
	uint8_t ControlLeft;
	uint8_t ControlRight;		   	//����ģʽ
}ControlStatus_t;

//���̹���ģʽ
typedef enum
{
	ChassisWorkMode_Follow = 0,		//������̨ģʽ
	ChassisWorkMode_Spin = 1,			//���߱���ģʽ
	ChassisWorkMode_Twister , 	  //Ť��ģʽ
	ChassisWorkMode_AutoTrace ,	  //�Զ�׷��ģʽ
	ChassisWorkMode_Supply,				//����ģʽ
	ChassisWorkMode_Disable				//����ģʽ
}ChassisWorkMode_e;

typedef struct {
	float x;
	float y;
	float radian;
	float degrees;
	float distance;
}rocker_t;

#endif   
