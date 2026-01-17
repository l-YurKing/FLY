#ifndef __BALANCE_TASK_H
#define __BALANCE_TASK_H

#include <stdint.h>

//Dshot���ŷ�Χ(48~2047)
#define Dshot_MIN 48
#define Dshot_MAX 1500 //ʵ���������ֵ2047

//��������¼���
enum{
	UNUSE_HeightMode_Event = (1<<0),  //�޶���ģʽ�¼�
	IMU_CalibZeroDone_Event = (1<<1), //IMU�궨����¼�
	StartFly_Event = (1<<2),           //���������¼�
	LowPower_Event = (1<<3),          //�͵����¼�
	
	//��·�����4·�������
	TestMotorAup_Event   = ( 1<<4 ),
	TestMotorAdown_Event = ( 1<<5 ),
	TestMotorBup_Event   = ( 1<<6 ),
	TestMotorBdown_Event = ( 1<<7 ),
	TestMotorCup_Event   = ( 1<<8 ),
	TestMotorCdown_Event = ( 1<<9 ),
	TestMotorDup_Event   = ( 1<<10 ),
	TestMotorDdown_Event = ( 1<<11 ),
	TestMotorAllup_Event = ( 1<<12 ),
	TestMotorAlldown_Event=( 1<<13 ),
	
	//�������ģʽ
	TestMotorMode_Event =  ( 1<<14 ),
	FlyMode_HeadLessMode_Event = ( 1<<15 ),
	
	//����ģʽѡ��
	lidar_follow_mode = (1<<16),
	lidar_avoid_mode = (1<<17),
	
	/* �¼������23 */
};

//�������Դ,�������ȼ�����Ϊ�͵���
enum{
	IDLECmd = 0,//���ƿ���
	APPCmd = 1, //APP����,������ȼ�, 1
	PS2Cmd = 2,
	AviodCmd = 3,
};

//����������������
#define ENABLE_FLIGHT_DATA_OUTPUT 1  //���Ϊ1��������������,0�ر�
#define FLIGHT_DATA_USE_BLUETOOTH 1  //���Ϊ1ʹ������UART4,0ʹ��USART1

//������ƽṹ��
typedef struct{
	uint8_t source; //������Դ
	float pitch;   //ǰ�����,+ǰ��,-����
	float roll;    //���ҿ���,+����,-����
	float gyroz;     //�������,+��ʱ��,-˳ʱ��
	float height;  //�߶ȿ���
}FlyControlType_t;

extern float g_readonly_mainTaskFreq;
extern float angle_to_rad(float angle);


#endif /* __BALANCE_TASK_H */
