#include "balance_task.h"

/* C Lib */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* RTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

/* BSP */
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "bsp_ps2.h"
#include "bsp_dshot.h"
#include "bsp_adc.h"
#include "bsp_Rtosdebug.h"
#include "bsp_imu.h"
#include "bsp_stp23L.h"
#include "bsp_adc.h"
#include "main.h"

#include "lidar_task.h"
#include "show_task.h"
#include "pid.h"

typedef struct {
	dshotMotorVal_t A;
	dshotMotorVal_t B;
	dshotMotorVal_t C;
	dshotMotorVal_t D;
}MOTOR_t; /* ����4���������������� */

typedef struct{
	IMU_DATA_t* axis;
	ATTITUDE_DATA_t* attitude;
}IMU_ZEROPONIT_t; /* imu���ṹ�壬�������궨 */

//���ֹͣĬ��ֵ
static MOTOR_t MotorStopVal = {
	.A = {0,Dshot_MIN},
	.B = {0,Dshot_MIN},
	.C = {0,Dshot_MIN},
	.D = {0,Dshot_MIN},
};

//�������ֵ
static MOTOR_t motor = {
	.A = {0,Dshot_MIN},
	.B = {0,Dshot_MIN},
	.C = {0,Dshot_MIN},
	.D = {0,Dshot_MIN},
}; 

//��ʱ�����
static TimerHandle_t priv_BuzzerTipsTimer,priv_WaitImuTipsTimer,priv_OperateResponseTimer,priv_OperateFullTimer,priv_UNUSEHeightTimer,priv_lowpowerTimer; 
static TimerHandle_t follow_tipsTimer;

//4���¼�����
extern EventGroupHandle_t g_xEventFlyAction;

/* �ڲ�ʹ�ú��� */
static void SetPwm(MOTOR_t* m);
static IMU_ZEROPONIT_t* WaitImuStable(uint16_t freq,IMU_DATA_t imu,ATTITUDE_DATA_t attitude);
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer);
static void LedTipsTimer_Callback(TimerHandle_t xTimer);
static void OperateResponse_Callback(TimerHandle_t xTimer);
static void OperateFull_Callback(TimerHandle_t xTimer);
static void UNUSEHeightTips_Callback(TimerHandle_t xTimer);
static void LowPowerTips_Callback(TimerHandle_t xTimer);
static int target_limit_s16(short insert,short low,short high);
static float target_limit_float(float insert,float low,float high);
static uint8_t check_HeightStable(float height);
static void StopVal_SelfRecovery(MOTOR_t* m);
static uint16_t weight_to_throttle(float weight,float height);
static void FollwTips_Callback(TimerHandle_t xTimer);
/* �ڲ�ʹ�ú���  END */

//balance����ļ��Ƶ��
float g_readonly_BalanceTaskFreq = 0;

//������Ŀ�����
#define DefalutHeight 1.0f  //����ʱĬ�ϵķ��и߶�
static float FlyControl_pitch = 0;
static float FlyControl_roll = 0;
static float FlyControl_yaw = 0;
static float FlyControl_gyroz = 0;
static float FlyControl_height = 0;
static float FlyControl_unuseHeight = 0.0f;

//΢������ֵ
static float userset_pitch = 0;
static float userset_roll = 0;

//�����������ȼ���������
static uint8_t controlCmdNumber = IDLECmd;

//������������
#if ENABLE_FLIGHT_DATA_OUTPUT
static uint16_t dataOutputCounter = 0;
#endif

//��ɵ�ѹ����
#define ROBOT_VOL_LIMIT 10.0f

//����ģʽ,�������С�߶�����
#define HeightMode_MAX 1.5f
#define HeightMode_Min -0.5f

//�Ƕ���ģʽ,�������С�߶�����
#define UNHeightMode_MAX 5.0f
#define UNHeightMode_Min -0.5f

//PID������
extern PIDControllerType_t RollRatePID,RollPID;
extern PIDControllerType_t PitchRatePID,PitchPID;
extern PIDControllerType_t YawRatePID,YawPID;
extern PIDControllerType_t HeightSpeedPID,HeightPID;

extern void PID_Update(PIDControllerType_t* pid,float target,float current);
extern void PID_Reset(PIDControllerType_t* pid);

extern uint8_t g_lost_pos_dev;

//Z��Ƕȹ�һ������
#define M_PI 3.14159265f
static float normalize_radian(float angle) {
	const float TWO_PI = 2.0f * M_PI; // 2�� �� 6.283185307
	while (angle > M_PI) {
		angle -= TWO_PI;
	}
	while (angle < -M_PI) {
		angle += TWO_PI;
	}
	return angle;
}

//ʵ��ʹ�õĸ߶�ֵ
float use_distance = 0;

/* �����˲���Ľ��ٶ� */
float gx_filtered = 0, gy_filtered = 0; 

//�����ٶȡ�λ�á�Ŀ��λ��
float speedX=0,speedY=0;
float posX=0,posY=0;
float targetPosX=0,targetPosY=0;

//���㹦�ܿ�����־λ
uint8_t startPos = 1;

// �ٶ�(�����ںϺ��)
static float x_dot = 0, y_dot = 0; 


// �������ݻص�����
void getOpticalFlowResult_Callback(float* buf)
{
	speedY = -buf[0] / 200.0f; // �ٶ�(m/s)
	speedX = -buf[1] / 200.0f; // �ٶ�(m/s)
	
	speedY = use_distance * (speedY - fmaxf(fminf(gx_filtered,2.0f),-2.0f)); // ��ת���� + �߶�����
	speedX = use_distance * (speedX + fmaxf(fminf(gy_filtered,2.0f),-2.0f)); // ��ת���� + �߶�����
	
	g_lost_pos_dev=0;
}

void avoid_targetpos_lc307(uint8_t flag)
{
	xTimerStart(priv_OperateFullTimer,0);
	
	switch( flag )
	{
		case 0:
			targetPosX += 0.03f;//��
			break;
		case 1:
			targetPosX -= 0.03f;//��
			break;
		case 2:
			targetPosY += 0.03f;//ǰ
			break;
		case 3:
			targetPosY -= 0.03f;//��
			break;
		default:
			break;
	}
}

//����
/*
 ���룺0.3����ΪĿ��ֵ,̫�����������˶�
       0.5��һֱ������0.2��ԭ��
*/
//
void follow_targetpos_lc307(uint8_t flag)
{
	//1s��������ʾ
	static TickType_t LastTick;
	
	TickType_t now = xTaskGetTickCount();
	if( now - LastTick >= 1300 )
	{
		xTimerStart(follow_tipsTimer,0);
		LastTick = now;
	}
	
	switch( flag )
	{
		//����ʹ��
		case 0:
			targetPosX -= 0.01f;//��
			break;
		case 1:
			targetPosX += 0.01f;//��
			break;
		case 2:
			targetPosY -= 0.01f;//ǰ
			break;
		case 3:
			targetPosY += 0.01f;//��
			break;
		
		//Զ��ʹ��
		case 4:
			targetPosX += 0.005f;//��
			break;
		case 5:
			targetPosX -= 0.005f;//��
			break;
		case 6:
			targetPosY += 0.005f;//ǰ
			break;
		case 7:
			targetPosY -= 0.005f;//��
			break;
		
		default:
			break;
	}
}

//��������λ��
void set_targetpos_lc307(uint8_t flag)
{
	xTimerStart(priv_OperateResponseTimer,0);
	switch( flag )
	{
		case 0:
			targetPosX += 0.2f;//B
			break;
		case 1:
			targetPosX -= 0.2f;//X
			break;
		case 2:
			targetPosY += 0.2f;//Y
			break;
		case 3:
			targetPosY -= 0.2f;//A
			break;
		default:
			break;
	}
}

void start_lc307_pos(void)
{
	startPos = !startPos;
}

void balance_task(void* param)
{
	//�����������Ķ���
	extern QueueHandle_t g_xQueueFlyControl;
	
	//��ȡʱ��,���ڸ���������ָ���̶�Ƶ������
	TickType_t preTime = xTaskGetTickCount();
	
	//���������Ƶ��,��λHZ
	const uint16_t TaskFreq = 200;
	
	//����Ƶ�ʡ�ʱ����Ա���
	pRtosDebugInterface_t debug = &RTOSTaskDebug;
	RtosDebugPrivateVar debugPriv = { 0 };
	
	//ָ���õ�������
	pIMUInterface_t imu = &UserICM20948;
	
	//ADC����
	pADCInterface_t adc1 = &UserADC1;
	
	//IMU���,ϵͳ����ʱ���������ʱ,ִ�б궨
	IMU_ZEROPONIT_t* zero_point = { NULL };
	
	//���ڴ��IMU����
	IMU_DATA_t axis_9Val = { 0 };                 
	ATTITUDE_DATA_t AttitudeVal = { 0 };
	
	//ϵͳʱ��,���ڸ���IMU�궨
	uint16_t syscount = 0;
	
	//������ͬ���͵ķ�������ʾ��ʱ��
	priv_BuzzerTipsTimer = xTimerCreate("BuzzerTips",pdMS_TO_TICKS(10),pdFALSE,NULL,BuzzerTipsTimer_Callback);
	priv_WaitImuTipsTimer = xTimerCreate("WaitImuTips",pdMS_TO_TICKS(100),pdTRUE,NULL,LedTipsTimer_Callback);
	priv_OperateResponseTimer = xTimerCreate("priv_OperateResponseTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,OperateResponse_Callback);
	priv_OperateFullTimer = xTimerCreate("priv_OperateFullTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,OperateFull_Callback);
	priv_UNUSEHeightTimer = xTimerCreate("UnUseHeightTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,UNUSEHeightTips_Callback);
	priv_lowpowerTimer = xTimerCreate("LowPowerTimer",pdMS_TO_TICKS(100),pdFALSE,NULL,LowPowerTips_Callback);
	
	follow_tipsTimer = xTimerCreate("followtipsTimer",pdMS_TO_TICKS(1),pdFALSE,NULL,FollwTips_Callback);
	
	xTimerStart(priv_WaitImuTipsTimer,0);
	
	//�¼���־λ,���ڻ�ȡ������Ƶ��¼�
	EventBits_t uxBits ; 
	
	//�߶����
	float zero_distance = 0;
	
	//�����ɻ�ʱ���궨��־λ
	uint8_t StarFly_UpdateFlag = 1;
	
	//�߶�ƽ�����Ʊ�־λ
	uint8_t StartFly_SmoothHeightFlag = 1;
	
	//RTOSʹ�ø���
	portTASK_USES_FLOATING_POINT();
	
	//��ȡ�û��趨�Ļ������ֵ
	extern float g_userparam_pitchzero,g_userparam_rollzero;
	userset_pitch = g_userparam_pitchzero;
	userset_roll = g_userparam_rollzero;
	
	while(1)
	{
		//����ȥ������ĸ߶�
		use_distance = zero_distance - g_readonly_distance;
		
		imu->Update_9axisVal(&axis_9Val);           //���������ݸ���,��ȡ�����ݾ�Ϊԭʼ����.�˲�����ʱ 0.61 ms
		
		gx_filtered = 0.9f * gx_filtered + 0.1f * axis_9Val.gyro.x; /* һ�׵�ͨ�˲� */
		gy_filtered = 0.9f * gy_filtered + 0.1f * axis_9Val.gyro.y; /* һ�׵�ͨ�˲� */
		
		imu->UpdateAttitude(axis_9Val,&AttitudeVal);//������̬��
		
		//��ѹ��ȡ
		g_robotVOL = (float)adc1->getValue(userconfigADC_VBAT_CHANNEL)/4095.0f*3.3f*11.0f;
		
		//ϵͳ����ʱ��,30����ټ�ʱ
		if( syscount < TaskFreq*30 ) syscount++; 
		
		//��ȡ��������ص��¼���
		uxBits = xEventGroupGetBits(g_xEventFlyAction);
		
		/* �궨������ */
		if( 0 == (uxBits & IMU_CalibZeroDone_Event) )
		{  
			/* ����5�����ִ�б궨,�ȴ��ڼ���Ҫ��������ˮƽ��ֹ */
			if( syscount>= TaskFreq * 5 )
			{
				/* �ȴ�imu�ȶ����궨 */
				zero_point = WaitImuStable(TaskFreq,axis_9Val,AttitudeVal);        
				
				/* ���߶������Ƿ��ȶ� */
				uint8_t heightstatble = check_HeightStable(g_readonly_distance);
				
				//�궨˳��6�����ݡ���̬��
				if( zero_point->axis!=NULL )
				{
					imu->UpdateZeroPoint_axis(zero_point->axis);
				}
				
				if( zero_point->attitude!=NULL )
				{
					imu->UpdateZeroPoint_attitude(zero_point->attitude);           //imu��̬������
					zero_distance = g_readonly_distance;                           //�߶�������
					xTimerChangePeriod(priv_WaitImuTipsTimer,pdMS_TO_TICKS(800),0);//ϵͳ��������״̬,LED������ʾ
					xEventGroupSetBits(g_xEventFlyAction,IMU_CalibZeroDone_Event); //�����¼�,IMU�궨���
					
					/* �߶���Ϣ�ȶ�ֵ�쳣(����������).���벻����ģʽ */
					if( heightstatble >= 6 )
					{
						xEventGroupSetBits(g_xEventFlyAction,UNUSE_HeightMode_Event);
						xTimerStart(priv_UNUSEHeightTimer,0);
					}
					else /* ����ģʽ */
					{
						xTimerStart(priv_BuzzerTipsTimer,0); //��������ʾ�궨�����
					}
						
					
					/* ���Ʒ�ʽ,Ĭ����ͷģʽ */
					//xEventGroupSetBits(g_xEventFlyAction,FlyMode_HeadLessMode_Event);
				}
			}
		}
		/* �궨������ END */
		
		/* �߶���Ϣ���� */
		static float last_height=0,height_dot=0;
		static float last_height_dot = 0;
		height_dot = 0.4f*(use_distance - last_height)/0.005f + 0.6f*last_height_dot; // һ�׵�ͨ�˲�
		last_height_dot = height_dot;
		last_height = use_distance;
		
		if( uxBits & UNUSE_HeightMode_Event )
		{   /* ������ģʽ */
			use_distance = FlyControl_height; 
			height_dot = 0;
			startPos=0;
		}
		/* �߶���Ϣ���� END*/
		
		/* ��ȡ����ָ�� */
		FlyControlType_t controlVal = { 0 };
		static FlyControlType_t lastControlVal = { 0 }; //�ϴο���ָ��
		static uint8_t refresh_cmdstate = 0;
		
		// 指令平滑处理（Slew Rate Limiter）- 解决手动操控抖动问题
		static float target_pitch_smooth = 0;
		static float target_roll_smooth = 0;
		const float max_pitch_rate = 0.5f;  // 度/周期，限制每周期最大变化量（200Hz，0.005秒/周期）
		const float max_roll_rate = 0.5f;   // 10度在0.15秒内完成 = 10 / (0.15/0.005) = 0.33度/周期，设为0.5更平滑
		if( pdPASS == xQueueReceive(g_xQueueFlyControl,&controlVal,0) )
		{
			lastControlVal = controlVal; //�������µ�����ָ��
			refresh_cmdstate = 0;
			
			//��ͷģʽ
			if( uxBits & FlyMode_HeadLessMode_Event )
			{
				float raw_pitch = -controlVal.roll * sin(AttitudeVal.yaw) + controlVal.pitch * cos(AttitudeVal.yaw);
				float raw_roll =  controlVal.roll * cos(AttitudeVal.yaw) + controlVal.pitch * sin(AttitudeVal.yaw);
				
				// 平滑处理Pitch
				float pitch_diff = raw_pitch - target_pitch_smooth;
				if (fabs(pitch_diff) > max_pitch_rate) {
					target_pitch_smooth += (pitch_diff > 0 ? max_pitch_rate : -max_pitch_rate);
				} else {
					target_pitch_smooth = raw_pitch;
				}
				
				// 平滑处理Roll
				float roll_diff = raw_roll - target_roll_smooth;
				if (fabs(roll_diff) > max_roll_rate) {
					target_roll_smooth += (roll_diff > 0 ? max_roll_rate : -max_roll_rate);
				} else {
					target_roll_smooth = raw_roll;
				}
				
				FlyControl_pitch = target_pitch_smooth;
				FlyControl_roll = target_roll_smooth;
			}
			else
			{
				// 平滑处理Pitch
				float pitch_diff = controlVal.pitch - target_pitch_smooth;
				if (fabs(pitch_diff) > max_pitch_rate) {
					target_pitch_smooth += (pitch_diff > 0 ? max_pitch_rate : -max_pitch_rate);
				} else {
					target_pitch_smooth = controlVal.pitch;
				}
				
				// 平滑处理Roll
				float roll_diff = controlVal.roll - target_roll_smooth;
				if (fabs(roll_diff) > max_roll_rate) {
					target_roll_smooth += (roll_diff > 0 ? max_roll_rate : -max_roll_rate);
				} else {
					target_roll_smooth = controlVal.roll;
				}
				
				FlyControl_pitch = target_pitch_smooth;
				FlyControl_roll = target_roll_smooth;
			}
			
			//Z�������Ϊ�ۼ�ֵ
			FlyControl_yaw -= controlVal.gyroz;
			
			//����Ƿ������߶ȿ���(�͵���ʱ���������߲���)
			if( (uxBits & LowPower_Event) && controlVal.height > 0 )
			{
				/* ����͵���ģʽ��ֹ���� */
			}
			else
			{
				if( fabs(controlVal.height)!=0 )
					StartFly_SmoothHeightFlag=0;//���ƽ�����Ʋ���,���û����������ֹͣ
				
				//�߶ȿ���ʹ������ģʽ��������ģʽ�붨��ģʽ
				if( uxBits & UNUSE_HeightMode_Event )
					FlyControl_unuseHeight += (controlVal.height/6.0f);
				else
				{	
					FlyControl_height += controlVal.height;
					if( FlyControl_height > HeightMode_MAX )
						xTimerStart(priv_OperateFullTimer,0);//�������޷���ʾ
				}
				
				//�߶��½�һ������,�رշ���
				if( FlyControl_unuseHeight<=UNHeightMode_Min||FlyControl_height<=HeightMode_Min)
					xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
			}

		}
		else
		{
			/* ����״̬ˢ�� */
			refresh_cmdstate++;
			if( refresh_cmdstate >= TaskFreq/4 )
			{
				FlyControl_pitch = 0; 
				FlyControl_roll = 0;
				FlyControl_gyroz = 0;

				refresh_cmdstate = (TaskFreq/4) + 1;
				controlCmdNumber = IDLECmd;
			}	
		}
		/* ��ȡ����ָ�� END */
		
		/* �͵�������봦�� */
		static uint32_t lowVOLcount = 0;
		if( (uxBits & StartFly_Event) && g_robotVOL < 9.5f )
		{
			lowVOLcount++;
			if( 2*TaskFreq == lowVOLcount ) //����2���������10V
			{   //�͵������
				xEventGroupSetBits(g_xEventFlyAction,LowPower_Event); 
			}
		}
		else lowVOLcount = 0;
		
		//�͵�������
		if( (uxBits & LowPower_Event) && (uxBits & StartFly_Event) )
		{
			static uint16_t tipscount = 0;
			if( ++tipscount >= TaskFreq ) tipscount=0,xTimerStart(priv_lowpowerTimer,0);
			
			StartFly_SmoothHeightFlag=0;
			
			//�Զ����ͷ��и߶�
			if( uxBits & UNUSE_HeightMode_Event )
				FlyControl_unuseHeight-= 0.0005f;
			else
				FlyControl_height -= 0.001f;
			
			//�߶��½�һ������,�رշ���
			if( FlyControl_unuseHeight<=UNHeightMode_Min||FlyControl_height<=HeightMode_Min)
				xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
				
		}
		else if( !(uxBits & StartFly_Event) && g_robotVOL > ROBOT_VOL_LIMIT ) 
			xEventGroupClearBits(g_xEventFlyAction,LowPower_Event); 
		/* �͵�������봦�� END */
		
		//����ʱ�رո��桢���Ϲ���
		if( use_distance < 0.5f && controlVal.height<0 )
		{
			xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
			xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
		}
		
		/* �߶��޷����� */
		FlyControl_height = target_limit_float(FlyControl_height,HeightMode_Min,HeightMode_MAX);
		FlyControl_unuseHeight = target_limit_float(FlyControl_unuseHeight,UNHeightMode_Min,UNHeightMode_MAX);
		
		/* ƽ����ƺ������� */
		if( (uxBits & StartFly_Event)&&(uxBits & IMU_CalibZeroDone_Event ) )
		{
			if( 1==StarFly_UpdateFlag )
			{
				StarFly_UpdateFlag=0;
				
				/* ����λ�����궨 */
				zero_point->attitude->yaw += AttitudeVal.yaw;
				imu->UpdateZeroPoint_attitude(zero_point->attitude);
				
				//����λ��
				targetPosX = posX;
				targetPosY = posY;
				
				/* ������λ��,�����п�������λ */
				FlyControl_pitch = 0; FlyControl_roll = 0; FlyControl_gyroz = 0;
				FlyControl_yaw = AttitudeVal.yaw;
				FlyControl_unuseHeight = 0.0f;
				
				//PID����������ۼ�ֵ
				PID_Reset(&RollPID);
				PID_Reset(&RollRatePID);
				PID_Reset(&PitchPID);
				PID_Reset(&PitchRatePID);
				PID_Reset(&YawPID);
				PID_Reset(&YawRatePID);
				PID_Reset(&HeightPID);
				PID_Reset(&HeightSpeedPID);
				//�����������������
				
				//�������/�����״̬
				xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
				xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
				continue;
			}
			
			//���ʱִ�и߶�ƽ������
			if( 1 == StartFly_SmoothHeightFlag )
			{
				if( FlyControl_height < DefalutHeight ) FlyControl_height += 0.0025f;
				else StartFly_SmoothHeightFlag = 0,startPos=1;
			}
			
			//ƽ��״̬��΢������(���ۼ�,AttitudeValʵʱ����)
			AttitudeVal.pitch += userset_pitch;
			AttitudeVal.roll += userset_roll;
			
			/* �����㷨����... */
			
			///////////////////// �������ݴ��� ///////////////////////////////////
			static float x_dot_Prev = 0, y_dot_Prev = 0; // ��һ�ε��ٶ�ֵ
			
			// ������������
			float g = 9.8f;
			float g_x = -g * sin(AttitudeVal.pitch);
			float g_y =  g * sin(AttitudeVal.roll) * cos(AttitudeVal.pitch);
			float g_z =  g * cos(AttitudeVal.roll) * cos(AttitudeVal.pitch);

			// �����˶����ٶ�(��������ϵ)
			float a_x = axis_9Val.accel.x - g_x;
			float a_y = axis_9Val.accel.y - g_y;
			float a_z = axis_9Val.accel.z - g_z;

			// ����任(��������ϵ --> ��������ϵ)
			float a_motion_x = a_x * cos(AttitudeVal.pitch) + 
					   a_y * sin(AttitudeVal.roll) * sin(AttitudeVal.pitch) + 
							 a_z * cos(AttitudeVal.roll) * sin(AttitudeVal.pitch);
			float a_motion_y = a_y * cos(AttitudeVal.roll) - 
						   a_z * sin(AttitudeVal.roll);
			
			if( fabs(a_motion_x)<0.15f ) a_motion_x=0;
			if( fabs(a_motion_y)<0.15f ) a_motion_y=0;
			
			// �����˲�
			x_dot = 0.95f * (x_dot + a_motion_x * 0.005f) + 0.05f * speedX; // ��λ��m/s
			y_dot = 0.95f * (y_dot + a_motion_y * 0.005f) + 0.05f * speedY; // ��λ��m/s

			// ʹ�����λ��ַ�����λ��
			posX += (x_dot + x_dot_Prev) * 0.5f * 0.005f; // ��λ��m
			posY += (y_dot + y_dot_Prev) * 0.5f * 0.005f; // ��λ��m

			// �����޷�
			posX = fmaxf(fminf(posX, 5.0f), -5.0f);
			posY = fmaxf(fminf(posY, 5.0f), -5.0f);
			
			x_dot_Prev = x_dot;
			y_dot_Prev = y_dot;
			///////////////////// �������ݴ��� END ///////////////////////////////////
			///// �״�����ϰ��﷽�� /////
			static float lastErr;
			static float lastYErr;
			if( (LidarFollowRegion.avg_distance <= 800)&& (uxBits&lidar_follow_mode) )
			{
				follow_targetpos_lc307(255);//��������ʾ������
				float err = 0 - LidarFollowRegion.center_angle;
				FlyControl_yaw += ((0.004f*err + 0.004f*(err-lastErr))/57.3f);
				lastErr = err;

				//��ת������ɺ�,����ǰ�������pid����
				if( fabs(err)< 8 )
				{
					float Yerr = 370 - LidarFollowRegion.avg_distance;
					targetPosY -= ((0.002f*Yerr + 0.002f*(Yerr-lastYErr))/1000.0f);
					lastYErr = Yerr;
				}
			}
			///////////// END ////////////////
			
			//λ��PD����
			const float limitPos = 0.35f;
			float controlX = 0.5f * (targetPosX - posX) - 0.7f * x_dot;//ǰ��
			float controlY = 0.5f * (targetPosY - posY) - 0.7f * y_dot;//����
			
			if( controlX > limitPos ) controlX = limitPos;
			if( controlX <-limitPos ) controlX = -limitPos;
			if( controlY > limitPos ) controlY = limitPos;
			if( controlY <-limitPos ) controlY = -limitPos;
			
			//���ڿ������������豸�����ڻ�λ����δ���������Թ�����������Ϣ
			if( FlyControl_pitch!=0 || FlyControl_roll!=0 || startPos==0 || g_lost_pos_dev == 1 )
			{
				posX=0;posY=0;
				speedX=0;speedY=0;
				x_dot=0;y_dot=0;
				x_dot_Prev=0;y_dot_Prev=0;
				controlX=0;controlY=0;
				targetPosX=0;targetPosY=0;
			}
						
			//�Ƕ�---�⻷3�����
			PID_Update(&RollPID,-FlyControl_roll - controlY,AttitudeVal.roll);
			PID_Update(&PitchPID,-FlyControl_pitch + controlX,AttitudeVal.pitch);
			
			//ƫ���ǿ���,��Ҫ�ԽǶȽ��й�һ������
			float yaw_error = normalize_radian(FlyControl_yaw - AttitudeVal.yaw);
			PID_Update(&YawPID,yaw_error,0);
			
			PID_Update(&HeightPID,FlyControl_height,use_distance);
			
			//���ٶ�---�ڻ�3�����
			PID_Update(&RollRatePID,RollPID.output,axis_9Val.gyro.x);
			PID_Update(&PitchRatePID,PitchPID.output,axis_9Val.gyro.y);
			PID_Update(&YawRatePID,YawPID.output,axis_9Val.gyro.z);
			PID_Update(&HeightSpeedPID,HeightPID.output,height_dot);
			
			uint16_t base_throttle = weight_to_throttle((195.0f+0+FlyControl_unuseHeight*100.0f),use_distance); //��������ֵ
			motor.A.throttle = base_throttle - RollRatePID.output - PitchRatePID.output + YawRatePID.output + HeightSpeedPID.output;
			motor.B.throttle = base_throttle + RollRatePID.output - PitchRatePID.output - YawRatePID.output + HeightSpeedPID.output;
			motor.C.throttle = base_throttle + RollRatePID.output + PitchRatePID.output + YawRatePID.output + HeightSpeedPID.output;
			motor.D.throttle = base_throttle - RollRatePID.output + PitchRatePID.output - YawRatePID.output + HeightSpeedPID.output;
			
			//�Ƕȳ�����Χ����ͣ���
			const float stop_angle = angle_to_rad(30);
			if( fabs(AttitudeVal.pitch) >stop_angle || fabs(AttitudeVal.roll) > stop_angle )
			{
				xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);	
			}

		}
		else
		{
			/* �´�����ʱ,��Ҫ�궨��� */
			StarFly_UpdateFlag = 1;
			StartFly_SmoothHeightFlag = 1;
			FlyControl_height = 0;
			
			/* ͣת��� */
			StopVal_SelfRecovery(&MotorStopVal);//�����Իָ������dshotCMD����ֵ.
			memcpy(&motor,&MotorStopVal,sizeof(MOTOR_t));
		}
		/* ƽ����ƺ������� END */
		
		/* ��������ֵ����� */
		SetPwm(&motor); 

		/* ͳ���������е�����Ƶ�� */
		g_readonly_BalanceTaskFreq = debug->UpdateFreq(&debugPriv);
		
		/* ����������Ҫ������APP������ʾ,������޹� */
		extern APPShowType_t appshow;
		appshow.pitch = AttitudeVal.pitch;
		appshow.roll = AttitudeVal.roll;
		appshow.yaw = AttitudeVal.yaw;
		appshow.gyrox = axis_9Val.gyro.x;
		appshow.gyroy = axis_9Val.gyro.y;
		appshow.gyroz = axis_9Val.gyro.z;
		appshow.accelx = axis_9Val.accel.x;
		appshow.accely = axis_9Val.accel.y;
		appshow.accelz = axis_9Val.accel.z;
		appshow.balanceTaskFreq = g_readonly_BalanceTaskFreq;
		appshow.height = use_distance;
		//appshow.c_pitch = FlyControl_pitch;
		appshow.c_roll = FlyControl_roll;
		appshow.c_yaw = FlyControl_gyroz;
		appshow.c_height = FlyControl_height;
		appshow.zero_roll = userset_roll;
		appshow.zero_pitch = userset_pitch;
		appshow.posx = posX;
		appshow.posy = posY;
		appshow.targetX = targetPosX;
		appshow.targetY = targetPosY;
		appshow.speedx = x_dot;
		appshow.speedy = y_dot;

		/* ���������� */
		#if ENABLE_FLIGHT_DATA_OUTPUT
		if( ++dataOutputCounter >= 10 )  //200Hz�������10��������20Hz����,�������ز�������
		{
			dataOutputCounter = 0;
			extern QueueHandle_t g_xQueueFlightLog;
			FlightLogData_t logData = {
				.attitude = AttitudeVal,
				.imu = axis_9Val,
				.height = use_distance,
				.posX = posX,
				.posY = posY,
				.targetX = targetPosX,
				.targetY = targetPosY,
				.speedX = x_dot,
				.speedY = y_dot,
				.voltage = g_robotVOL,
				.motorA = motor.A.throttle,
				.motorB = motor.B.throttle,
				.motorC = motor.C.throttle,
				.motorD = motor.D.throttle,
				.cmd = lastControlVal
			};
			// 闈為樆濉炲啓鍏ラ槦鍒楋紝闃熷垪婊℃椂涓㈠純鏈鏁版嵁
			xQueueSend(g_xQueueFlightLog, &logData, 0);
		}
		#endif

		/* �ӳ�ָ��Ƶ�� */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}


float angle_to_rad(float angle)
{
	return angle*0.0174533f;
}

//static float rad_to_angle(float rad)
//{
//	return rad*57.29578f;
//}

//������ƺ���
static void SetPwm(MOTOR_t* m)
{
	//ִ�п���ʱ�����Ž����޷�
	if( 0 == m->A.Telemetry )
		m->A.throttle = target_limit_s16(m->A.throttle,Dshot_MIN,Dshot_MAX);
	
	if( 0 == m->B.Telemetry )
		m->B.throttle = target_limit_s16(m->B.throttle,Dshot_MIN,Dshot_MAX);
	
	if( 0 == m->C.Telemetry )
		m->C.throttle = target_limit_s16(m->C.throttle,Dshot_MIN,Dshot_MAX);
	
	if( 0 == m->D.Telemetry )
		m->D.throttle = target_limit_s16(m->D.throttle,Dshot_MIN,Dshot_MAX);
	
	//��������ָ��
	pMotorInterface_t motor = &UserDshotMotor;
	motor->set_target(m->A,m->D,m->C,m->B);

//�������������������Ӧ��ϵ
/*
	 ��ͷ���������
���C(˳)     ���B(��)
         |
         |
-----------------------	
         |
         |
���D(��)     ���A(˳)
*/    
}

//�޷�����
static int target_limit_s16(short insert,short low,short high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}


static float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

static void FollwTips_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(1000);
	tips->off();
}


//����������
static void OperateResponse_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(50);
	tips->off();
}

//�����Ѵﵽ���޷�����
static void OperateFull_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(50);
	tips->off();
	vTaskDelay(50);
	tips->on();
	vTaskDelay(50);
	tips->off();
	vTaskDelay(50);
	tips->on();
	vTaskDelay(50);
	tips->off();
}


//�궨��ɷ�������ʾ��ʱ��
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(200);
	tips->off();
	vTaskDelay(300);
	for(uint8_t i=0;i<30;i++)
	{
		tips -> toggle();
		vTaskDelay(25);
	}
	tips->off();
}

static void UNUSEHeightTips_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(200);
	tips->off();
	vTaskDelay(300);
	for(uint8_t i=0;i<5;i++)
	{
		tips -> toggle();
		vTaskDelay(100);
	}
	tips->off();
}

static void LowPowerTips_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(500);
	tips->off();
}

//LED��ʾ��ʱ��
static void LedTipsTimer_Callback(TimerHandle_t xTimer)
{
	static uint8_t init = 0;
	pLedInterface_t led1 = &UserLed1;
	pLedInterface_t led2 = &UserLed2;
	if( 0 == init )
	{
		init = 1; led1->on(); led2->off();
	}
	
	//�궨���ʱ������˸,�궨��󵥿�LED��˸
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( 0 == (uxBits & IMU_CalibZeroDone_Event) ) led1->toggle();
	led2->toggle();
}

//���߶������Ƿ��ȶ�
static uint8_t check_HeightStable(float height)
{
	static float last_height = 0 ;
	
	static uint8_t StableThreshold = 0; //�ȶ���ֵ
	
	//����Ư�Ʋ��ȶ�,���߳�ʱ��Ϊ0,�����������쳣
	if( fabs( last_height - height ) > 0.5f || ( last_height == 0 && height == 0 ) )
	{
		if( StableThreshold<255 ) StableThreshold++;
	}
	
	last_height = height;
	
	return StableThreshold;
}

//�ȴ�imu�����ȶ�
static IMU_ZEROPONIT_t* WaitImuStable(uint16_t freq,IMU_DATA_t NowImu,ATTITUDE_DATA_t NowAttitude)
{
	static uint16_t timecore = 0;
	
	//�������궨
	static IMU_DATA_t LastImuData = { 0 };
	static ATTITUDE_DATA_t LastAttitudeData = { 0 };
	
	//������㱣��
	static IMU_DATA_t ZeroPoint = { 0 };
	static ATTITUDE_DATA_t AttitudeZeroPoint = { 0 };
	static IMU_ZEROPONIT_t res_p = {NULL,NULL};//����ʵ��
	
	//�����ȶ��Ĵ���
	static uint8_t stablecount = 0;
	
	//���궨�Ĵ���
	const uint8_t calibratetimes = 5;
	
	static uint8_t step = 0;
	
	uint8_t state = 0;
	timecore++;
	if( timecore >= freq/5 ) //200ms���1��
	{
		timecore = 0;
		
		if( 0 == step ) //����0,�궨6�����ݵ����(gyro,accel)
		{
			if( fabs(NowImu.gyro.x - LastImuData.gyro.x) < 0.01f ) state++;
			if( fabs(NowImu.gyro.y - LastImuData.gyro.y) < 0.01f ) state++;
			if( fabs(NowImu.gyro.z - LastImuData.gyro.z) < 0.01f ) state++;
			if( fabs(NowImu.accel.x - LastImuData.accel.x) < 0.06f ) state++;
			if( fabs(NowImu.accel.y - LastImuData.accel.y) < 0.06f ) state++;
			if( fabs(NowImu.accel.z - LastImuData.accel.z) < 0.06f ) state++;
			
			if( state==6 )  
			{
				stablecount++;
				ZeroPoint.gyro.x+=NowImu.gyro.x;
				ZeroPoint.gyro.y+=NowImu.gyro.y;
				ZeroPoint.gyro.z+=NowImu.gyro.z;
				ZeroPoint.accel.x+=NowImu.accel.x;
				ZeroPoint.accel.y+=NowImu.accel.y;
				ZeroPoint.accel.z+=NowImu.accel.z;
				
				//�����������ȶ�,����Ϊ��ǰ�����ȶ�
				if( stablecount == calibratetimes ) 
				{
					stablecount = 0;//���6�����ݱ궨
					step = 1;       //���벽��1
					
					//ȡ�����ȶ��ڼ������ƽ��ֵ
					ZeroPoint.gyro.x/=calibratetimes;
					ZeroPoint.gyro.y/=calibratetimes;
					ZeroPoint.gyro.z/=calibratetimes;
					ZeroPoint.accel.x/=calibratetimes;
					ZeroPoint.accel.y/=calibratetimes;
					ZeroPoint.accel.z/=calibratetimes;
					
					ZeroPoint.accel.z-=9.8f;
					res_p.axis = &ZeroPoint;
					
					//����궨�����
//					printf("======== step1 ==========\r\n");
//					printf("gx:%.3f\r\n",ZeroPoint.gyro.x);
//					printf("gy:%.3f\r\n",ZeroPoint.gyro.y);
//					printf("gz:%.3f\r\n",ZeroPoint.gyro.z);
//					printf("ax:%.3f\r\n",ZeroPoint.accel.x);
//					printf("ay:%.3f\r\n",ZeroPoint.accel.y);
//					printf("az:%.3f\r\n",ZeroPoint.accel.z);
//					printf("======== step1 ==========\r\n");	
					
					return &res_p;//����һ��,����������δ����ʱ����������һ������
				}
		
			}
			else  //���������ȶ�����,����ۼ���ֵ���µȴ��궨
			{   
				stablecount=0;
				memset(&ZeroPoint,0,sizeof(IMU_DATA_t));
			}			
			
			//������һ������
			memcpy(&LastImuData,&NowImu,sizeof(IMU_DATA_t));
		}
		
		else if( 1 == step ) //����1,�궨ŷ����
		{
			if( fabs( NowAttitude.roll - LastAttitudeData.roll ) < 0.002f )   state++;
			if( fabs( NowAttitude.pitch - LastAttitudeData.pitch ) < 0.002f ) state++;
			if( fabs( NowAttitude.yaw - LastAttitudeData.yaw ) < 0.002f )     state++;
			
			if( state==3 )  
			{
				stablecount++;
				AttitudeZeroPoint.pitch+=NowAttitude.pitch;
				AttitudeZeroPoint.yaw+=NowAttitude.yaw;
				AttitudeZeroPoint.roll+=NowAttitude.roll;
				
				//�����������ȶ�,����Ϊ��ǰ�����ȶ�
				if( stablecount == calibratetimes ) 
				{
					stablecount = 0;//���6�����ݱ궨
					step = 2;       //���벽��1
					
					//ȡ�����ȶ��ڼ������ƽ��ֵ
					AttitudeZeroPoint.pitch/=calibratetimes;
					AttitudeZeroPoint.yaw/=calibratetimes;
					AttitudeZeroPoint.roll/=calibratetimes;
					
					res_p.attitude = &AttitudeZeroPoint;
					
					//����궨�����
//					printf("======== step2 ==========\r\n");
//					printf("pitch:%.3f\r\n",AttitudeZeroPoint.pitch);
//					printf("  yaw:%.3f\r\n",AttitudeZeroPoint.yaw);
//					printf(" roll:%.3f\r\n",AttitudeZeroPoint.roll);
//					printf("======== step2 ==========\r\n");	
				}
			}
			else  //���������ȶ�����,����ۼ���ֵ���µȴ��궨
			{
				stablecount=0;
				memset(&AttitudeZeroPoint,0,sizeof(ATTITUDE_DATA_t));
			}		
			memcpy(&LastAttitudeData,&NowAttitude,sizeof(ATTITUDE_DATA_t)); //������һ�ε�����
		} /* if( 1 == step ) */
	} /* if( timecore >= freq/5 ) */
	
	return &res_p;
}

static void StopVal_SelfRecovery(MOTOR_t* m)
{
	const uint8_t times = 30;
	static uint8_t Flag_recoveryA = 0,Flag_recoveryB = 0,Flag_recoveryC = 0,Flag_recoveryD = 0;
	
	//����dshotCMD����ʱ,������������ times �κ��Զ��ָ�����ָ��
	if( 1 == m->A.Telemetry )
	{
		Flag_recoveryA++;
		if( Flag_recoveryA > times ) 
		{
			Flag_recoveryA = 0;
			m->A.Telemetry = 0;
			m->A.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//�ָ���ʾ��
		}
	}
	
	if( 1 == m->B.Telemetry )
	{
		Flag_recoveryB++;
		if( Flag_recoveryB > times ) 
		{
			Flag_recoveryB = 0;
			m->B.Telemetry = 0;
			m->B.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//�ָ���ʾ��
		}
	}
	
	if( 1 == m->C.Telemetry )
	{
		Flag_recoveryC++;
		if( Flag_recoveryC > times ) 
		{
			Flag_recoveryC = 0;
			m->C.Telemetry = 0;
			m->C.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//�ָ���ʾ��
		}
	}
	
	if( 1 == m->D.Telemetry )
	{
		Flag_recoveryD++;
		if( Flag_recoveryD > times ) 
		{
			Flag_recoveryD = 0;
			m->D.Telemetry = 0;
			m->D.throttle = Dshot_MIN;
			xTimerStart(priv_OperateFullTimer,0);//�ָ���ʾ��
		}
	}
	
}

//����תΪ����ֵ,��ڲ�����λ��g
static uint16_t weight_to_throttle(float weight,float height)
{
//	weight = weight + height*14.0f;
	
    return 1537.96f + 8.70f*weight - 225.2f*g_robotVOL - 0.0088f*weight*weight - 0.24f*weight*g_robotVOL + 9.01f*g_robotVOL*g_robotVOL;
}

/**************************************************************************
�������ܣ�ͨ��USART1���������������,CSV��ʽ,����������
��    �ߣ�Claude Code
**************************************************************************/

		xSemaphoreGive(HandleMutex_printf);
	}
}
#endif

/**************************************************************************
�������ܣ���������ƶ�����д�����ָ��,���Զ������������ȼ��������Ƿ�Ϸ�������
��ڲ�������Ҫд��Ŀ���ָ���ַ,�������еĻ���(0���񻷾�,1 ISR����),����Ƿ��и������ȼ�������(��ISR����ʹ��)
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void WriteFlyControlQueue(FlyControlType_t val,uint8_t writeEnv,BaseType_t* HigherPriorityTask)
{
	extern QueueHandle_t g_xQueueFlyControl;
	
	//���ָ���Ƿ���Դ�ڸ������ȼ�
	if( val.source > controlCmdNumber )  controlCmdNumber = val.source;
	
	//���ȼ�ƥ��,д����ƶ���
	if( val.source == controlCmdNumber  )
	{
		if( 1 == writeEnv ) //ISR����
		{
			xQueueOverwriteFromISR(g_xQueueFlyControl,&val,HigherPriorityTask);
		}
		else //���񻷾�
		{
			xQueueOverwrite(g_xQueueFlyControl,&val);
		}
	}
}

void BeepTips(void)
{
	xTimerStart(priv_OperateResponseTimer,0);
}

//�����¼�(���û���Ҫ��������)
void StartFlyAction(void)
{
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	//����Ѿ�����,�û��ٴβ���ʱ,��ر�
	if( uxBits&StartFly_Event )
	{
		xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
	}
	
	//����
	else if( g_robotVOL > ROBOT_VOL_LIMIT && (uxBits & IMU_CalibZeroDone_Event) && !(uxBits & LowPower_Event ) )
	{
		xEventGroupSetBits(g_xEventFlyAction,StartFly_Event);
		xTimerStart(priv_OperateResponseTimer,0);
		if( uxBits&TestMotorMode_Event ) xEventGroupClearBits(g_xEventFlyAction,TestMotorMode_Event);
	}

	else
		xTimerStart(priv_OperateFullTimer,0);
}

//ֹͣ�¼�(���û���Ҫ��������)
void StopFlyAction(void)
{
	xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
	xTimerStart(priv_OperateResponseTimer,0);
}

//������ģʽ������˳�(���û���Ҫ��������)
void FlyAction_EnterUNUSEHeightMode(void)
{
	static uint8_t mode = 0;
	
	mode = !mode;
	if( mode ) xEventGroupSetBits(g_xEventFlyAction,UNUSE_HeightMode_Event),xTimerStart(priv_UNUSEHeightTimer,0);
	else xEventGroupClearBits(g_xEventFlyAction,UNUSE_HeightMode_Event),xTimerStart(priv_OperateResponseTimer,0);
}

//�������ǵ������е���
void FlyAction_AdjustDiffAngle(uint8_t changeNum)
{
	const float step = 0.2f;
	        if( 1==changeNum ) userset_pitch += angle_to_rad(step);
	else if( 2 == changeNum ) userset_pitch -= angle_to_rad(step);
	else if( 3 == changeNum ) userset_roll += angle_to_rad(step);
	else if( 4 == changeNum ) userset_roll -= angle_to_rad(step);
	
	xTimerStart(priv_OperateResponseTimer,0);
}

void FlyAction_ChangeLidarAvoidMode(uint8_t id)
{
	xTimerStart(priv_OperateResponseTimer,0);
	if( id==1 ) 
	{	//��������
		xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
		xEventGroupSetBits(g_xEventFlyAction,lidar_avoid_mode);
	}
	else if( id==2 )
	{	//��������
		xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
		xEventGroupSetBits(g_xEventFlyAction,lidar_follow_mode);
	}
	else 
	{
		xEventGroupClearBits(g_xEventFlyAction,lidar_follow_mode);
		xEventGroupClearBits(g_xEventFlyAction,lidar_avoid_mode);
	}
}

//�����û��趨��΢������
void FlyAction_SaveDiffAngleParam(void)
{
	extern uint8_t User_Flash_SaveParam(uint32_t* data,uint16_t datalen);
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	if( !(uxBits & StartFly_Event) ) //���ڲ�����ʱ�����������
	{
		int32_t saveparam[2] = { 0 };
		saveparam[0] = *((int32_t*)&userset_pitch);
		saveparam[1] = *((int32_t*)&userset_roll);
		
		taskENTER_CRITICAL(); //�����ٽ���,��������������ж�
		uint8_t res = User_Flash_SaveParam((uint32_t*)saveparam,2);
		taskEXIT_CRITICAL(); //д�����,�˳��ٽ�
		if( 1 == res ) xTimerStart(priv_OperateFullTimer,0);
	}
}

//��ͷ��ͷģʽ�л�
void FlyAction_HeadLessModeChange(void)
{
	static uint8_t flag = 1;
	
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	if( !(uxBits & StartFly_Event) ) //ֻ֧�����ǰ����
	{
		flag = !flag;
		
		//�ر���ͷģʽ
		if( flag )
		{
			xEventGroupClearBits(g_xEventFlyAction,FlyMode_HeadLessMode_Event);
			xTimerStart(priv_OperateResponseTimer,0);
		}
		else
		{
			xEventGroupSetBits(g_xEventFlyAction,FlyMode_HeadLessMode_Event);
			xTimerStart(priv_UNUSEHeightTimer,0);
		}
	}

		
}

//��λ,����Ҫ����������
void ResetSystem(uint8_t isFromISR)
{
	EventBits_t uxBits = 0;
	
	if( isFromISR ) uxBits = xEventGroupGetBitsFromISR(g_xEventFlyAction);
	else            uxBits = xEventGroupGetBits(g_xEventFlyAction);
	
	if( !(uxBits&StartFly_Event) )
	{
		NVIC_SystemReset();
	}
	else
	{
		if( isFromISR ) xTimerStartFromISR(priv_OperateFullTimer,0);
		else xTimerStart(priv_OperateFullTimer,0);
	}

}

//����������ģʽ
void FlyAction_TestMotorMode(void)
{
	static uint8_t flag = 0;
	xEventGroupClearBits(g_xEventFlyAction,StartFly_Event);
	xTimerStart(priv_OperateResponseTimer,0);
	
	flag=!flag;
	
	if(flag)
		xEventGroupSetBits(g_xEventFlyAction,TestMotorMode_Event);
	else
		xEventGroupClearBits(g_xEventFlyAction,TestMotorMode_Event);
}

const uint16_t testmotorVal = 160;

#if 1
void TestMotorMode_TestA(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.A.Telemetry = 1;
			MotorStopVal.A.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestB(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.B.Telemetry = 1;
			MotorStopVal.B.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestC(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.C.Telemetry = 1;
			MotorStopVal.C.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestD(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle = testmotorVal;
			}
			else
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle = Dshot_MIN;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.D.Telemetry = 1;
			MotorStopVal.D.throttle = DSHOT_CMD_SAVE_SETTINGS;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

#else //����������Է���
void TestMotorMode_TestA(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle += 50;
			}
			else
			{
				MotorStopVal.A.Telemetry = 0;
				MotorStopVal.A.throttle += 50;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.A.Telemetry = 1;
				MotorStopVal.A.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.A.Telemetry = 0;
			MotorStopVal.A.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestB(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle += 50;
			}
			else
			{
				MotorStopVal.B.Telemetry = 0;
				MotorStopVal.B.throttle += 50;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.B.Telemetry = 1;
				MotorStopVal.B.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.B.Telemetry = 0;
			MotorStopVal.B.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestC(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle += 50;
			}
			else
			{
				MotorStopVal.C.Telemetry = 0;
				MotorStopVal.C.throttle += 50;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.C.Telemetry = 1;
				MotorStopVal.C.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.C.Telemetry = 0;
			MotorStopVal.C.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}

void TestMotorMode_TestD(uint8_t operateNum)
{
	if( operateNum>2 ) return; //������δ�����ı�ſ���
	
	static uint8_t flag1 = 0,flag2 = 0;
	EventBits_t uxBits = xEventGroupGetBits(g_xEventFlyAction);
	if( uxBits&TestMotorMode_Event )
	{
		xTimerStart(priv_OperateResponseTimer,0);
		
		if( 0==operateNum ) //�������,0�Ų��Ե��
		{
			flag1 = !flag1;
			if(flag1)
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle += 50;
			}
			else
			{
				MotorStopVal.D.Telemetry = 0;
				MotorStopVal.D.throttle += 50;
			}
		}
		else if( 1==operateNum ) //�������,1���޸ĵ����ת��
		{
			flag2 = !flag2;
			if( flag2 == 1 )
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_1;
			}
			else
			{
				MotorStopVal.D.Telemetry = 1;
				MotorStopVal.D.throttle = DSHOT_CMD_SPIN_DIRECTION_2;
			}

		}
		else if( 2==operateNum ) //�������,2�ű�������ת��
		{
			MotorStopVal.D.Telemetry = 0;
			MotorStopVal.D.throttle = Dshot_MIN;
		}

	}
	else flag1 = 0 , flag2 = 0;
}
#endif