#include "pid.h"
#include "math.h"


//清除积累量,用于停止控制时使用
void PID_Reset(PIDControllerType_t* pid)
{
	pid->intergral = 0;
	pid->output = 0;
	pid->prev_error = 0;
	pid->derivative = 0;
}

//PID更新函数
void PID_Update(PIDControllerType_t* pid,float target,float current)
{
	
	float error = target - current; //本次误差
	
//	//积分因子
//	float integral_factor = (fabs(error) < pid->IntegralThreshold) ? 1.0f : (pid->IntegralThreshold / fabs(error));
//	pid->intergral += integral_factor  * error;
	
	if( fabs(error) < pid->IntegralThreshold ) pid->intergral += error;
	
	//微分项,采用一阶低通滤波
	float Tmpderivative = (error - pid->prev_error);
	pid->derivative = pid->alpha*Tmpderivative + (1.0f-pid->alpha)*pid->derivative;
	pid->prev_error = error;
	
	//输出限幅
	if( pid->output > pid->LimitOutputMax ) 
	{
		pid->output = pid->LimitOutputMax;
//		if( error>0 ) pid->intergral *= 0.9f; // 轻微减少积分项，防止累积过多
	}
	if( pid->output < pid->LimitOutputMin ) 
	{
		pid->output = pid->LimitOutputMin;
//		if( error<0 ) pid->intergral *= 0.9f; // 轻微减少积分项，防止累积过多
	}
	
	//积分限幅
	if( pid->intergral>pid->LimitIntegralMax ) pid->intergral = pid->LimitIntegralMax;
	if( pid->intergral<pid->LimitIntegralMin ) pid->intergral = pid->LimitIntegralMin;
	
	//pid输出
	pid->output = pid->kp * error + pid->ki * pid->intergral + pid->kd*pid->derivative;
	
}

//角速度环roll
PIDControllerType_t RollRatePID = {
	.kp = 85.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.LimitIntegralMax = 3.5f,   //积分限幅,200°/s
	.LimitIntegralMin = -3.5f, 
	.LimitOutputMax = 500.0f,   //角速度环输出为油门值
	.LimitOutputMin = -500.0f,
	.IntegralThreshold = 0.35f , // 20°/s开始积分
	.alpha = 0.8f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角速度环pitch
PIDControllerType_t PitchRatePID = {
	.kp = 85.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.LimitIntegralMax = 3.5f,   //积分限幅,200°/s
	.LimitIntegralMin = -3.5f, 
	.LimitOutputMax = 500.0f,   //角速度环输出为油门值
	.LimitOutputMin = -500.0f,
	.IntegralThreshold = 0.35f , // 20°/s开始积分
	.alpha = 0.8f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角度环roll
PIDControllerType_t RollPID = {
	.kp = 4.2f,
	.ki = 0.0f,
	.kd = 0.0f,
	.LimitIntegralMax = 0.52f,
	.LimitIntegralMin = -0.52f,    //积分限制,30°
	.LimitOutputMax = 1.7f,      //角速度输出,最大为 100°/s
	.LimitOutputMin = -1.7f,
	.IntegralThreshold = 0.2f ,  //10°积分生效生效
	.alpha = 0.9f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角度环pitch
PIDControllerType_t PitchPID = {
	.kp = 4.2f,
	.ki = 0.0f,
	.kd = 0.0f,
	.LimitIntegralMax = 0.52f,
	.LimitIntegralMin = -0.52f,    //积分限制,30°
	.LimitOutputMax = 1.7f,      //角速度输出,最大为 100°/s
	.LimitOutputMin = -1.7f,
	.IntegralThreshold = 0.2f ,  //10°积分生效生效
	.alpha = 0.9f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角速度环yaw
PIDControllerType_t YawRatePID = {
	.kp = 200.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.LimitIntegralMax = 1.04f,   //积分限幅,60°/s
	.LimitIntegralMin = -1.04f, 
	.LimitOutputMax = 500,
	.LimitOutputMin = -500,
	.IntegralThreshold = 0.17f , // 10°/s开始积分
	.alpha = 0.8,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角度环yaw
PIDControllerType_t YawPID = {
	.kp = 6.0f,
	.ki = 0.0f,
	.kd = 0.0f,
	.LimitIntegralMax = 0.52f,
	.LimitIntegralMin = -0.52f,    //积分限制,30°
	.LimitOutputMax = 3.7f,      //角速度输出,最大为 100°/s
	.LimitOutputMin = -3.7f,
	.IntegralThreshold = 0.2f ,  //10°积分生效生效
	.alpha = 1.0f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//PIDControllerType_t HeightSpeedPID = {
//	.kp = 60.0f,
//	.ki = 0.02f,
//	.kd = 80.0f,
//	.LimitIntegralMax = 100.0f,
//	.LimitIntegralMin = -100.0f,
//	.LimitOutputMax = 500,
//	.LimitOutputMin = -500,
//	.IntegralThreshold = 20 ,
//	.alpha = 0.4,
//	.prev_error = 0,
//	.intergral = 0,
//	.derivative = 0,
//	.output = 0
//};


//PIDControllerType_t HeightPID = {
//	.kp = 5.5f,
//	.ki = 0.0f,
//	.kd = 0.5f,
//	.LimitIntegralMax = 10,
//	.LimitIntegralMin = -10,
//	.LimitOutputMax = 1.0f,
//	.LimitOutputMin = -1.0f,
//	.IntegralThreshold = 10 ,
//	.alpha = 1.0f,
//	.prev_error = 0,
//	.intergral = 0,
//	.derivative = 0,
//	.output = 0
//};

PIDControllerType_t HeightSpeedPID = {
	.kp = 60.0f,
	.ki = 0.02f,
	.kd = 80.0f,
	.LimitIntegralMax = 100.0f,
	.LimitIntegralMin = -100.0f,
	.LimitOutputMax = 500,
	.LimitOutputMin = -500,
	.IntegralThreshold = 20 ,
	.alpha = 0.4,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};


PIDControllerType_t HeightPID = {
	.kp = 3.5f,
	.ki = 0.0f,
	.kd = 0.5f,
	.LimitIntegralMax = 10,
	.LimitIntegralMin = -10,
	.LimitOutputMax = 1.0f,
	.LimitOutputMin = -1.0f,
	.IntegralThreshold = 10 ,
	.alpha = 1.0f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

