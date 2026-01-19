#include "flight_log_task.h"
#include "balance_task.h"

/* C Lib */
#include <stdio.h>
#include <string.h>

/* RTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

/* HAL */
#include "usart.h"

#if ENABLE_FLIGHT_DATA_OUTPUT

// 格式化并输出飞行日志数据
static void FormatAndOutputFlightLog(FlightLogData_t* logData)
{
	extern SemaphoreHandle_t HandleMutex_printf;  // 打印互斥锁

	// 选择串口，如果选择UART4输出
	#if FLIGHT_DATA_USE_BLUETOOTH
		extern UART_HandleTypeDef huart4;
		UART_HandleTypeDef* dataSerial = &huart4;
	#else
		extern UART_HandleTypeDef huart1;
		UART_HandleTypeDef* dataSerial = &huart1;
	#endif

	// 获取互斥锁，非阻塞
	if( xSemaphoreTake(HandleMutex_printf, 0) == pdTRUE )
	{
		static char output_buffer[512];
		int len = 0;

		// 获取命令源字符串
		const char* cmdSource[] = {"IDLE", "APP", "PS2", "AVOID"};
		const char* source = (logData->cmd.source < 4) ? cmdSource[logData->cmd.source] : "UNKN";

		// 优化输出格式，包含所有字段名称和单位
		len = sprintf(output_buffer,
			"[CMD] Src:%s R:%.3f P:%.3f Gz:%.3f H:%.3f | "
			"[ATTITUDE] Roll:%.3f Pitch:%.3f Yaw:%.3f | "
			"[GYRO] X:%.2f Y:%.2f Z:%.2f | "
			"[ACCEL] X:%.2f Y:%.2f Z:%.2f | "
			"[POSITION] H:%.3f X:%.3f Y:%.3f | "
			"[TARGET] X:%.3f Y:%.3f | "
			"[SPEED] X:%.3f Y:%.3f | "
			"[POWER] V:%.2f | "
			"[MOTOR] A:%d B:%d C:%d D:%d\r\n",
			// 控制指令
			source, logData->cmd.roll * 57.3f, logData->cmd.pitch * 57.3f, logData->cmd.gyroz * 57.3f, logData->cmd.height,
			// 姿态角度（度）
			logData->attitude.roll * 57.3f, logData->attitude.pitch * 57.3f, logData->attitude.yaw * 57.3f,
			// 三轴角速度（rad/s）
			logData->imu.gyro.x, logData->imu.gyro.y, logData->imu.gyro.z,
			// 三轴加速度（m/s²）
			logData->imu.accel.x, logData->imu.accel.y, logData->imu.accel.z,
			// 位置（米）
			logData->height, logData->posX, logData->posY,
			// 目标位置（米）
			logData->targetX, logData->targetY,
			// 速度（m/s）
			logData->speedX, logData->speedY,
			// 电压（V）
			logData->voltage,
			// 四个电机的油门值
			logData->motorA, logData->motorB, logData->motorC, logData->motorD
		);

		// 发送格式化数据
		#if FLIGHT_DATA_USE_BLUETOOTH
			// 蓝牙使用DMA发送，不能使用阻塞方式
			HAL_UART_Transmit(dataSerial, (uint8_t*)output_buffer, len, 100);
		#else
			// USART1使用printf的方式(已经配置)
			printf("%s", output_buffer);
		#endif

		xSemaphoreGive(HandleMutex_printf);
	}
}

// 飞行日志输出任务
void FlightLogTask(void* param)
{
	extern QueueHandle_t g_xQueueFlightLog;
	FlightLogData_t logData;
	
	// RTOS使用浮点
	portTASK_USES_FLOATING_POINT();
	
	while(1)
	{
		// 从队列读取数据，阻塞等待
		if( xQueueReceive(g_xQueueFlightLog, &logData, portMAX_DELAY) == pdTRUE )
		{
			// 格式化并输出数据
			FormatAndOutputFlightLog(&logData);
		}
	}
}

#endif /* ENABLE_FLIGHT_DATA_OUTPUT */

