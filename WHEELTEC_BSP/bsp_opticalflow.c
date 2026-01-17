#include "bsp_opticalflow.h"

#include "usart.h"

//使用哪个串口作为光流接口
static UART_HandleTypeDef* opticalflow_serial = &huart4;

//光流数据
static PositionType_t postiton = { 0 };
static PositionType_t postitondot = { 0 };

//存放光流原始数据的长度和数据
#define bufferLen 10
static uint8_t serialbuffer[bufferLen];

//启动数据接收
static void start_dma_recv(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(opticalflow_serial,serialbuffer,bufferLen);
}

//返回处理的数据
static PositionType_t returnPos(void)
{
	return postiton;
}

static PositionType_t returnPosDot(void)
{
	return postitondot;
}

//光流数据处理（环境为ISR,可使用写队列方式将处理环境放置于任务中）
static void Handle_opticalflow_buffer(uint16_t size)
{
	//帧头帧尾、数据大小正确才执行解析
	if( size==9 && serialbuffer[0]==0xFE && serialbuffer[8]==0xAA )
	{
		//sum校验
		if( ( (serialbuffer[2]+serialbuffer[3]+serialbuffer[4]+serialbuffer[5])&0xff ) == serialbuffer[6] )
		{
			//图像质量,仅在良好时使用数据
			if( serialbuffer[7] > 30 )
			{
				postitondot.x = -((short)(serialbuffer[3]<<8 | serialbuffer[2]));
				postitondot.y = (short)(serialbuffer[5]<<8 | serialbuffer[4]);
				
				postiton.x += postitondot.x;
				postiton.y += postitondot.y;
			}
		}
	}
}

//清空累计值,标记当前位置为0点坐标
static void ClearPostition(void)
{
	postiton.x = 0;
	postiton.y = 0;
}


OpticalFlowInterface_t  UserOpticalFlow = {
	.StartRecvData = start_dma_recv,
	.getpos = returnPos,
	.getpos_dot = returnPosDot,
	.HandleRecvData = Handle_opticalflow_buffer,
	.SetZeroPoint = ClearPostition
};

