/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "update.h"
#include "debug.h"
#include "string.h"
#include <stdlib.h>
#include "action_driver.h"
#include "MotionCard.h"
#include "update.h"
#include "action_driver.h"
#include "threeWheelMove.h"



//正常模式
#define NORMAL_MODE 1
//采集模式
#define SAMPLING_MODE 2


static int  mode = 1;

_Bool GyroscopeFlag = 0;



void Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	

	//CAN1初始化，用于电机控制
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9); 
	
	
	//陀螺仪接收串口
	USART3_Init(115200);
	
	//蓝牙串口
	GYRO_USART2_Init(115200);
	
	//蜂鸣器初始化
	MyBeefInit();
	
	//行程开关
	MySwitchInit();
	
	//定时器初始化 10ms
	TIM_Init(TIM2, 999, 839, 0, 0);
	
	//初始化内存池
	BufferZizeInit(500);
	
	BeefON();
	
  TIM_Delayms(TIM5, 1000);
	
	BeefOFF();
	
	while(GyroscopeFlag == 0)
	{
		//如果等待陀螺仪初始化期间发生按键触发情况，进行采样模式
		if(ReadSwitch() == 1)
		{
				BeefON();
				mode = SAMPLING_MODE;
		}
		else
		{
				BeefOFF();
		}
	}


	/*********************MOTOR DRIVE INTI**************************/

if(mode == NORMAL_MODE)
{
		ActionSetHeartbeatInterval(1, 100);
		ActionSetHeartbeatInterval(2, 100);
		ActionSetHeartbeatInterval(3, 100);
		ActionSetOperationalMode(1); // 发送命令前，需要控制驱动器 在可操作模式
		ActionSetOperationalMode(2); // 发送命令前，需要控制驱动器 在可操作模
		ActionSetOperationalMode(3); // 发送命令前，需要控制驱动器 在可操作模
		ActionConfigVelocity(1,200000,200000);
		ActionConfigVelocity(2,200000,200000);
		ActionConfigVelocity(3,200000,200000);
	
		BeefON();

		TIM_Delayms(TIM5, 1000);

		BeefOFF();

		TIM_Delayms(TIM5, 1000);
}

	/*********************TIMER INIT*******************************/
		BeefON();

		TIM_Delayms(TIM5, 1000);

		BeefOFF();

}



#define NUMM 8
Pose_t aaa1[NUMM] = 
{
{0,0,0},
{100,500,0},
{0,660,0},
{100,700,0},
{0,700,0},
{0,1000,60},
{0,1500,-60},
{0,4000,0}


};



Pose_t aaa2[NUMM] = 
{
{0,0,0},
{0,4000,0}
};

extern _Bool USARTSEND;
int main(void)
{

	Init();
//	mode = 4;
	while(1)
	{

		
	switch(mode)
		{
			
			case NORMAL_MODE:
				

					ReadFlashPointInformation();

					mode = 3;
				break;
			
			//采集模式
			case SAMPLING_MODE:
				
				PoseSampling();
			
				if(ReadSwitch() == 1)
				{
					static int timeCount = 0;
					
					timeCount++;
					
					//如果行程开关按下一定时间，则将所记录的信息写入flash
					if(timeCount > 2000)
					{						

						PoseSamplingDone();

						BeefON();
						while(1);
					}
				}

				break;
				
				
			case 3:
				
						PathFollowing(1);

						if(USARTSEND == 1)
						{

							USARTSEND = 0;
						}
				break;
			
			//测试模式
			case 4:
					InputPoints2RingBuffer(aaa1,NUMM);

					mode = 5;
				break;
			
			
			case 5:
						if(GetPosy() > 2000)
						{
							aaa1[0].direction = 0;
							aaa1[0].point = GetPosPresent().point;
							aaa1[1].point.x = -2000;
							aaa1[1].point.y = 4000;
							aaa1[1].direction = 40;
							ClearRingBuffer();
							mode = 6;
						}
						
						PathFollowing(1);
					
						if(USARTSEND == 1)
						{
							
							USART_OUT_XYAngle();

							USARTSEND = 0;
						}
				break;
						
			case 6:
						InputPoints2RingBuffer(aaa1,2);
						int qqq = GetCount();
//						USART_OUT(SENDUSART,(uint8_t *)"%d\r\n",qqq);	
//						for(int i = 1;i <= qqq;i++)
//						{
//							USART_OUT(SENDUSART,(uint8_t *)"%d\t %d\t %d\t %d\t %d\t %d\t %d\t\r\n",
//							(int)GetRingBufferPoint(i).x,(int)GetRingBufferPoint(i).y,(int)GetRingBufferPointAngle(i),
//							(int)GetRingBufferPointPoseAngle(i),(int)GetRingBufferPointLen(i),(int)GetRingBufferAverCurvature(i),(int)GetRingBufferPointVell(i));	
//						}
					mode = 7;
				break;
			
			case 7:
					PathFollowing(1);
									if(USARTSEND == 1)
						{
							
							USART_OUT_XYAngle();

							USARTSEND = 0;
						}
				break;
			
			
			
			default:
				
				break;
		
		}
	
	}


}



