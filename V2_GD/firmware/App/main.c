#include "gd32f3x0.h"
#include "systick.h"
#include "main.h"
#include "stdbool.h"

#include "imu.h"
#include "mpu6050.h"
#include "pid.h"
//========================================================================
// 函数: main
//========================================================================

int main(void)
{
    systick_config();
    while (1)
    {
        Task_Pro_Handler_Callback();
    }
}


//========================================================================
// 函数: basic  task
// 描述: .
// 参数: None.
// 返回: None.
//========================================================================
	
void basicTask_entry()
{
		
}


void task1()
{
	static uint8_t i,j;
	i++;
	j++;
	if(i==1)
	{
		i=0;
		MpuGetData();
		motor_control();
	}
	if(j==2)
	{
		j=0;
		GetAngle(&MPU6050,&Angle,0.006f);
	}	

}

/***************************************************/
/*********************初始化************************/
/***************************************************/

//========================================================================
//                         本地变量声明 & 任务分配
//========================================================================
static TASK_COMPONENTS Task_Comps[]=
{
//状态  计数  周期  函数
	{0, 20, 20, basicTask_entry},				/* task 1 Period： 2ms*/
	{0, 1000, 1000, task1},				/* task 2 Period： 100ms 飞控设置*/
//	{0, 50, 50, RX1_task},			/* task 3 Period： 300ms 电池电压*/
//	{0, 20, 20, adc_scan},					/* task 4 Period： 500ms */
//	{0, 500, 500, task_B},					/* task 5 Period： 500ms */
//	{0, 500, 500, task_C},					/* task 6 Period： 500ms */
//	{0, 500, 500, task_D},					/* task 7 Period： 500ms */
//	{0, 500, 500, task_E},					/* task 8 Period： 500ms */

	/* Add new task here */
};

uint8_t Tasks_Max = sizeof(Task_Comps)/sizeof(Task_Comps[0]);

//========================================================================
// 函数: Task_Handler_Callback
// 描述: 任务标记回调函数.
// 参数: None.
// 返回: None.
//========================================================================
void Task_Marks_Handler_Callback(void)
{
	uint8_t i;
	for(i=0; i<Tasks_Max; i++)
	{
		if(Task_Comps[i].TIMCount)    /* If the time is not 0 */
		{
			Task_Comps[i].TIMCount--;  /* Time counter decrement */
			if(Task_Comps[i].TIMCount == 0)  /* If time arrives */
			{
				/*Resume the timer value and try again */
				Task_Comps[i].TIMCount = Task_Comps[i].TRITime;  
				Task_Comps[i].Run = 1;    /* The task can be run */
			}
		}
	}
}

//========================================================================
// 函数: Task_Pro_Handler_Callback
// 描述: 任务处理回调函数.
// 参数: None.
// 返回: None.
//========================================================================
void Task_Pro_Handler_Callback(void)
{
	uint8_t i;
	for(i=0; i<Tasks_Max; i++)
	{
		if(Task_Comps[i].Run) /* If task can be run */
		{
			Task_Comps[i].Run = 0;    /* Flag clear 0 */
			Task_Comps[i].TaskHook();  /* Run task */
		}
	}
}

void nop(){}

