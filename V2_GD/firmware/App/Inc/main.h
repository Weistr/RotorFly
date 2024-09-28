
#ifndef MAIN_H
#define MAIN_H
#include "gd32f3x0.h"
typedef struct 
{
	uint8_t Run;               //任务状态：Run/Stop
	uint16_t TIMCount;         //定时计数器
	uint16_t TRITime;          //重载计数器
	void (*TaskHook) (void); //任务函数
} TASK_COMPONENTS;       


//========================================================================
//                             外部函数和变量声明
//========================================================================

void Task_Marks_Handler_Callback(void);
void Task_Pro_Handler_Callback(void);
void nop(void);
#endif /* MAIN_H */
