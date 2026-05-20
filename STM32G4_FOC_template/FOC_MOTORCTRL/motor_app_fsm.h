#ifndef MOTOR_APP_FSM_H
#define MOTOR_APP_FSM_H
#include "motor_mw_foc.h"
#include "rtwtypes.h"
#include "motor_pll.h"

/* * 电机运行状态机枚举
 * 严格对应原 Simulink 模型中的四种状态
 */
typedef enum {
    APP_STATE_IDLE = 0,     /* 待机阶段：封锁输出，等待启动指令 */
    APP_STATE_ALIGN,        /* 预定位阶段：注入恒定 D 轴电流，将转子吸合至初始位置 */
    APP_STATE_OPEN_LOOP,    /* 开环强拖阶段：强制旋转磁场，克服机械惯量 */
    APP_STATE_RUN_CLOSED    /* 闭环运行阶段：切入正常的有感/无感 FOC 闭环控制 */
} Motor_APP_State_e;

/* 应用层初始化函数 (在 main 函数进入 while(1) 前调用) */
void Motor_APP_Init(void);

/* 电机启动指令下发 */
void Motor_APP_Start(void);

/* 电机停止指令下发 */
void Motor_APP_Stop(void);

/* * 应用层核心调度任务
 * 必须放置在 10kHz 的 ADC 注入组完成中断中周期性调用
 */
void Motor_APP_Task_10kHz(void);

void Motor_VF_Spin_Test(void);

/* 获取当前状态机状态 (可供串口或 CAN 状态上报使用) */
Motor_APP_State_e Motor_APP_GetState(void);

extern  FOC_Input_t foc_in;
extern  FOC_Output_t foc_out;
extern  SRF_PLL_t bemf_pll;

#endif /* MOTOR_APP_FSM_H */

