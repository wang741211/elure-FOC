/*
 * File: FOC_Model.h
 *
 * Code generated for Simulink model 'FOC_Model'.
 *
 * Model version                  : 1.17
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 * 1. Execution efficiency
 * 2. RAM efficiency
 */

#ifndef RTW_HEADER_FOC_Model_h_
#define RTW_HEADER_FOC_Model_h_
#include <stddef.h>
#include <float.h>
#include <math.h>
#ifndef FOC_Model_COMMON_INCLUDES_
#define FOC_Model_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOC_Model_COMMON_INCLUDES_ */

/* 宏定义：用于访问实时模型错误状态 */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* 实时模型结构体的前置声明 */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' 
 * 模块信号与状态工作区 (DW: Discrete Workspace)
 * 用于保存算法在运行过程中需要跨周期记忆的上下文变量，如积分器状态、状态机标志等。
 */
typedef struct {
  real_T Motor_state;                  /* 电机当前主状态 (1:待机, 2:预定位, 3:开环起步, 4:闭环运行) */
  real_T ZReset;                       /* 位置积分器复位标志 (用于开环阶段的角度强拖) */
  real_T SpeedReset;                   /* 速度积分器复位标志 */
  real_T cnt;                          /* 状态机内部通用计数器 */
  real32_T Merge1;                     /* 电流环 q 轴参考值 (Iq_ref)，由速度环输出或开环强制给定 */
  real32_T Saturation;                 /* 速度环 PI 输出限幅后的最终结果 */
  real32_T Integrator_DSTATE;          /* d轴 电流环 PI 控制器的离散积分器历史状态 */
  real32_T Integrator_DSTATE_b;        /* q轴 电流环 PI 控制器的离散积分器历史状态 */
  real32_T DiscreteTimeIntegrator_DSTATE; /* 开环拖动阶段：角速度积分器状态 */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* 开环拖动阶段：电角度积分器状态 */
  real32_T Integrator_DSTATE_h;        /* 速度环 PI 控制器的离散积分器历史状态 */
  real32_T Integrator_PREV_U;          /* 速度环前一拍的误差输入 (用于抗积分饱和判断) */
  uint32_T speedloop_PREV_T;           /* 速度环上一次执行的系统 Tick 绝对时间戳 */
  uint16_T temporalCounter_i1;         /* 状态机的时间计数器，用于判断状态转移 (例如延时多久进入下一步) */
  int8_T DiscreteTimeIntegrator_PrevRese;/* 开环角速度积分器的前置复位状态标记 */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* 开环电角度积分器的前置复位状态标记 */
  int8_T Integrator_PrevResetState;    /* 速度环积分器的前置复位状态标记 */
  uint8_T is_active_c3_FOC_Model;      /* 状态机激活标志 (0:未激活, 1:已激活) */
  uint8_T is_c3_FOC_Model;             /* 当前所处的具体 Stateflow 子状态 */
  uint8_T Integrator_SYSTEM_ENABLE;    /* 速度环积分器系统使能标志 */
  boolean_T speedloop_RESET_ELAPS_T;   /* 速度环耗时计算复位标志 */
} DW;

/* External inputs (root inport signals with default storage) 
 * 外部输入接口结构体 (ExtU: External Inputs)
 * 硬件层 (如 ADC 中断) 获取到数据后，必须赋值给这里的变量供 FOC 算法读取
 */
typedef struct {
  real32_T ia;                         /* A相实际采样电流 (物理量，单位通常为 A) */
  real32_T ib;                         /* B相实际采样电流 (物理量，单位通常为 A) */
  real32_T ic;                         /* C相实际采样电流 (物理量，单位通常为 A) */
  real32_T v_bus;                      /* 当前直流母线电压 (物理量，单位为 V，用于 SVPWM 占空比归一化) */
  real32_T Motor_OnOff;                /* 电机启停控制指令 (1.0F: 启动, 0.0F: 停止) */
  real_T SpeedRef;                     /* 目标参考电角速度 (单位通常为 rad/s 或 RPM) */
  real32_T SpeedFd;                    /* 实际反馈电角速度 (由霍尔传感器或观测器计算得出) */
  real32_T theta;                      /* 实际反馈转子电角度 (由霍尔传感器计算，范围 0 ~ 2Pi) */
} ExtU;

/* External outputs (root outports fed by signals with default storage) 
 * 外部输出接口结构体 (ExtY: External Outputs)
 * FOC 算法执行完毕后，将结果写入此结构体，硬件层读取并写入定时器
 */
typedef struct {
  real32_T tABC[3];                    /* 最终输出给 TIM1 的三相 PWM 比较寄存器 (CCR) 值 
                                        * tABC[0] -> CCR1 (A相)
                                        * tABC[1] -> CCR2 (B相)
                                        * tABC[2] -> CCR3 (C相)
                                        */
} ExtY;

/* Type definition for custom storage class: Struct 
 * 电机本体物理参数结构体定义
 */
typedef struct motor_tag {
  real32_T Pn;                         /* 电机极对数 (Pole Pairs)，用于机械角度与电角度转换 */
} motor_type;

/* 速度环 PI 控制参数结构体定义 */
typedef struct spd_kpki_tag {
  real32_T spd_ki;                     /* 速度环积分增益 Ki */
  real32_T spd_kp;                     /* 速度环比例增益 Kp */
} spd_kpki_type;

/* Real-time Model Data Structure 
 * 实时模型内部时基与错误状态管理结构体
 */
struct tag_RTM {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * 包含系统任务调度和多速率运行的时基信息
   */
  struct {
    uint32_T clockTick1;               /* 基准时钟滴答计数 (对应 TID[0]，如 10kHz) */
    struct {
      uint8_T TID[2];                  /* 任务分频计数器，TID[1] 用于触发低频任务 (如速度环) */
    } TaskCounters;
  } Timing;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/*
 * Exported Global Signals
 * 导出的全局信号：开环计算角度，供观测或外部调试使用
 */
extern real32_T ThetaOpen;             /* '<S2>/Merge' */

/* Model entry point functions 
 * 算法入口函数声明
 */
extern void FOC_Model_initialize(void); /* 系统初始化函数，开机或复位时调用一次 */
extern void FOC_Model_step(void);       /* FOC 运算步进函数，放在高频 ADC 中断中周期性调用 */

/* Exported data declaration 
 * 暴露给外部修改或读取的参数结构体实例
 */
extern motor_type motor;
extern spd_kpki_type spd_kpki;

/* Real-time Model object */
extern RT_MODEL *const rtM;

#endif                                 /* RTW_HEADER_FOC_Model_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
 

