/*
 * File: FOC_Model.c
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

#include "FOC_Model.h"

/* Named constants for Chart: '<S2>/Chart' 
 * 定义电机状态机的几个核心状态枚举
 */
#define IN_AlignStage                  ((uint8_T)1U) // 预定位阶段 (强拖起步准备)
#define IN_IDLE                        ((uint8_T)2U) // 待机阶段
#define IN_OpenStage                   ((uint8_T)3U) // 开环强拖启动阶段
#define IN_RunStage                    ((uint8_T)4U) // 闭环运行阶段 (FOC Sensorless/Hall 运行)
#define NumBitsPerChar                 8U            // 字节位宽

/* Exported block signals */
real32_T ThetaOpen;                    /* 开环强制积分角度 (或者闭环时的反馈角度) */

/* Exported data definition */

/* Definition for custom storage class: Struct 
 * 电机极对数 Pn
 */
motor_type motor = {
  /* Pn */
  2.0F
};

/* 速度环 PI 控制器参数 */
spd_kpki_type spd_kpki = {
  /* spd_ki 积分增益 */
  0.00144F,

  /* spd_kp 比例增益 */
  0.003389F
};

/* Block signals and states (default storage) 
 * 存储延时、积分器状态、状态机计数器等上下文变量
 */
DW rtDW;

/* External inputs (root inport signals with default storage) 
 * 外部输入接口：关联 ADC 采样电流、母线电压、参考速度等
 */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) 
 * 外部输出接口：关联输出给定时器的三相占空比 tABC
 */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* 函数声明 */
extern real32_T rt_modf_snf(real32_T u0, real32_T u1);
static void SVPWM(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus, real32_T rty_tABC[3]);
static void rate_scheduler(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);

/* * 内存对齐和 IEEE-754 浮点数表示相关的联合体
 * 用于定义和检测 NaN (非数字) 与 Inf (无穷大)
 */
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

/* 浮点数极值变量定义 */
real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/* IEEE-754 异常值获取与初始化相关函数 */
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

static real_T rtGetNaN(void) {
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;
    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }
  return nan;
}

static real32_T rtGetNaNF(void) {
  IEEESingle nanF = { { 0.0F } };
  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

static void rt_InitInfAndNaN(size_t realSize) {
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

static boolean_T rtIsInf(real_T value) {
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

static boolean_T rtIsInfF(real32_T value) {
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

static boolean_T rtIsNaN(real_T value) {
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;
    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }
  return result;
}

static boolean_T rtIsNaNF(real32_T value) {
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

static real_T rtGetInf(void) {
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;
    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }
  return inf;
}

static real32_T rtGetInfF(void) {
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

static real_T rtGetMinusInf(void) {
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;
    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }
  return minf;
}

static real32_T rtGetMinusInfF(void) {
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * 速率调度器 (多速率控制)
 * 电流环运行在基准频率 (TID[0], 例如 10kHz), 速度环运行在较低频率 (TID[1], 例如 1kHz).
 * 这里计数器每次加1，到9清零，说明电流环与速度环的频率比例是 10:1。
 */
static void rate_scheduler(void)
{
  (rtM->Timing.TaskCounters.TID[1])++;
  if ((rtM->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.001s, 0.0s] */
    rtM->Timing.TaskCounters.TID[1] = 0;
  }
}

/* * SVPWM 空间矢量调制 - 零序注入等效算法
 * 参数说明：
 * rtu_Valpha: 目标电压在静止正交坐标系的 alpha 轴分量
 * rtu_Vbeta:  目标电压在静止正交坐标系的 beta 轴分量
 * rtu_v_bus:  当前读取的母线电压 (用于归一化)
 * rty_tABC:   计算输出给三相 PWM 的占空比计数
 */
static void SVPWM(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus, real32_T rty_tABC[3])
{
  real32_T rtb_Min;
  real32_T rtb_Sum1_a;
  real32_T rtb_Sum_o;

  /* * 1. 逆 Clarke 变换，计算静止三相坐标系参考相电压
   * V_a = V_alpha
   * V_b = -0.5 * V_alpha + (sqrt(3)/2) * V_beta
   * V_c = -0.5 * V_alpha - (sqrt(3)/2) * V_beta
   */
  rtb_Min = -0.5F * rtu_Valpha;                               // 计算 -0.5 * V_alpha
  rtb_Sum1_a = 0.866025388F * rtu_Vbeta;                      // 0.866025388 是 sqrt(3)/2, 计算 (sqrt(3)/2) * V_beta
  rtb_Sum_o = rtb_Min + rtb_Sum1_a;                           // 此处得到 V_b
  rtb_Sum1_a = rtb_Min - rtb_Sum1_a;                          // 此处得到 V_c

  /* * 2. 计算零序电压分量 V_zero
   * 为了达到中点钳位并发射马鞍波，V_zero = -(V_max + V_min) / 2
   * 这里 rtu_Valpha 代表 V_a, rtb_Sum_o 代表 V_b, rtb_Sum1_a 代表 V_c
   */
  rtb_Min = (fminf(fminf(rtu_Valpha, rtb_Sum_o), rtb_Sum1_a) + fmaxf(fmaxf(rtu_Valpha, rtb_Sum_o), rtb_Sum1_a)) * -0.5F;

  /* * 3. 注入零序分量，形成最终调制的马鞍波电压
   */
  rty_tABC[0] = rtb_Min + rtu_Valpha;
  rty_tABC[1] = rtb_Min + rtb_Sum_o;
  rty_tABC[2] = rtb_Min + rtb_Sum1_a;

  /* * 4. 占空比映射与反极性处理
   * 将期望电压按母线电压 (rtu_v_bus) 归一化。
   * 公式: Duty = (-V_phase / V_bus + 0.5) * 8000
   * 乘 -1 是由于硬件或特定发波配置下的极性反转匹配；0.5 为偏置零点；8000 是 TIM1 的计数周期。
   */
  rty_tABC[0] = (-rty_tABC[0] / rtu_v_bus + 0.5F) * 8000.0F;
  rty_tABC[1] = (-rty_tABC[1] / rtu_v_bus + 0.5F) * 8000.0F;
  rty_tABC[2] = (-rty_tABC[2] / rtu_v_bus + 0.5F) * 8000.0F;
}

/* 浮点取模函数，用于角度在 0~2Pi 内的循环限制 */
real32_T rt_modf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  y = u0;
  if (u1 == 0.0F) {
    if (u0 == 0.0F) {
      y = u1;
    }
  } else if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
    y = (rtNaNF);
  } else if (u0 == 0.0F) {
    y = 0.0F / u1;
  } else if (rtIsInfF(u1)) {
    if ((u1 < 0.0F) != (u0 < 0.0F)) {
      y = u1;
    }
  } else {
    boolean_T yEq;
    y = fmodf(u0, u1);
    yEq = (y == 0.0F);
    if ((!yEq) && (u1 > floorf(u1))) {
      real32_T q;
      q = fabsf(u0 / u1);
      yEq = !(fabsf(q - floorf(q + 0.5F)) > FLT_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0F;
    } else if ((u0 < 0.0F) != (u1 < 0.0F)) {
      y += u1;
    }
  }

  return y;
}

/* =========================================================================
 * FOC 核心步进执行函数
 * 由 ADC 中断以固定频率调用 (如 10kHz)
 * ========================================================================= */
void FOC_Model_step(void)
{
  real32_T Integrator;
  real32_T rtb_IntegralGain;
  real32_T rtb_Saturation;
  real32_T rtb_SignPreIntegrator;
  real32_T rtb_SignPreIntegrator_l;
  real32_T rtb_SinCos;
  real32_T rtb_SinCos1;
  real32_T rtb_Sum;

  /* -----------------------------------------------------------------------
   * 1. 速度环逻辑 (多速率调度，由 TID[1] 决定，例如每 10 次执行 1 次)
   * ----------------------------------------------------------------------- */
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    uint32_T speedloop_ELAPS_T;

    /* 计算离散采样周期跨度 (以 Tick 为单位)，用于积分系数计算 */
    if (rtDW.speedloop_RESET_ELAPS_T) {
      speedloop_ELAPS_T = 0U;
    } else {
      speedloop_ELAPS_T = rtM->Timing.clockTick1 - rtDW.speedloop_PREV_T;
    }
    rtDW.speedloop_PREV_T = rtM->Timing.clockTick1;
    rtDW.speedloop_RESET_ELAPS_T = false;

    /* 计算速度误差： Error = 目标速度(SpeedRef) - 反馈速度(SpeedFd) */
    rtb_IntegralGain = (real32_T)rtU.SpeedRef - rtU.SpeedFd;

    /* 速度环积分器更新逻辑 (前向欧拉法积分) */
    if (rtDW.Integrator_SYSTEM_ENABLE != 0) {
      Integrator = rtDW.Integrator_DSTATE_h;
    } else if ((rtDW.SpeedReset > 0.0) && (rtDW.Integrator_PrevResetState <= 0)) {
      rtDW.Integrator_DSTATE_h = 0.0F;
      Integrator = rtDW.Integrator_DSTATE_h;
    } else {
      /* 离散积分: 新积分值 = 旧积分值 + 步长(0.001 * ELAPS_T) * 增益前的误差 */
      Integrator = 0.001F * (real32_T)speedloop_ELAPS_T * rtDW.Integrator_PREV_U + rtDW.Integrator_DSTATE_h;
    }

    /* 速度环 PI 总输出: PI_Output = Kp * Error + 积分结果 */
    rtb_Sum = spd_kpki.spd_kp * rtb_IntegralGain + Integrator;

    /* 速度环抗积分饱和 (Anti-windup) 与 输出限幅 (Saturation) (-3.0 ~ 3.0) 
     * 计算限幅前的死区超调量 rtb_SignPreIntegrator_l，反馈给积分器
     */
    if (rtb_Sum > 3.0F) {
      rtb_SignPreIntegrator_l = rtb_Sum - 3.0F;
      rtDW.Saturation = 3.0F;
    } else {
      if (rtb_Sum >= -3.0F) {
        rtb_SignPreIntegrator_l = 0.0F;
      } else {
        rtb_SignPreIntegrator_l = rtb_Sum - -3.0F;
      }

      if (rtb_Sum < -3.0F) {
        rtDW.Saturation = -3.0F;
      } else {
        rtDW.Saturation = rtb_Sum;
      }
    }

    /* 将速度误差乘以积分系数 Ki，留作下个周期积分计算使用 */
    rtb_IntegralGain *= spd_kpki.spd_ki;

    /* 存储当前速度环状态 */
    rtDW.Integrator_SYSTEM_ENABLE = 0U;
    rtDW.Integrator_DSTATE_h = Integrator;
    if (rtDW.SpeedReset > 0.0) {
      rtDW.Integrator_PrevResetState = 1;
    } else if (rtDW.SpeedReset < 0.0) {
      rtDW.Integrator_PrevResetState = -1;
    } else if (rtDW.SpeedReset == 0.0) {
      rtDW.Integrator_PrevResetState = 0;
    } else {
      rtDW.Integrator_PrevResetState = 2;
    }

    /* 符号函数预处理 (为 Anti-windup 钳位判断做准备) */
    if (rtb_SignPreIntegrator_l < 0.0F) {
      Integrator = -1.0F;
    } else if (rtb_SignPreIntegrator_l > 0.0F) {
      Integrator = 1.0F;
    } else if (rtb_SignPreIntegrator_l == 0.0F) {
      Integrator = 0.0F;
    } else {
      Integrator = (rtNaNF);
    }

    if (rtb_IntegralGain < 0.0F) {
      rtb_SinCos1 = -1.0F;
    } else if (rtb_IntegralGain > 0.0F) {
      rtb_SinCos1 = 1.0F;
    } else if (rtb_IntegralGain == 0.0F) {
      rtb_SinCos1 = 0.0F;
    } else {
      rtb_SinCos1 = (rtNaNF);
    }

    /* 条件钳位抗积分饱和算法 (Clamping Anti-Windup)
     * 如果 PI 输出已经达到限幅值，且积分方向导致过饱和恶化，则将本次积分量置零，停止累计。
     */
    if ((0.0F * rtb_Sum != rtb_SignPreIntegrator_l) && ((int8_T)Integrator == (int8_T)rtb_SinCos1)) {
      rtDW.Integrator_PREV_U = 0.0F;
    } else {
      rtDW.Integrator_PREV_U = rtb_IntegralGain;
    }
  }

  /* -----------------------------------------------------------------------
   * 2. 电流环逻辑 (以基频 TID[0] 运行)
   * ----------------------------------------------------------------------- */
  
  /* * Clarke 变换 (3/2变换)，求解静止正交坐标系 I_alpha
   * 公式: I_alpha = 2/3 * Ia - 1/3 * (Ib + Ic)
   * 其中 0.666666687F 约等于 2/3, 0.333333343F 约等于 1/3
   */
  rtb_SignPreIntegrator_l = 0.666666687F * rtU.ia - (rtU.ib + rtU.ic) * 0.333333343F;

  /* Stateflow 电机状态机控制逻辑 */
  if (rtDW.temporalCounter_i1 < 32767U) {
    rtDW.temporalCounter_i1++;
  }

  if (rtDW.is_active_c3_FOC_Model == 0U) {
    rtDW.is_active_c3_FOC_Model = 1U;
    rtDW.is_c3_FOC_Model = IN_IDLE;
  } else {
    switch (rtDW.is_c3_FOC_Model) {
     case IN_AlignStage: /* 对齐/预定位阶段 */
      if (rtDW.temporalCounter_i1 >= 2000) {
        rtDW.is_c3_FOC_Model = IN_OpenStage;
        rtDW.temporalCounter_i1 = 0U;
        rtDW.ZReset = 0.0;
        rtDW.cnt = 0.0;
      } else if (rtU.Motor_OnOff == 0.0F) {
        rtDW.is_c3_FOC_Model = IN_IDLE;
      } else {
        rtDW.Motor_state = 2.0;
      }
      break;

     case IN_IDLE: /* 待机阶段 */
      if (rtU.Motor_OnOff == 1.0F) {
        rtDW.is_c3_FOC_Model = IN_AlignStage;
        rtDW.temporalCounter_i1 = 0U;
      } else {
        rtDW.Motor_state = 1.0;
      }
      break;

     case IN_OpenStage: /* 强制开环启动阶段 */
      if (rtU.Motor_OnOff == 0.0F) {
        rtDW.is_c3_FOC_Model = IN_IDLE;
      } else if (rtDW.temporalCounter_i1 >= 20000) {
        rtDW.is_c3_FOC_Model = IN_RunStage;
      } else {
        if (rtDW.cnt == 1.0) {
          rtDW.ZReset = 1.0;
        }
        rtDW.cnt = 1.0;
        rtDW.Motor_state = 3.0;
        rtDW.SpeedReset = 0.0;
      }
      break;

     default: /* IN_RunStage 正常闭环运行阶段 */
      if (rtU.Motor_OnOff == 0.0F) {
        rtDW.is_c3_FOC_Model = IN_IDLE;
      } else {
        rtDW.Motor_state = 4.0;
        rtDW.SpeedReset = 1.0;
      }
      break;
    }
  }

  /* 根据状态机当前状态，分配电流环目标值 Iq_ref 与 位置反馈 ThetaOpen */
  switch ((int32_T)rtDW.Motor_state) {
   case 1: /* 待机 */
    ThetaOpen = 0.0F;
    rtDW.Merge1 = 0.0F; // Iq_ref = 0
    break;

   case 2: /* 对齐 */
    ThetaOpen = 0.0F;
    rtDW.Merge1 = 1.0F; // Iq_ref = 1.0 锁定电流
    break;

   case 3: /* 开环步进运行 */
    if ((rtDW.ZReset > 0.0) && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0)) {
      rtDW.DiscreteTimeIntegrator_DSTATE = 0.0F;
    }
    rtb_IntegralGain = rtDW.DiscreteTimeIntegrator_DSTATE;

    if ((rtDW.ZReset > 0.0) && (rtDW.DiscreteTimeIntegrator1_PrevRes <= 0)) {
      rtDW.DiscreteTimeIntegrator1_DSTATE = 0.0F;
    }

    /* 计算开环强制积分角度 */
    ThetaOpen = rt_modf_snf(rtDW.DiscreteTimeIntegrator1_DSTATE, 6.28318548F);
    rtDW.Merge1 = 1.0F; // 开环 Iq 给定

    /* 离散积分更新角速度和角度 */
    rtDW.DiscreteTimeIntegrator_DSTATE += motor.Pn * 83.7758F * 0.5F * 0.0001F;
    if (rtDW.ZReset > 0.0) {
      rtDW.DiscreteTimeIntegrator_PrevRese = 1;
    } else if (rtDW.ZReset < 0.0) {
      rtDW.DiscreteTimeIntegrator_PrevRese = -1;
    } else if (rtDW.ZReset == 0.0) {
      rtDW.DiscreteTimeIntegrator_PrevRese = 0;
    } else {
      rtDW.DiscreteTimeIntegrator_PrevRese = 2;
    }

    rtDW.DiscreteTimeIntegrator1_DSTATE += 0.0001F * rtb_IntegralGain;
    if (rtDW.ZReset > 0.0) {
      rtDW.DiscreteTimeIntegrator1_PrevRes = 1;
    } else if (rtDW.ZReset < 0.0) {
      rtDW.DiscreteTimeIntegrator1_PrevRes = -1;
    } else if (rtDW.ZReset == 0.0) {
      rtDW.DiscreteTimeIntegrator1_PrevRes = 0;
    } else {
      rtDW.DiscreteTimeIntegrator1_PrevRes = 2;
    }
    break;

   case 4: /* 闭环正常运行 */
    ThetaOpen = rtU.theta;          // 采用外部反馈角度（如Hall运算结果）
    rtDW.Merge1 = rtDW.Saturation;  // Iq_ref 取自外环速度 PI 的输出
    break;
  }

  /* 求解当前电角度的三角函数值，后续坐标变换频繁调用 */
  rtb_SinCos1 = cosf(ThetaOpen);
  rtb_SinCos = sinf(ThetaOpen);

  /* * Clarke 变换 (3/2变换)，求解静止正交坐标系 I_beta
   * 公式: I_beta = sqrt(3)/3 * (Ib - Ic)
   * 其中 0.577350259F 约等于 sqrt(3)/3
   */
  rtb_IntegralGain = (rtU.ib - rtU.ic) * 0.577350259F;

  /* * Park 变换求 Id 误差 
   * 公式: Id = I_alpha * cos(theta) + I_beta * sin(theta)
   * 考虑到表面式永磁电机通常 Id_ref = 0，
   * 因此误差 Error_Id = 0 - Id = -(I_alpha * cos + I_beta * sin)
   */
  Integrator = 0.0F - (rtb_SignPreIntegrator_l * rtb_SinCos1 + rtb_IntegralGain * rtb_SinCos);

  /* d 轴电流环 PI 控制器: Vd = Kp * Error_Id + Integral_Id
   * 此处 Id 环 Kp = 0.26F
   */
  rtb_Sum = Integrator * 0.26F + rtDW.Integrator_DSTATE;

  /* d 轴输出电压限幅 (-12.47V ~ +12.47V) */
  if (rtb_Sum > 12.4707661F) {
    rtb_Saturation = 12.4707661F;
  } else if (rtb_Sum < -12.4707661F) {
    rtb_Saturation = -12.4707661F;
  } else {
    rtb_Saturation = rtb_Sum;
  }

  /* * Park 变换求 Iq 误差
   * 公式: Iq = -I_alpha * sin(theta) + I_beta * cos(theta)
   * 误差 Error_Iq = Iq_ref - Iq = rtDW.Merge1 - (-I_alpha * sin + I_beta * cos)
   * 代码结构重排等效于上式。
   */
  rtb_IntegralGain = rtDW.Merge1 - (rtb_IntegralGain * rtb_SinCos1 - rtb_SignPreIntegrator_l * rtb_SinCos);

  /* q 轴电流环 PI 控制器: Vq = Kp * Error_Iq + Integral_Iq
   * 此处 Iq 环 Kp = 0.26F
   */
  rtb_SignPreIntegrator_l = rtb_IntegralGain * 0.26F + rtDW.Integrator_DSTATE_b;

  /* q 轴输出电压限幅 (-12.47V ~ +12.47V) */
  if (rtb_SignPreIntegrator_l > 12.4707661F) {
    rtb_SignPreIntegrator = 12.4707661F;
  } else if (rtb_SignPreIntegrator_l < -12.4707661F) {
    rtb_SignPreIntegrator = -12.4707661F;
  } else {
    rtb_SignPreIntegrator = rtb_SignPreIntegrator_l;
  }

  /* * 逆 Park 变换 (Anti-Park) 并调用 SVPWM
   * 将同步旋转坐标系下的给定电压 Vd, Vq 转回静止坐标系下的 V_alpha, V_beta
   * V_alpha = Vd * cos(theta) - Vq * sin(theta)  --> (rtb_Saturation * rtb_SinCos1 - rtb_SignPreIntegrator * rtb_SinCos)
   * V_beta  = Vd * sin(theta) + Vq * cos(theta)  --> (rtb_Saturation * rtb_SinCos + rtb_SignPreIntegrator * rtb_SinCos1)
   */
  SVPWM(rtb_Saturation * rtb_SinCos1 - rtb_SignPreIntegrator * rtb_SinCos,
        rtb_Saturation * rtb_SinCos + rtb_SignPreIntegrator * rtb_SinCos1,
        rtU.v_bus, rtY.tABC);

  /* ------------- 下方为电流环的抗积分饱和(Anti-windup)和离散状态更新 ------------- */
  
  /* d轴零增益与死区处理 */
  rtb_SinCos1 = 0.0F * rtb_Sum;

  if (rtb_Sum > 12.4707661F) {
    rtb_Sum -= 12.4707661F;
  } else if (rtb_Sum >= -12.4707661F) {
    rtb_Sum = 0.0F;
  } else {
    rtb_Sum -= -12.4707661F;
  }

  /* Id 环积分乘法计算，Ki = 35.0F */
  Integrator *= 35.0F;

  /* q轴零增益与死区处理 */
  rtb_SinCos = 0.0F * rtb_SignPreIntegrator_l;

  if (rtb_SignPreIntegrator_l > 12.4707661F) {
    rtb_SignPreIntegrator_l -= 12.4707661F;
  } else if (rtb_SignPreIntegrator_l >= -12.4707661F) {
    rtb_SignPreIntegrator_l = 0.0F;
  } else {
    rtb_SignPreIntegrator_l -= -12.4707661F;
  }

  /* Iq 环积分乘法计算，Ki = 35.0F */
  rtb_IntegralGain *= 35.0F;

  /* Id 环积分钳位判断 */
  if (rtb_Sum < 0.0F) {
    rtb_Saturation = -1.0F;
  } else if (rtb_Sum > 0.0F) {
    rtb_Saturation = 1.0F;
  } else if (rtb_Sum == 0.0F) {
    rtb_Saturation = 0.0F;
  } else {
    rtb_Saturation = (rtNaNF);
  }

  if (Integrator < 0.0F) {
    rtb_SignPreIntegrator = -1.0F;
  } else if (Integrator > 0.0F) {
    rtb_SignPreIntegrator = 1.0F;
  } else if (Integrator == 0.0F) {
    rtb_SignPreIntegrator = 0.0F;
  } else {
    rtb_SignPreIntegrator = (rtNaNF);
  }

  if ((rtb_SinCos1 != rtb_Sum) && ((int8_T)rtb_Saturation == (int8_T)rtb_SignPreIntegrator)) {
    Integrator = 0.0F;
  }

  /* Id 积分量更新 (步长0.0001秒，对应 10kHz) */
  rtDW.Integrator_DSTATE += 0.0001F * Integrator;

  /* Iq 环积分钳位判断 */
  if (rtb_SignPreIntegrator_l < 0.0F) {
    Integrator = -1.0F;
  } else if (rtb_SignPreIntegrator_l > 0.0F) {
    Integrator = 1.0F;
  } else if (rtb_SignPreIntegrator_l == 0.0F) {
    Integrator = 0.0F;
  } else {
    Integrator = (rtNaNF);
  }

  if (rtb_IntegralGain < 0.0F) {
    rtb_SinCos1 = -1.0F;
  } else if (rtb_IntegralGain > 0.0F) {
    rtb_SinCos1 = 1.0F;
  } else if (rtb_IntegralGain == 0.0F) {
    rtb_SinCos1 = 0.0F;
  } else {
    rtb_SinCos1 = (rtNaNF);
  }

  if ((rtb_SinCos != rtb_SignPreIntegrator_l) && ((int8_T)Integrator == (int8_T)rtb_SinCos1)) {
    rtb_IntegralGain = 0.0F;
  }

  /* Iq 积分量更新 (步长0.0001秒，对应 10kHz) */
  rtDW.Integrator_DSTATE_b += 0.0001F * rtb_IntegralGain;

  /* 更新控制频率的时间 Tick */
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    rtM->Timing.clockTick1++;
  }

  rate_scheduler();
}

/* * 模型初始化函数 
 * 系统上电或复位时调用，用来初始化 NaN/Inf 并清理各积分器的历史状态 
 */
void FOC_Model_initialize(void)
{
  rt_InitInfAndNaN(sizeof(real_T));

  rtDW.Integrator_PrevResetState = 2;
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;
  rtDW.DiscreteTimeIntegrator1_PrevRes = 2;

  rtDW.speedloop_RESET_ELAPS_T = true;
  rtDW.Integrator_SYSTEM_ENABLE = 1U;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
