#include "motor_hal.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "opamp.h"
#include "comp.h"

/* ---------------- 硬件私有参数宏定义 ---------------- */
/* 电流计算物理系数：(3.3V / 4096) / (采样电阻 * 运放增益) 
 * 假设 R_shunt = 0.01 Ohm, Gain = 15V/V -> 系数约为 0.00537
 * 这里根据您之前的代码逻辑使用固定系数 0.02197265625f 
 */
#define ADC_VOLTAGE_REF    3.3f
#define ADC_RESOLUTION     4096.0f
#define CURRENT_FACTOR     0.02197265625f 
#define VBUS_FACTOR        (ADC_VOLTAGE_REF / ADC_RESOLUTION * 26.0f)

/* 定时器计数值周期 (TIM1->ARR) */
#define PWM_PERIOD_COUNT   8000.0f

/* ---------------- 硬件私有静态变量 ---------------- */
static uint32_t Offset_Ia = 2048;
static uint32_t Offset_Ib = 2048;
static uint32_t Offset_Ic = 2048;

/**
  * @brief  初始化所有电机相关的底层硬件外设
  */
void Motor_HAL_Init(void)
{
    /* 调用由 CubeMX 生成的外设初始化函数 */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_OPAMP1_Init();
    MX_OPAMP2_Init();
    MX_OPAMP3_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_COMP1_Init();
    MX_DAC3_Init();

    /* 启动内部运放 */
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    /* 开启比较器与对应的 DAC 阈值 (3000 对应约 2.4V) */
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 3000);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
    HAL_COMP_Start(&hcomp1);

    /* 配置 ADC 注入组触发源并启动 (由 TIM1 CH4 触发) */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);

    /* 启动定时器主计数器 */
    HAL_TIM_Base_Start(&htim1);
    
    /* 启动TIM1 CH4 PWM输出 - 这是触发ADC注入组转换的关键！ */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
  * @brief  开启 PWM 功率输出
  */
void Motor_HAL_EnablePWM(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* 开启触发 ADC 的通道 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
  * @brief  紧急封锁 PWM 功率输出
  */
void Motor_HAL_DisablePWM(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/**
  * @brief  获取三相物理电流
  */
void Motor_HAL_GetPhaseCurrents(real32_T *ia, real32_T *ib, real32_T *ic)
{
    /* 直接从 ADC 注入组结果寄存器读取数据并转换 */
    uint32_T raw_a = ADC1->JDR1;
    uint32_T raw_b = ADC2->JDR1;
    uint32_T raw_c = ADC1->JDR2;

    *ia = (real32_T)((int32_T)raw_a - (int32_T)Offset_Ia) * CURRENT_FACTOR;
    *ib = (real32_T)((int32_T)raw_b - (int32_T)Offset_Ib) * CURRENT_FACTOR;
    *ic = (real32_T)((int32_T)raw_c - (int32_T)Offset_Ic) * CURRENT_FACTOR;
}

/**
  * @brief  获取母线电压
  */
real32_T Motor_HAL_GetBusVoltage(void)
{
    /* ADC2 通道 1 负责母线电压采集 */
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
    {
        return (real32_T)HAL_ADC_GetValue(&hadc2) * VBUS_FACTOR;
    }
    return 0.0f;
}

/**
  * @brief  下发标准化占空比 (0.0 ~ 1.0)
  */
void Motor_HAL_SetPWM(real32_T dutyA, real32_T dutyB, real32_T dutyC)
{
    /* 占空比安全限幅，防止 PWM 计数器溢出或电荷泵失效 */
    if (dutyA > 0.98f) dutyA = 0.98f; else if (dutyA < 0.02f) dutyA = 0.02f;
    if (dutyB > 0.98f) dutyB = 0.98f; else if (dutyB < 0.02f) dutyB = 0.02f;
    if (dutyC > 0.98f) dutyC = 0.98f; else if (dutyC < 0.02f) dutyC = 0.02f;

    /* 将比例转换为定时器寄存器值 */
    TIM1->CCR1 = (uint32_T)(dutyA * PWM_PERIOD_COUNT);
    TIM1->CCR2 = (uint32_T)(dutyB * PWM_PERIOD_COUNT);
    TIM1->CCR3 = (uint32_T)(dutyC * PWM_PERIOD_COUNT);
}

/**
  * @brief  执行零电流校准
  */
void Motor_HAL_CalibrateOffsets(void)
{
    uint32_T sum_a = 0, sum_b = 0, sum_c = 0;
    const uint8_T sample_num = 64;

    /* 确保 PWM 关闭时进行校准 */
    Motor_HAL_DisablePWM();
    HAL_Delay(10);

    for (uint8_T i = 0; i < sample_num; i++)
    {
        /* 手动触发或等待 ADC 采样 */
        HAL_ADCEx_InjectedStart(&hadc1);
        HAL_ADCEx_InjectedStart(&hadc2);
        HAL_Delay(1); 
        sum_a += ADC1->JDR1;
        sum_b += ADC2->JDR1;
        sum_c += ADC1->JDR2;
    }

    Offset_Ia = sum_a / sample_num;
    Offset_Ib = sum_b / sample_num;
    Offset_Ic = sum_c / sample_num;
}
