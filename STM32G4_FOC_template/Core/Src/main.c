/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 极致简化的核心调度入口
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "motor_app_fsm.h"
#include "sys_config.h"
#include "usart.h"
#include "fdcan.h"
#include "motor_sensor.h"
#include <string.h>
#include <stdio.h>
/* ---------------- 全局变量与外部引用 ---------------- */
/* ---------------- 全局变量与外部引用 ---------------- */
/* 实例化霍尔传感器相关的全局变量（分配内存） */
float HallTheta = 0.0f;
float HallThetaAdd = 0.0f;
float HallSpeed = 0.0f;
uint8_t HallReadTemp = 0;
#define PI  3.14159265358979f
#define PHASE_SHIFT_ANGLE   (float)(220.0f/360.0f*2.0f*PI)

/* 虚拟示波器数据泵变量 */
static float load_data[5];
static uint8_t tempData[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7F};
extern FDCAN_TxHeaderTypeDef TxHeader;

int main(void)
{
    /* 1. 硬件最低限度初始化 */
    HAL_Init();

    /* 2. 调用移出的系统时钟配置 (位于 sys_config.c) */
    SystemClock_Config();

    /* 3. 调用应用层初始化 (级联初始化 HAL/MW 层) */
    Motor_APP_Init();

    MX_USART3_UART_Init();

    /* 4. 调用移出的通讯配置 (位于 sys_config.c) */
    FDCAN_Config();
    extern uint8_t aRxBuffer;
    HAL_UART_Receive_IT(&huart3, &aRxBuffer, 1);

    /* 主循环：仅处理超低频的状态上报 */
    while (1)
    {
        uint8_t TxData[8] = {(uint8_t)Motor_APP_GetState(), 0};
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
        HAL_Delay(50); /* 20Hz 频率的心跳包足够了 */
    }
}

/**
  * @brief  核心高频中断：ADC 注入组完成 (10kHz)
  * 这是系统中唯一允许保持高频运行的地方
  */
extern  real32_T theta_elec_est;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC1)
    {
        /* 1. 位置插补逻辑 */
        HallTheta += HallThetaAdd;
        if(HallTheta < 0.0f) HallTheta += 2.0f * PI;
        else if(HallTheta > (2.0f * PI)) HallTheta -= 2.0f * PI;
            
        /* 2. 驱动应用层任务 (核心状态机与 FOC) */
        Motor_APP_Task_10kHz();
            
        /* 3. 数据观测 (VOFA+ 上位机泵出) */
				load_data[0] = g_foc_vars.I_alpha;
        load_data[1] = g_foc_vars.I_beta;
        load_data[2] = bemf_obs.e_alpha_est;
        load_data[3] = bemf_obs.e_beta_est; 
        load_data[4] = g_foc_vars.Theta_Elec;
	
        memcpy(tempData, (uint8_t *)&load_data, 20);
        HAL_UART_Transmit_DMA(&huart3, tempData, 24); 
    }
}

/**
  * @brief  霍尔位置捕获中断 (保持在 main.c 中作为核心反馈源)
  */
/* * ==============================================================================
 * 霍尔位置捕获中断 (保持在 main.c 中作为核心反馈源)
 * 核心作用：通过硬件定时器的输入捕获机制，实时测量转子转速，并在霍尔状态跳变瞬间
 * 强行校准电角度的绝对零点，消除积分累积误差。
 * ==============================================================================
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* 确保是专用于霍尔捕获的 TIM4 触发的中断 */
    if(htim->Instance == TIM4) 
    {
        /* 1. 读取捕获时间并计算速度 */
        /* HallTemp 记录了两次霍尔边沿跳变之间经历的计数器 Tick 数 */
        float HallTemp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        /* 计算 100us (10kHz FOC周期) 内需要累加的角度增量 
         * 公式：每 1 个 FOC 周期转过的角度 = (PI/3 弧度 / 经历的绝对时间) / 10000 
         */
        HallThetaAdd = (PI/3.0f) / (HallTemp / 3200000.0f) / 10000.0f; 
        
        /* 计算当前实时机械转速 (RPM)
         * 公式：电角速度(rad/s) = (PI/3) / 经历时间
         * 机械转速(RPM) = 电角速度 * 30 / PI / 极对数(此处先算电角速度对应的RPM，外层若有极对数需再除)
         */
        HallSpeed = (PI/3.0f) / (HallTemp / 3200000.0f) * 30.0f / (2.0f * PI);
        
        /* 2. 读取三个霍尔引脚电平，拼合成 3-bit 的绝对状态字 */
        HallReadTemp = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);               // W 相 (Bit 0)
        HallReadTemp |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) << 1;         // V 相 (Bit 1)
        HallReadTemp |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 2;         // U 相 (Bit 2)
        
        /* 3. 状态查表：强行校准转子绝对电角度 
         * 根据标准 120度 霍尔真值表，每一个离散状态对应转子跨过了一个 60度 (PI/3) 的扇区边界。
         * 同时叠加 PHASE_SHIFT_ANGLE (220度)，补偿霍尔传感器安装物理位置相对于电机 A 相磁轴的固定偏差。
         */
        if(HallReadTemp == 0x05) {           // 状态 5：对应 0度起始
            HallTheta = 0.0f + PHASE_SHIFT_ANGLE;
        } 
        else if(HallReadTemp == 0x04) {      // 状态 4：对应 60度 (PI/3) 起始
            HallTheta = (PI / 3.0f) + PHASE_SHIFT_ANGLE;
        } 
        else if(HallReadTemp == 0x06) {      // 状态 6：对应 120度 (2*PI/3) 起始
            HallTheta = (PI * 2.0f / 3.0f) + PHASE_SHIFT_ANGLE;
        } 
        else if(HallReadTemp == 0x02) {      // 状态 2：对应 180度 (PI) 起始
            HallTheta = PI + PHASE_SHIFT_ANGLE;
        } 
        else if(HallReadTemp == 0x03) {      // 状态 3：对应 240度 (4*PI/3) 起始
            HallTheta = (PI * 4.0f / 3.0f) + PHASE_SHIFT_ANGLE;
        } 
        else if(HallReadTemp == 0x01) {      // 状态 1：对应 300度 (5*PI/3) 起始
            HallTheta = (PI * 5.0f / 3.0f) + PHASE_SHIFT_ANGLE;
        }
        
        /* 4. 电角度周期的边界循环处理
         * 确保补偿后的最终角度严格限制在 0 ~ 2*PI 弧度之内，防止后续正余弦数学函数 (sinf/cosf) 计算溢出或精度下降。
         */
        if(HallTheta < 0.0f) {
            HallTheta += 2.0f * PI;
        } 
        else if(HallTheta > (2.0f * PI)) {
            HallTheta -= 2.0f * PI;
        }
    }
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
int fputc(int ch, FILE *f) { while (!(USART3->ISR & 0X40)); USART3->TDR = ch; return ch; }

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

