#include "motor_app_fsm.h"
#include "motor_hal.h"
#include "motor_mw_foc.h"
#include <math.h>

/* ---------------- 内部私有变量与对象 ---------------- */
static Motor_APP_State_e current_state = APP_STATE_IDLE;
static uint8_T  motor_cmd_run = 0;       /* 运行指令标志位 (0:停机, 1:启动) */
static uint32_T state_timer = 0;         /* 状态机时间计数器 (步长 100us) */

/* 核心算法层的输入输出结构体实例 */
static FOC_Input_t foc_in;
static FOC_Output_t foc_out;

/* 开环强拖使用的虚拟积分角度与速度 */
static real32_T open_loop_theta = 0.0f;
static real32_T open_loop_speed = 0.0f;

/* 原有工程的速度环 PI 参数 (提取自 Simulink spd_kpki) */
static real32_T spd_kp = 0.003389f;
static real32_T spd_ki = 0.00144f;
static real32_T spd_integral = 0.0f;
static real32_T target_speed_rpm = 1000.0f; /* 目标转速 */

/* 声明一个模拟的霍尔/观测器角度获取接口 (实际中可替换为您解耦后的观测器接口) */
extern real32_T HallTheta;
extern real32_T HallSpeed;

/**
  * @brief  简单的速度环 PI 控制器 (带限幅与抗积分饱和)
  * @param  ref_speed: 目标转速
  * @param  fdb_speed: 反馈转速
  * @retval Iq_ref:    输出给电流环的 Q 轴目标电流 (A)
  */
static real32_T Speed_Loop_PI(real32_T ref_speed, real32_T fdb_speed)
{
    real32_T error = ref_speed - fdb_speed;
    real32_T output;
    
    /* 速度环 PI 计算 */
    output = spd_kp * error + spd_integral;
    
    /* 速度环输出限幅 (提取自原工程 -3.0A ~ 3.0A) */
    real32_T limit_max = 3.0f;
    real32_T limit_min = -3.0f;
    
    if (output > limit_max) {
        output = limit_max;
    } else if (output < limit_min) {
        output = limit_min;
    } else {
        /* 未饱和时才进行积分累加，防止抗积分饱和 (Anti-windup) */
        spd_integral += spd_ki * error; 
    }
    
    return output;
}

/**
  * @brief  应用层初始化
  */
void Motor_APP_Init(void)
{
    /* 初始化硬件底层 */
    Motor_HAL_Init();
    
    /* 上电默认关闭 PWM */
    Motor_HAL_DisablePWM();
    
    /* 执行底层运放零偏校准 */
    Motor_HAL_CalibrateOffsets();
    
    /* 初始化 FOC 纯数学算法层 */
    Motor_MW_FOC_Init();
    
    current_state = APP_STATE_IDLE;
    motor_cmd_run = 0;
}

/**
  * @brief  接收启动指令
  */
void Motor_APP_Start(void)
{
    motor_cmd_run = 1;
}

/**
  * @brief  接收停止指令
  */
void Motor_APP_Stop(void)
{
    motor_cmd_run = 0;
}

/**
  * @brief  获取当前状态机状态
  */
Motor_APP_State_e Motor_APP_GetState(void)
{
    return current_state;
}

/**
  * @brief  核心调度节拍，运行于 10kHz (100us) 的中断中
  */
void Motor_APP_Task_10kHz(void)
{
    /* 1. 全局检查：只要指令为停止，立刻切回待机状态并封锁输出 */
    if (motor_cmd_run == 0) {
        current_state = APP_STATE_IDLE;
    }

    /* 2. 从 HAL 层获取干净的物理量 */
    Motor_HAL_GetPhaseCurrents(&foc_in.Ia, &foc_in.Ib, &foc_in.Ic);
    foc_in.Vbus = Motor_HAL_GetBusVoltage();

    /* 3. 核心状态机流转与策略执行 */
    switch (current_state) 
    {
        case APP_STATE_IDLE:
            /* 待机状态：关闭硬件发波，清零所有算法状态 */
            Motor_HAL_DisablePWM();
            state_timer = 0;
            open_loop_theta = 0.0f;
            spd_integral = 0.0f;
            
            foc_in.Id_Ref = 0.0f;
            foc_in.Iq_Ref = 0.0f;
            
            if (motor_cmd_run == 1) {
                current_state = APP_STATE_ALIGN;
                Motor_HAL_EnablePWM(); /* 状态转移：开启波形输出 */
            }
            break;

        case APP_STATE_ALIGN:
            /* 预定位阶段：固定电角度为 0，注入一定的 D 轴电流进行转子吸合 */
            foc_in.Theta_Elec = 0.0f;
            foc_in.Id_Ref = 1.0f;  /* 注入 1.0A 预定位电流 (需根据电机特性调整) */
            foc_in.Iq_Ref = 0.0f;  /* Q 轴不出力 */
            
            /* 调用数学层执行一次 FOC 运算 */
            Motor_MW_FOC_Step(&foc_in, &foc_out);
            
            /* 下发占空比驱动硬件 */
            Motor_HAL_SetPWM(foc_out.Duty_A, foc_out.Duty_B, foc_out.Duty_C);
            
            /* 时间控制：2000 拍 (200ms) 后切入开环启动阶段 */
            state_timer++;
            if (state_timer >= 2000) {
                current_state = APP_STATE_OPEN_LOOP;
                state_timer = 0;
                open_loop_speed = 0.0f; // 开环速度初值重置
            }
            break;

        case APP_STATE_OPEN_LOOP:
            /* 开环强拖阶段：不依赖传感器，人为制造旋转磁场拉动转子 */
            
            /* 模拟一个平滑加速的开环转速 (此处可替换为您实际的加速曲线) */
            open_loop_speed += 0.05f; 
            if (open_loop_speed > 300.0f) {
                open_loop_speed = 300.0f; /* 限制最大强拖转速 */
            }
            
            /* 离散积分更新开环角度：Theta = Theta + Omega * Ts 
             * 2.0 * PI / 60 转换为弧度制，0.0001f 是 10kHz 的周期
             */
            open_loop_theta += open_loop_speed * 0.1047197f * 2.0f /* 极对数 */ * 0.0001f;
            
            /* 角度循环限幅 0 ~ 2Pi */
            if (open_loop_theta > 6.283185f) open_loop_theta -= 6.283185f;
            
            /* 将生成的开环角度赋给 FOC */
            foc_in.Theta_Elec = open_loop_theta;
            foc_in.Id_Ref = 0.0f;
            foc_in.Iq_Ref = 1.0f;  /* 开环阶段给定 1.0A 的固定驱动电流 */
            
            Motor_MW_FOC_Step(&foc_in, &foc_out);
            Motor_HAL_SetPWM(foc_out.Duty_A, foc_out.Duty_B, foc_out.Duty_C);
            
            /* 时间控制：20000 拍 (2s) 后强制切入闭环运行阶段 */
            state_timer++;
            if (state_timer >= 20000) {
                current_state = APP_STATE_RUN_CLOSED;
                state_timer = 0;
            }
            break;

        case APP_STATE_RUN_CLOSED:
            /* 闭环运行阶段：引入真实传感器数据和外环 PI 调节 */
            
            /* 1. 角度反馈闭环 (替换为真实的观测器或霍尔变量) */
            foc_in.Theta_Elec = HallTheta; 
            
            /* 2. 速度环逻辑：以较低频率执行 (例如每 10 拍执行一次 -> 1kHz) */
            state_timer++;
            if (state_timer >= 10) {
                state_timer = 0;
                /* 计算 PI 吐出 Iq 的目标值 */
                foc_in.Iq_Ref = Speed_Loop_PI(target_speed_rpm, HallSpeed);
            }
            
            foc_in.Id_Ref = 0.0f; /* 表面式永磁电机一般采用 Id=0 控制 */
            
            /* 3. 驱动电流内环并下发占空比 */
            Motor_MW_FOC_Step(&foc_in, &foc_out);
            Motor_HAL_SetPWM(foc_out.Duty_A, foc_out.Duty_B, foc_out.Duty_C);
            break;

        default:
            current_state = APP_STATE_IDLE;
            break;
    }
}

