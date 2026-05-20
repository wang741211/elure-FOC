#include "motor_app_fsm.h"
#include "motor_hal.h"
#include "motor_mw_foc.h"
#include "motor_pll.h"
#include "stm32g4xx_hal.h" // 确保能调用 TIM1 寄存器
#include <math.h>

/* ---------------- 内部私有变量与对象 ---------------- */
 Motor_APP_State_e current_state = APP_STATE_IDLE;
 uint8_T  motor_cmd_run = 0;       /* 运行指令标志位 (0:停机, 1:启动) */
 uint32_T state_timer = 0;         /* 状态机时间计数器 (步长 100us) */
 static uint32_T run_timer = 0;

/* 核心算法层的输入输出结构体实例 */
 FOC_Input_t foc_in;
 FOC_Output_t foc_out;

/* 开环强拖使用的虚拟积分角度与速度 */
 real32_T open_loop_theta = 0.0f;
 real32_T open_loop_speed = 0.0f;

 BEMF_Observer_t bemf_obs;
 SRF_PLL_t bemf_pll;
 real32_T theta_elec_est = 0.0f;
 real32_T omega_elec_est = 0.0f;
 real32_T sensorless_theta_offset = -1.5707963f;
/* 原有工程的速度环 PI 参数 (提取自 Simulink spd_kpki) */
static real32_T spd_kp = 0.003389f;
static real32_T spd_ki = 0.00144f;
static real32_T spd_integral = 0.0f;
static real32_T target_speed_rpm = 2000.0f; /* 目标转速 */

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

    BEMF_Observer_Init(&bemf_obs, 0.59f, 0.00066f, 50.0f, 5000.0f, 0.0001f);
    SRF_PLL_Init(&bemf_pll, 200.0f, 20000.0f, 0.0001f);
    SRF_PLL_Reset(&bemf_pll, -sensorless_theta_offset, 0.0f);
    theta_elec_est = 0.0f;
    omega_elec_est = 0.0f;
    
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
uint32_T looptest_i = 3;
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
            BEMF_Observer_Init(&bemf_obs, bemf_obs.Rs, bemf_obs.Ls, bemf_obs.kp, bemf_obs.ki, bemf_obs.Ts);
            SRF_PLL_Reset(&bemf_pll, 0.0f, 0.0f);
            theta_elec_est = 0.0f;
            omega_elec_est = 0.0f;
            
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
            open_loop_speed += 0.5f; 
            if (open_loop_speed > 300.0f) {
                open_loop_speed = 300.0f; /* 限制最大强拖转速 */
            }
            
            /* 离散积分更新开环角度：Theta = Theta + Omega * Ts 
             * 2.0 * PI / 60 转换为弧度制，0.0001f 是 10kHz 的周期
             */
            open_loop_theta += open_loop_speed * 0.1047197f * 4.0f /* 极对数 */ * 0.0001f;
            
            /* 角度循环限幅 0 ~ 2Pi */
            if (open_loop_theta > 6.283185f) open_loop_theta -= 6.283185f;
            
            /* 将生成的开环角度赋给 FOC */
            foc_in.Theta_Elec = open_loop_theta;
            foc_in.Id_Ref = 0.0f;
            foc_in.Iq_Ref = looptest_i;  /* 开环阶段给定 1.0A 的固定驱动电流 */
            
            Motor_MW_FOC_Step(&foc_in, &foc_out);
            Motor_HAL_SetPWM(foc_out.Duty_A, foc_out.Duty_B, foc_out.Duty_C);

            {
                const real32_T i_alpha = 0.666666667f * foc_in.Ia - 0.333333333f * (foc_in.Ib + foc_in.Ic);
                const real32_T i_beta = 0.577350269f * (foc_in.Ib - foc_in.Ic);
                BEMF_Observer_Step(&bemf_obs, foc_out.V_alpha, foc_out.V_beta, i_alpha, i_beta);
                SRF_PLL_Step(&bemf_pll, (real32_T)bemf_obs.e_alpha_est, (real32_T)bemf_obs.e_beta_est);
                if (!isfinite(bemf_pll.Theta) || !isfinite(bemf_pll.Omega)) {
                    SRF_PLL_Reset(&bemf_pll, open_loop_theta - sensorless_theta_offset, open_loop_speed * 0.1047197f * 4.0f);
                }
                omega_elec_est = bemf_pll.Omega;
                const real32_T theta_offset = (omega_elec_est >= 0.0f) ? sensorless_theta_offset : -sensorless_theta_offset;
                theta_elec_est = bemf_pll.Theta + theta_offset;
                if (theta_elec_est < 0.0f) theta_elec_est += 6.283185f;
                else if (theta_elec_est >= 6.283185f) theta_elec_est -= 6.283185f;
            }
            
            /* 时间控制：20000 拍 (2s) 后强制切入闭环运行阶段 */
            state_timer++;
            if (state_timer >= 50000) {
                current_state = APP_STATE_RUN_CLOSED;
                state_timer = 0;
                run_timer = 0;
                const real32_T omega_init = omega_elec_est;
                const real32_T theta_offset = (omega_init >= 0.0f) ? sensorless_theta_offset : -sensorless_theta_offset;
                SRF_PLL_Reset(&bemf_pll, open_loop_theta - theta_offset, open_loop_speed * 0.1047197f * 4.0f);
                theta_elec_est = open_loop_theta;
            }
            break;

        case APP_STATE_RUN_CLOSED:
            /* 闭环运行阶段：引入真实传感器数据和外环 PI 调节 */
            run_timer++;
            
            /* 1. 角度反馈闭环 (替换为真实的观测器或霍尔变量) */
            foc_in.Theta_Elec = theta_elec_est;
            
            /* 2. 速度环逻辑：以较低频率执行 (例如每 10 拍执行一次 -> 1kHz) */
            if (0) {
                foc_in.Iq_Ref = looptest_i;
                state_timer = 0;
            } else {
                state_timer++;
                if (state_timer >= 10) {
                    state_timer = 0;
                    const real32_T pole_pairs = 4.0f;
                    const real32_T mech_speed_rpm = omega_elec_est * (60.0f / 6.283185f) / pole_pairs;
                    foc_in.Iq_Ref = Speed_Loop_PI(target_speed_rpm, mech_speed_rpm);
                }
            }
            
            foc_in.Id_Ref = 0.0f; /* 表面式永磁电机一般采用 Id=0 控制 */
            
            /* 3. 驱动电流内环并下发占空比 */
            Motor_MW_FOC_Step(&foc_in, &foc_out);
            Motor_HAL_SetPWM(foc_out.Duty_A, foc_out.Duty_B, foc_out.Duty_C);

            {
                const real32_T i_alpha = 0.666666667f * foc_in.Ia - 0.333333333f * (foc_in.Ib + foc_in.Ic);
                const real32_T i_beta = 0.577350269f * (foc_in.Ib - foc_in.Ic);
                BEMF_Observer_Step(&bemf_obs, foc_out.V_alpha, foc_out.V_beta, i_alpha, i_beta);
                SRF_PLL_Step(&bemf_pll, (real32_T)bemf_obs.e_alpha_est, (real32_T)bemf_obs.e_beta_est);
                if (!isfinite(bemf_pll.Theta) || !isfinite(bemf_pll.Omega)) {
                    SRF_PLL_Reset(&bemf_pll, foc_in.Theta_Elec - sensorless_theta_offset, 0.0f);
                }
                omega_elec_est = bemf_pll.Omega;
                const real32_T theta_offset = (omega_elec_est >= 0.0f) ? sensorless_theta_offset : -sensorless_theta_offset;
                theta_elec_est = bemf_pll.Theta + theta_offset;
                if (theta_elec_est < 0.0f) theta_elec_est += 6.283185f;
                else if (theta_elec_est >= 6.283185f) theta_elec_est -= 6.283185f;
            }
            break;

        default:
            current_state = APP_STATE_IDLE;
            break;
    }
}

/* V/F 开环控制器的物理参数配置 */
#define PI_CONST       3.1415926535f
#define PI_2_3         2.0943951023f  /* 120度电角度 */
#define PI_4_3         4.1887902047f  /* 240度电角度 */
#define PWM_ARR        8499.0f        /* 您的 10kHz 载波重装载值 */

/* 全局测试变量，方便您在 Keil 的 Watch 窗口实时修改调试 */
float VF_Theta = 0.0f;           /* 当前电角度 (rad) */
float VF_Omega = 0.0f;           /* 当前电角速度 (rad/s) */
float VF_Accel = 60.0f;          /* 加速度: 决定转速爬升快慢 */
float VF_Target_Omega = 200.0f;  /* 目标稳态转速 */

/**
 * 【致命参数】：电压调制比 (Modulation Index)
 * 物理意义：它代表直接施加在电机相线上的电压大小 (0.0 ~ 0.5 之间)。
 * 警告：由于没有电流环保护，0.1 代表使用母线电压的 10%。
 * 如果您的母线是 24V，电机电阻是 0.1欧姆，启动瞬间反电动势为0，
 * 相电流可能达到 (24 * 0.1)/0.1 = 24A！
 * 测试时，请务必从 0.02 (2%) 开始给，慢慢在 Watch 窗口往上加，直到电机刚好能转！
 */
float VF_Modulation = 0.05f;     /* 初始给 5% 的电压强拖 */

/**
 * @brief  极简 V/F SPWM 开环发波器
 * @note   纯物理强制旋转，无视任何反馈
 */
void Motor_VF_Spin_Test(void)
{
    /* 1. 频率(速度)生成：线性斜坡加速 */
    if (VF_Omega < VF_Target_Omega) {
        VF_Omega += VF_Accel * 0.0001f; /* Ts = 100us (10kHz) */
    }

    /* 2. 角度生成：对速度进行离散积分 */
    VF_Theta += VF_Omega * 0.0001f;
    
    /* 角度归一化，防止浮点数溢出 */
    if (VF_Theta >= (2.0f * PI_CONST)) {
        VF_Theta -= (2.0f * PI_CONST);
    }

    /* 3. 核心物理：生成三相相差 120 度的 SPWM 占空比 (0.0 ~ 1.0)
     * 基准为 0.5 (即 50% 占空比时，相电压为 0)
     * 注：STM32G4 带 FPU，cosf 运算在 1 微秒内即可完成，不会阻塞 10kHz 中断
     */
    float duty_a = 0.5f + VF_Modulation * cosf(VF_Theta);
    float duty_b = 0.5f + VF_Modulation * cosf(VF_Theta - PI_2_3);
    float duty_c = 0.5f + VF_Modulation * cosf(VF_Theta - PI_4_3);

    /* 4. 安全限幅：防止调制比过大导致占空比越界炸管 */
    if (duty_a > 0.95f) duty_a = 0.95f; else if (duty_a < 0.05f) duty_a = 0.05f;
    if (duty_b > 0.95f) duty_b = 0.95f; else if (duty_b < 0.05f) duty_b = 0.05f;
    if (duty_c > 0.95f) duty_c = 0.95f; else if (duty_c < 0.05f) duty_c = 0.05f;

    /* 5. 暴力下发：直接重写高级定时器的比较寄存器 */
    TIM1->CCR1 = (uint32_t)(duty_a * PWM_ARR);
    TIM1->CCR2 = (uint32_t)(duty_b * PWM_ARR);
    TIM1->CCR3 = (uint32_t)(duty_c * PWM_ARR);
}



