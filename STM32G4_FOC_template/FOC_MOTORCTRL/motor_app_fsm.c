#include "motor_app_fsm.h"
#include "motor_foc.h"
#include "motor_sensor.h"
#include "motor_hal.h"

/* 实例化系统核心部件 */
MC_Foc_Vars_t   g_foc_vars;
PI_Controller_t g_pi_id;
PI_Controller_t g_pi_iq;
PI_Controller_t g_pi_speed; /* 【新增】速度环 PI 控制器实例 */

/* 动态指向当前使用的传感器/观测器 */
MC_Position_Sensor_t *Active_Sensor = &Sensor_BEMF; // 若需测霍尔，改为 &Sensor_Hall

Motor_APP_State_e current_state = APP_STATE_IDLE;
uint8_t  motor_cmd_run = 0;
uint32_t state_timer = 0;

/* 开环虚拟参数 */
float open_loop_theta = 0.0f;
float open_loop_speed = 0.0f;

/* 全局目标转速 (RPM) */
float global_target_speed_rpm = 1500.0f; 

/* ==========================================================
 * 速度外环 PI 运算函数 (带抗积分饱和)
 * 注意：由于速度环跑在 1kHz，积分步长 Ts = 0.001s
 * ========================================================== */
static float Speed_PI_Update(PI_Controller_t *pi, float reference, float feedback)
{
    float error = reference - feedback;
    float output;

    /* 比例项 + 历史积分项 */
    output = pi->Kp * error + pi->Integral;

    /* 抗积分饱和：只有在未触及输出天花板时，才累加积分 */
    if (output >= pi->OutMax) {
        output = pi->OutMax;
    } else if (output <= pi->OutMin) {
        output = pi->OutMin;
    } else {
        /* 注意：这里硬编码了 0.001f，因为分频后是 1ms 的执行周期 */
        pi->Integral += pi->Ki * error * 0.001f; 
    }

    return output;
}

void Motor_APP_Init(void)
{
    Motor_HAL_Init();
    Motor_HAL_DisablePWM(); /* 硬件绝对防御：关闭定时器输出 */
    Motor_HAL_CalibrateOffsets();
    
    /* 初始化电流环 PI 参数 (10kHz响应) */
    g_pi_id.Kp = 0.26f;  g_pi_id.Ki = 35.0f;
    g_pi_id.OutMax = 12.47f; g_pi_id.OutMin = -12.47f;
    g_pi_id.Integral = 0.0f;
    
    g_pi_iq.Kp = 0.26f;  g_pi_iq.Ki = 35.0f;
    g_pi_iq.OutMax = 12.47f; g_pi_iq.OutMin = -12.47f;
    g_pi_iq.Integral = 0.0f;

    /* 【新增】初始化速度环 PI 参数 (基于真实物理量：误差RPM -> 输出A) */
    g_pi_speed.Kp = 0.003f;
    g_pi_speed.Ki = 0.02f;
    g_pi_speed.OutMax = 3.0f;  /* 最大请求 3.0A 的驱动力 */
    g_pi_speed.OutMin = 0.0f; /* 最大请求 -3.0A 的刹车力 */
    g_pi_speed.Integral = 0.0f;
    
    Motor_FOC_Init(&g_foc_vars, &g_pi_id, &g_pi_iq);
    Active_Sensor->Init();
}

/**
  * @brief 10kHz ADC 中断核心调度函数
  */
void Motor_APP_Task_10kHz(void)
{
    /* 1. 安全指令拦截 */
    if (motor_cmd_run == 0) {
        current_state = APP_STATE_IDLE;
    }

    /* 2. 获取最原始的物理数据放入总线 */
    Motor_HAL_GetPhaseCurrents(&g_foc_vars.Ia, &g_foc_vars.Ib, &g_foc_vars.Ic);
    g_foc_vars.Vbus = Motor_HAL_GetBusVoltage();

    /* 3. 驱动激活的观测器更新自身状态 (提取上一拍的 V_alpha/beta) */
    Active_Sensor->Update(g_foc_vars.V_alpha, g_foc_vars.V_beta, 
                          g_foc_vars.I_alpha, g_foc_vars.I_beta);

    /* 4. 干净清爽的业务流转状态机 */
    switch (current_state) 
    {
        case APP_STATE_IDLE:
            Motor_HAL_DisablePWM();
            Motor_FOC_Init(&g_foc_vars, &g_pi_id, &g_pi_iq); /* 核心变量清零防止突变 */
            Active_Sensor->Init();
            
            g_pi_speed.Integral = 0.0f; /* 【防御】待机时清空速度积分 */
            open_loop_theta = 0.0f;
            state_timer = 0;
            
            if (motor_cmd_run == 1) {
                current_state = APP_STATE_ALIGN;
                Motor_HAL_EnablePWM();
            }
            break;

        case APP_STATE_ALIGN:
            /* 预定位：锁定在 0 度，注入励磁电流 */
            g_foc_vars.Theta_Elec = 0.0f;
            g_foc_vars.Id_Ref = 1.0f; 
            g_foc_vars.Iq_Ref = 0.0f;
            
            if (++state_timer >= 2000) { /* 200ms 后切换 */
                current_state = APP_STATE_OPEN_LOOP;
                state_timer = 0;
            }
            break;

        case APP_STATE_OPEN_LOOP:
            g_pi_speed.Integral = 0.0f; /* 【防御】强拖阶段继续封锁速度积分 */
            
            /* 开环强拖：不涉及底层复杂数学，纯粹的虚拟角度积分与电流给定 */
            open_loop_speed += 0.5f; 
            if (open_loop_speed > 1000.0f) open_loop_speed = 1000.0f;
            
            /* 离散积分累加电角度 */
            open_loop_theta += open_loop_speed * 0.1047197f * 4.0f * 0.0001f;
            if (open_loop_theta > 6.283185f) open_loop_theta -= 6.283185f;
            
            g_foc_vars.Theta_Elec = open_loop_theta;
            g_foc_vars.Id_Ref = 0.0f;
            g_foc_vars.Iq_Ref = 1.0f; /* 固定给 1.0A 强拉 */
            
            /* 当观测器标志位表明反电动势或 PLL 已锁定时，柔性切换到闭环 */
            if (++state_timer >= 50000 || Active_Sensor->is_ready) {
                current_state = APP_STATE_RUN_CLOSED;
            }
            break;

        case APP_STATE_RUN_CLOSED:
        {
            /* 1. 角度环 10kHz 极速更新 */
            g_foc_vars.Theta_Elec = Active_Sensor->GetTheta();
            g_foc_vars.Id_Ref = 0.0f; /* 表面式永磁同步通常 Id=0 */

            /* ==========================================================
             * 2. 速度环软件分频调度 (10kHz / 10 = 1kHz = 1ms)
             * ========================================================== */
            static uint8_t speed_loop_cnt = 0;
            speed_loop_cnt++;
            if (speed_loop_cnt >= 10) 
            {
                speed_loop_cnt = 0;

                /* (1) 获取物理转速 (提取观测器的 rad/s，假设极对数为 4) */
                float raw_speed_rads = Active_Sensor->GetSpeed();
                float current_speed_rpm = raw_speed_rads * (60.0f / 6.2831853f) / 4.0f;
                
                /* (2) 一阶低通滤波消除无感速度抖动 (Alpha = 0.1) */
                static float filtered_speed_rpm = 0.0f;
                filtered_speed_rpm = filtered_speed_rpm * 0.9f + current_speed_rpm * 0.1f;

                /* (3) 调用标准的 PI 计算模块，并将结果直接赋值给电流内环的 Q 轴 */
                g_foc_vars.Iq_Ref = Speed_PI_Update(&g_pi_speed, global_target_speed_rpm, filtered_speed_rpm);
            }
            break;
        }
    }

    /* 5. 纯数学解算与占空比物理下发 (只有在非 IDLE 状态才执行发波) */
    if (current_state != APP_STATE_IDLE) {
        Motor_FOC_Step(&g_foc_vars, &g_pi_id, &g_pi_iq);
        Motor_HAL_SetPWM(g_foc_vars.Duty_A, g_foc_vars.Duty_B, g_foc_vars.Duty_C);
    }
}

/**
  * @brief  获取当前电机应用层的状态
  * @retval Motor_APP_State_e 当前状态机枚举值
  */
Motor_APP_State_e Motor_APP_GetState(void)
{
    return current_state;
}

/**
  * @brief  下发电机启动指令
  * @note   仅修改指令标志位，具体硬件动作由 10kHz 中断内的状态机异步执行，保证安全
  */
void Motor_APP_Start(void)
{
    motor_cmd_run = 1;
}

/**
  * @brief  下发电机停止指令
  */
void Motor_APP_Stop(void)
{
    motor_cmd_run = 0;
}
