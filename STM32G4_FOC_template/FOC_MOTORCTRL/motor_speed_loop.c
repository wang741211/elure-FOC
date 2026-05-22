//#include <rtthread.h>
//#include "motor_app_fsm.h"
//#include "motor_mw_foc.h"

///* 线程控制块指针 */
//static rt_thread_t speed_loop_thread = RT_NULL;

//extern SRF_PLL_t bemf_pll; 

///* 速度环 PI 控制器专属数据结构 */
//typedef struct {
//    float Kp;             /* 比例系数：决定响应的爆发力 */
//    float Ki;             /* 积分系数：决定消除稳态误差的速度 */
//    float error_sum;      /* 积分历史累加器 */
//    float out_limit;      /* 输出总限幅 (物理单位: A) */
//    float integral_limit; /* 积分项独立限幅，防止深度饱和 (物理单位: A) */
//} Speed_PI_Controller_t;

///* * 【未归一化的物理量 PI 参数经验整定】
// * 设计目标：输入误差是转速 (RPM)，输出是定子扭矩电流 (A)
// * 假设差 100 RPM，我们希望立刻给出 0.3A 的驱动力 -> Kp = 0.003
// * 假设差 100 RPM，我们希望 1 秒钟内积分能追加 2.0A 的驱动力 -> Ki = 0.02
// */
//static Speed_PI_Controller_t speed_pi = {
//    .Kp = 0.003f,         
//    .Ki = 0.02f,
//    .error_sum = 0.0f,
//    .out_limit = 3.0f,    /* 保护硬件：最大允许 3.0A 的输出 */
//    .integral_limit = 2.0f /* 积分项最多只能提供 2.0A 的力量，留出 1.0A 给 Kp 做动态响应 */
//};

///* 全局目标转速 (RPM) */
//float global_target_speed_rpm = 1500.0f; 
//    float current_speed_rpm = 0.0f;    
//    float target_iq = 0.0f;    

///* 速度环线程入口函数 */
//static void speed_loop_entry(void *parameter)
//{
//    rt_tick_t tick;
//    float current_speed_rpm = 0.0f;    
//    float target_iq = 0.0f;        

//    /* 获取初始绝对时间戳 */
//    tick = rt_tick_get();

//    while (1)
//    {
//        /* 仅在闭环状态下接管 Iq 的控制权 */
//        if (Motor_APP_GetState() == APP_STATE_RUN_CLOSED)
//        {
//            /* -------------------------------------------------------------
//             * 1. 速度反馈获取与滤波
//             * ------------------------------------------------------------- */
//            /* 直接从底层的 PLL 锁相环读取角速度，并转为 RPM
//             * 极对数是 4.0f
//             */
//            current_speed_rpm = bemf_pll.Omega * (60.0f / 6.283185f) / 4.0f;
//            
//            /* 无感观测器算出的速度通常有高频毛刺，必须加一阶低通滤波
//             * Alpha = 0.1f，表示信任 90% 的历史数据，10% 的新数据
//             */
//            static float filtered_speed_rpm = 0.0f;
//            filtered_speed_rpm = filtered_speed_rpm * 0.9f + current_speed_rpm * 0.1f;

//            /* 计算物理转速误差 (RPM) */
//            float error = global_target_speed_rpm - filtered_speed_rpm;

//            /* -------------------------------------------------------------
//             * 2. 离散 PI 算法核心与抗积分饱和 (Anti-Windup)
//             * ------------------------------------------------------------- */
//            /* 【写代码的物理标准】：既然在跑 1ms 的周期，必须把时间步长 Ts (0.001s) 乘进去！
//             * 这样您的 Ki 参数才具有时间尺度上的物理意义，否则换个线程频率参数就得全废。
//             */
//            speed_pi.error_sum += error * 0.001f;

//            /* 积分限幅 (防御性编程)：
//             * 将误差和限制在 (integral_limit / Ki) 的范围内。
//             */
//            if (speed_pi.error_sum > speed_pi.integral_limit / speed_pi.Ki) {
//                speed_pi.error_sum = speed_pi.integral_limit / speed_pi.Ki;
//            } else if (speed_pi.error_sum < -speed_pi.integral_limit / speed_pi.Ki) {
//                speed_pi.error_sum = -speed_pi.integral_limit / speed_pi.Ki;
//            }

//            /* 计算最终目标电流 (A) */
//            target_iq = (speed_pi.Kp * error) + (speed_pi.Ki * speed_pi.error_sum);

//            /* 总输出限幅 (保护功率管) */
//            if (target_iq > speed_pi.out_limit) target_iq = speed_pi.out_limit;
//            if (target_iq < -speed_pi.out_limit) target_iq = -speed_pi.out_limit;

//            /* -------------------------------------------------------------
//             * 3. 跨线程数据下发
//             * ------------------------------------------------------------- */
//            /* 写入 FOC 物理输入总线 */
//            g_foc_vars.Iq_Ref = target_iq;
//        }
//        else
//        {
//            speed_pi.error_sum = 0.0f;
//        }

//        /* -------------------------------------------------------------
//         * 4. 绝对时间片延时
//         * ------------------------------------------------------------- */
//        /* 严丝合缝的 1ms 绝对周期休眠，零抖动 */
//        rt_thread_delay_until(&tick, 1);
//    }
//}

///* 线程初始化与导出 */
//int motor_speed_loop_init(void)
//{
//    speed_loop_thread = rt_thread_create("spd_loop",
//                                         speed_loop_entry,
//                                         RT_NULL,
//                                         1024,
//                                         5,   
//                                         10); 
//    
//    if (speed_loop_thread != RT_NULL)
//    {
//        rt_thread_startup(speed_loop_thread);
//    }
//    return 0;
//}
//INIT_APP_EXPORT(motor_speed_loop_init);
