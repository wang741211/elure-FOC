#ifndef MOTOR_SENSOR_H
#define MOTOR_SENSOR_H
#include "motor_mw_foc.h"
#include <stdint.h>

/* 标准化的角度传感器/观测器接口抽象 */
typedef struct {
    /* 观测器初始化 */
    void  (*Init)(void);
    
    /* 观测器步进更新 (参数为：alpha/beta轴电压与电流)，有感传感器可留空此函数 */
    void  (*Update)(float v_alpha, float v_beta, float i_alpha, float i_beta);
    
    /* 接口：获取反馈电角度 (rad) */
    float (*GetTheta)(void);
    
    /* 接口：获取反馈电角速度 (rad/s) */
    float (*GetSpeed)(void);
    
    /* 状态位：指示当前观测器是否已经收敛可信任 */
    uint8_t is_ready;
} MC_Position_Sensor_t;

/* 外部声明具体实现的实例 (例如在 sensor_bemf.c 或 sensor_hall.c 中实现) */
extern MC_Position_Sensor_t Sensor_BEMF;
extern MC_Position_Sensor_t Sensor_Hall;
extern BEMF_Observer_t bemf_obs;

#endif
