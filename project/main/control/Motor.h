#include "PID.h"

/**
 * @file Motor.h
 * @brief 后轮操作头文件
 * @author Cao Xin
 * @date 2025-04-05
 */

 #pragma once

// 电机 1 方向驱动路径
#define MOTOR1_DIR "/dev/zf_driver_gpio_motor_1"
// 电机 1 PWM 驱动路径
#define MOTOR1_PWM "/dev/zf_device_pwm_motor_1"

// 电机 2 方向驱动路径
#define MOTOR2_DIR "/dev/zf_driver_gpio_motor_2"
// 电机 2 PWM 驱动路径
#define MOTOR2_PWM "/dev/zf_device_pwm_motor_2"

// 编码器 1 驱动路径
#define ENCODER_1 "/dev/zf_encoder_1"
// 编码器 2 驱动路径
#define ENCODER_2 "/dev/zf_encoder_2"

// 电机 1 最大占空比, 在设备树定义
#define MOTOR1_PWM_DUTY_MAX (motor_1_pwm_info.duty_max)       
// 电机 2 最大占空比, 在设备树定义
#define MOTOR2_PWM_DUTY_MAX (motor_2_pwm_info.duty_max)        

// 输入的最大占空比
#define MAX_DUTY (30) 

// 是否打开 Debug 模式
#define MOTOR_DEBUG 1


typedef struct {
    /**
     * 电机差速系数
     */
    float diff_factor;
    /**
     * 电机基础速度
     */
    float base_speed;
} motor_params;
void motor_init();
void set_left_speed(int speed);
void set_right_speed(int speed);
int sync_motor_duty(int duty);
void left_motor_run(int duty, int dir);
void right_motor_run(int duty, int dir);
void set_left_speed(int speed);
void set_right_speed(int speed);