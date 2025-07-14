/**
 * @file Servo.h
 * @brief 舵机操作头文件
 * @author Cao Xin
 * @date 2025-04-06
 */

#pragma once

// 定义驱动路劲，该路劲由设备树生成
#define SERVO_MOTOR1_PWM "/dev/zf_device_pwm_servo"

// 定义主板上舵机频率  请务必注意范围 50-300
// 如果要修改，需要直接修改设备树。
#define SERVO_MOTOR_FREQ (servo_pwm_info.freq)                       

// 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
#define PWM_DUTY_MAX (servo_pwm_info.duty_max)      

// 定义主板上舵机活动范围 角度                                                     
#define SERVO_MOTOR_L_MAX (75)                       
#define SERVO_MOTOR_R_MAX (100)                       

// 计算舵机占空比
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

// 是否打开 Debug 模式
#define SERVO_DEBUG 0
typedef struct {
    /**
     * 舵机中线角度
     */
    float base_duty;
} servo_params;

void servo_init();
void set_servo_duty(float duty);
servo_params get_servo_param();