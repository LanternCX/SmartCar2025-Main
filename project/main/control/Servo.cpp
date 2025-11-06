#include <iostream>

#include "Motor.h"
#include "zf_common_headfile.h"

#include "Servo.h"
#include "PID.h"
#include "LowPass.h"
#include "Math.h"
#include "Debug.h"

 /**
 * @file Servo.cpp
 * @brief 舵机相关操作
 * @author Cao Xin
 * @date 2025-04-06
 */

struct pwm_info servo_pwm_info;
servo_params servo_param;
pid_param pid;

/**
 * @brief 设置舵机占空比
 * @param duty 舵机占空比
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void set_servo_duty(float duty){
    duty = clip(duty, SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX);
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(duty));
}

/**
 * @brief 舵机初始化
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void servo_init(){
    // 获取PWM设备信息
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
    servo_param.base_duty = 88;
    set_servo_duty(servo_param.base_duty);
}

servo_params get_servo_param(){
    return servo_param;
}