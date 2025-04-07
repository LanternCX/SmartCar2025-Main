#include <iostream>

#include "Motor.h"
#include "zf_common_headfile.h"

#include "Servo.h"
#include "PID.h"
#include "LowPass.h"
#include "Math.h"

/**
 * 前轮舵机控制相关函数
 */

struct pwm_info servo_pwm_info;
servo_params servo_param;
pid_param pid;
low_pass_param centerLowPass;

/**
 * @brief 初始化低通滤波器
 * @param lowpass P低通滤波器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_servo_low_pass(low_pass_param &lowpass){
    lowpass.alpha = 0.1;
    lowpass.last = 0.0;
}

/**
 * @brief 设置舵机占空比
 * @param duty 舵机占空比
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void set_servo_duty(int duty){
    duty = minmax(duty, SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX);
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
    init_servo_low_pass(centerLowPass);
    servo_param.base_duty = 87;
    set_servo_duty(servo_param.base_duty);
}

servo_params get_servo_param(){
    return servo_param;
}