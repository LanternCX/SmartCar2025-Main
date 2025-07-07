#include <algorithm>
#include <iostream>

#include "zf_common_headfile.h"

#include "Motor.h"
#include "PID.h"
#include "LowPass.h"
#include "Math.h"

#include "Debug.h"

 /**
 * @file Motor.cpp
 * @brief 后轮相关操作
 * @author Cao Xin
 * @date 2025-04-05
 */

// Motor PWM Info
struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;

// PID
pid_param left_pid;
pid_param right_pid;

// Low Pass Filter
low_pass_param left_low_pass;
low_pass_param right_low_pass;

// Duty
const int base_duty = 2300;
const int det_duty = 220;
static int left_duty = base_duty;
static int right_duty = base_duty - det_duty;

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void init_motor_pid(pid_param &pid){
    pid.kp = 0.030;
    pid.ki = 0.035;
    pid.kd = 0.025;

    pid.p_max = 30.0;
    pid.i_max = 30.0;
    pid.d_max = 30.0;

    pid.out_min = -30.0;
    pid.out_max = 30.0;
    
    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;

    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

/**
 * @brief 初始化低通滤波器
 * @param lowpass 低通滤波器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void init_motor_low_pass(low_pass_param &lowpass){
    lowpass.alpha = 0.3;
    lowpass.last = 0.0;
}

/**
 * @brief 设置左电机的占空比和方向
 * @param duty 占空比
 * @param dir 方向
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void left_motor_run(int duty, int dir){
    gpio_set_level(MOTOR1_DIR, dir);
    int duty_real = duty * (MOTOR1_PWM_DUTY_MAX / 10000);
    duty_real = min(duty, 3000);
    pwm_set_duty(MOTOR1_PWM, duty_real);
}

/**
 * @brief 设置右电机的占空比和方向
 * @param duty 占空比
 * @param dir 方向
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void right_motor_run(int duty, int dir){
    gpio_set_level(MOTOR2_DIR, dir);
    int duty_real = duty * (MOTOR2_PWM_DUTY_MAX / 10000);
    duty_real = min(duty, 3000);
    pwm_set_duty(MOTOR2_PWM, duty_real);
}

/**
 * @brief 电机初始化
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void motor_init(){
    // 获取PWM设备信息
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);

    // 初始化左右电机 PID 控制器
    init_motor_pid(left_pid);
    init_motor_pid(right_pid);

    // 初始化左右编码器低通滤波器
    init_motor_low_pass(left_low_pass);
    init_motor_low_pass(right_low_pass);
    
    // Debug 信息
    if(MOTOR_DEBUG){
        std::cout << "Motor Param:" << '\n';
        std::cout << "P: " << left_pid.kp;
        std::cout << "I: " << left_pid.ki;
        std::cout << "D: " << left_pid.kd << '\n';
        std::cout << "Alpha: " << left_low_pass.alpha << '\n';
    }
}

/**
 * @brief 设置左轮转速
 * @param speed 目标转速, 单位 rpm
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void set_left_speed(int speed){
    // 获取当前转速
    // 左右编码器因为朝向安装的不同左电机需要取相反数
    float now = -encoder_get_count(ENCODER_2);
    // 对转速进行低通滤波
    now = low_pass_filter(&left_low_pass, now);
    // 计算误差
    float error = speed - now;
    // PID 计算增量
    float det = increment_pid_solve(&left_pid, error);
    
    // 计算输出值
    // left_duty += det;
    
    // 输出到电机控制
    left_motor_run(abs(left_duty), left_duty > 0 ? 0 : 1);
    if (MOTOR_DEBUG) {
        // debug("left", now);
        // cout << now << '\n';
    }
}

/**
 * @brief 设置右轮转速
 * @param speed 转速, 单位 rpm
 * @return none
 * @author Cao Xin
 * @date 2025-04-05
 */
void set_right_speed(int speed){
    // 获取当前转速
    float now = encoder_get_count(ENCODER_1);
    // 对转速进行低通滤波
    now = low_pass_filter(&right_low_pass, now);
    // 计算误差
    float error = speed - now;
    // PID 计算增量
    float det = increment_pid_solve(&right_pid, error);
    
    // 计算输出值
    // right_duty += det;

    
    // 输出到电机控制
    // debug(abs(right_duty));
    right_motor_run(abs(right_duty), right_duty < 0 ? 0 : 1);
    
    // Debug 信息
    if(MOTOR_DEBUG){
        // debug("right", now);
        // cout << now << '\n';
    }
}
