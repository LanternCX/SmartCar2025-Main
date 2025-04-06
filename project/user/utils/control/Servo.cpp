#include <iostream>

#include "zf_common_headfile.h"

#include "Servo.h"
#include "PID.h"
#include "LowPass.h"

/**
 * 前轮舵机控制相关函数
 */

struct pwm_info servo_pwm_info;
pid_param pid;
low_pass_param centerLowPass;

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_servo_pid(pid_param &pid){
    pid.kp = 0.3;
    pid.ki = 0.00;
    pid.kd = 0.20;

    pid.p_max = 30.0;
    pid.i_max = 30.0;
    pid.d_max = 30.0;

    pid.out_min = -15.0;
    pid.out_max = 15.0;

    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;

    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

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
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(duty));
}

/**
 * @brief 寻线到中线
 * @param now 当前中线位置
 * @param target 目标中线位置 一般为 width / 2
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void servo_to_center(int now, int target){
    int error = target - now;
    int duty = 90;
    if(std::abs(error) < 50){
        duty = pid_slove(&pid, error);
        std::cerr << "target: " << target << ' ';
        std::cerr << "now: " << now << ' '; 
        std::cerr << "duty: " << duty << ' ';
        std::cerr << "error: " << error << '\n';
    }
    set_servo_duty(90 + duty);
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
    init_servo_pid(pid);
    init_servo_low_pass(centerLowPass);
}