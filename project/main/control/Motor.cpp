#include <algorithm>
#include <iostream>

#include "zf_common_headfile.h"

#include "Motor.h"
#include "PID.h"
#include "LowPass.h"
#include "Math.h"

#include "Debug.h"
#include "zf_driver_delay.h"

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
const int base_duty = 2750;
// const int base_duty = 0;
const int det_duty = 0;
static int left_duty = base_duty + det_duty;
static int right_duty = base_duty;

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
/**
 * @brief 校准左右轮转速
 * @author Cao Xin
 * @date 2025-07-09
 */
int sync_motor_duty(int duty) {
    std::cout << "\n\nsync motor duty" << '\n';
    // 固定左轮转速对右轮转速进行尝试直到两边转速的平均值差不多
    // 实际上是固定左轮占空比对右轮占空比进行二分
    int num = 100;
    int base_duty = duty;

    auto check  = [&](int mid) {
        float eps = 0.1;
        float sum_l = 0, sum_r = 0;
        int duty_r = base_duty;
        int duty_l = base_duty + mid;
        right_motor_run(duty_r, 1);
        left_motor_run(duty_l, 0);

        // wait for response
        system_delay_ms(100);

        // right
        for (int i = 0; i < num; i++) {
            float now_r = encoder_get_count(ENCODER_1);
            now_r = low_pass_filter(&right_low_pass, now_r);
            sum_r += now_r;
            float now_l = -encoder_get_count(ENCODER_2);
            now_l = low_pass_filter(&left_low_pass, now_l);
            sum_l += now_l;
            system_delay_ms(50);
        }

        sum_l /= num;
        sum_r /= num;

        float diff = sum_l - sum_r;

        std::cout << "now diff: " << diff << '\n';
        if (abs(diff) < eps) return 0;
        else if (diff < 0) return -1;
        else return 1;
    };

    // det duty: [-300, 300]
    int l = -300, r = 300;
    int result = 0;

    while (std::abs(l - r) > 0) {
        int mid = (l + r) / 2;
        std::cout << "now det:" << mid << '\n';
        int res = check(mid);
        result = mid;
        debug(res);
        if (res == 0) {
            r = mid - 1;
            break;
        } else if (res < 0) {
            l = mid + 1;
        } else {
            r = mid - 1;
        }
    }

    std::cout << "result: " << result << '\n';
    while (true) {
        // right_motor_run(base_duty, 1);
        // left_motor_run(base_duty - result, 0);
        float now_r = encoder_get_count(ENCODER_1);
        now_r = low_pass_filter(&right_low_pass, now_r);
        float now_l = -encoder_get_count(ENCODER_2);
        now_l = low_pass_filter(&left_low_pass, now_l);
        // debug(now_r, now_l);
    }

    return base_duty + result;
}
