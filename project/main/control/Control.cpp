#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "LowPass.h"
#include "Control.hpp"
#include "Math.h"
#include "Motor.h"
#include "Servo.h"
#include "Debug.h"

#include "image.hpp"

/**
 * @file PID.cpp
 * @brief 方向控制主要操作
 * @author Cao Xin
 * @date 2025-04-06
 */

// 方向环 PID 参数
pid_param dir_pid;
// 方向环低通参数
low_pass_param dir_low_pass;
// 当前速度参数
speed_param speed;

std::vector<control_param> pids;

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_dir_pid(pid_param &pid) {
    pid.kp = 0.00;
    pid.ki = 0.00;
    pid.kd = 0.00;

    pid.low_pass = 0.8;

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
 * @brief 模糊 PID 初始化
 */
void init_fuzzy_pid() {
    pids.push_back(control_param(0.60, 3.40, 0.5));
    pids.push_back(control_param(1.10, 3.40, 0.5));
    pids.push_back(control_param(1.10, 1.40, 0.5));
    pids.push_back(control_param(1.10, 1.40, 0.5));
    // pids.push_back(control_param(1.10, 0.20, 0.5));
    // pids.push_back(control_param(1.20, 0.30, 0.5));
    // pids.push_back(control_param(1.15, 0.30, 0.5));`
    // pids.push_back(control_param(1.40, 0.40, 0.5));
    // pids.push_back(control_param(1.10, 0.00, 0.5));
    // pids.push_back(control_param(1.10, 0.00, 0.5));
    // pids.push_back(control_param(1.30, 0.00, 0.5));
}

/**
 * @brief 初始化位置低通滤波器
 * @param lowpass 低通滤波器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_dir_lowpass(low_pass_param &lowpass) {
    lowpass.alpha = 0.5;
}

/**
 * @brief 方向控制初始化
 * @param line_speed 直线速度
 * @param curve_speed 曲线速度
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void control_init(int line_speed, int curve_speed) {
    init_dir_pid(dir_pid);
    init_dir_lowpass(dir_low_pass);
    init_fuzzy_pid();
    speed.line_speed = line_speed;
    speed.curve_speed = curve_speed;
}

/**
 * @brief 设置当前的 PID 参数
 */
void set_control_param(control_param param) {
    dir_pid.kp = param.kp;
    dir_pid.kd = param.kd;
    dir_low_pass.alpha = param.low_pass;
}

/**
 * @brief 计算并设置当前元素的 PID 参数
 */
void calc_control_param() {
    // 3 * a, a = (max - det) / max, max = 60 - ImageStatus.TowPoint
    int det = ImageStatus.OFFLine;
    int siz = pids.size();
    int max = 60;
    float alpha = 1.0 * det / max;
    int idx = clip(std::floor(siz * alpha), 0, siz - 1);
    debug(idx);
    set_control_param(pids[idx]);
    if (ImageFlag.image_element_rings_flag) {
        set_control_param(control_param(1.05, 1.40, 0.5));
    }
}

/**
 * @brief 寻线到中线
 * @param now 当前中线位置
 * @param target 目标中线位置 一般为 width / 2
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void to_center(int now, int target) {
    // 计算模糊 pid
    calc_control_param();

    // 误差
    static int error = 0;
    // 舵机占空比相较目标点的位置值
    static int servo_duty_det = 0;
    
    error = target - now;
    error = low_pass_filter(&dir_low_pass, error);
    servo_duty_det = pid_slove(&dir_pid, error);
    set_servo_duty(get_servo_param().base_duty + servo_duty_det);
    
    // 计算差速比 10% pre 5 degree 
    int det = speed.current * (servo_duty_det / 5.0 * 0.1);
    det = 0;
    set_left_speed(speed.current + det);
    set_right_speed(speed.current - det);
}