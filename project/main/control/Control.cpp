#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "LowPass.h"
#include "Control.hpp"
#include "Math.h"
#include "Motor.h"
#include "PID.h"
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
// 角度环 PID 参数
pid_param gyro_pid;
// 方向环低通参数
low_pass_param dir_low_pass;
// 偏航角低通参数
low_pass_param gyro_low_pass;
// 当前速度参数
speed_param speed;

std::vector<control_param> pid_left;

std::vector<std::vector<int>> pid_map;

/**
 * @brief 模糊 PID 初始化
 * @return none
 * @author Cao Xin
 * @date 2025-07-06
 */
void init_fuzzy_pid() {
    pid_left.push_back(control_param(1.20, 3.20, 0.008, 0));
    pid_left.push_back(control_param(1.75, 3.20, 0.000, 3));
    pid_left.push_back(control_param(1.85, 3.20, -0.002, 4));
    pid_left.push_back(control_param(2.05, 3.20, -0.001, 5));
    // pids.push_back(control_param(1.50, 3.0, 0.000));
    // pids.push_back(control_param(1.60, 3.0, 0.000));
    // pids.push_back(control_param(1.70, 2.0, 0.000));
    // pids.push_back(control_param(1.80, 2.0, 0.000));
    // pids.push_back(control_param(1.90, 1.0, 0.000));
    // pids.push_back(control_param(1.30, 1.0, 0.000));
    // pids.push_back(control_param(1.40, 1.0, 0.000));
    // pids.push_back(control_param(1.50, 1.0, 0.000));

    pid_map = {
        { 0, 0, 0, 0, },
        { 1, 1, 2, 2, },
        { 2, 2, 3, 3, },
        { 3, 3, 3, 3, },
    };
    // pid_map = {
    //     { 0, 0, 0, 0, },
    //     { 1, 1, 1, 1, },
    //     { 2, 2, 2, 2, },
    //     { 3, 3, 3, 3, },
    // };
}

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

    pid.p_max = 60.0;
    pid.i_max = 60.0;
    pid.d_max = 60.0;

    pid.out_min = -60.0;
    pid.out_max = 60.0;

    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;

    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

/**
 * @brief 初始化位置低通滤波器
 * @param lowpass 低通滤波器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_dir_lowpass(low_pass_param &lowpass) {
    lowpass.alpha = 0.6;
}

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_gyro_pid(pid_param &pid) {
    pid.kp = 0.0015;
    pid.ki = 0.00;
    pid.kd = 0.00;

    pid.low_pass = 0.8;

    pid.p_max = 60.0;
    pid.i_max = 60.0;
    pid.d_max = 60.0;

    pid.out_min = -60.0;
    pid.out_max = 60.0;

    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;

    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

/**
 * @brief 初始化偏航角低通滤波器
 * @param lowpass 低通滤波器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-07-08
 */
void init_gyro_lowpass(low_pass_param &lowpass) {
    lowpass.alpha = 0.3;
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

    init_gyro_pid(gyro_pid);
    init_gyro_lowpass(gyro_low_pass);
}

/**
 * @brief 设置当前的 PID 参数
 * @param param 舵机 PD 参数
 * @author Cao Xin
 * @date 2025-07-06
 */
void set_control_param(control_param param) {
    dir_pid.kp = param.kp;
    dir_pid.kd = param.kd;

    gyro_pid.kp = param.gyro_p;
}

/**
 * @brief 计算并设置当前元素的 PID 参数
 * @author Cao Xin
 * @date 2025-07-06
 */
int calc_control_param(int &error) {
    // 3 * a, a = (max - det) / max, max = 60 - ImageStatus.TowPoint
    int siz = pid_map.size();

    int det_y = ImageStatus.OFFLine;
    int max_y = 60;
    float alpha_y = 1.0 * det_y / max_y;
    int idx_y = clip(std::floor(siz * alpha_y), 0, siz - 1);

    int det_x = abs(error);
    int max_x = 15;
    float alpha_x = 1.0 * det_x / max_x;
    int idx_x = clip(std::floor(siz * alpha_x), 0, siz - 1);
    int idx = pid_map[idx_y][idx_x];

    idx = clip(idx, 0, pid_left.size() - 1);

    control_param param = pid_left[idx];

    error += param.gain * error > 0 ? 1 : -1;

    // 圆环 PD
    if (ImageFlag.image_element_rings_flag) {
        pid_left.push_back(control_param(1.85, 3.20, -0.002, 4));
    }

    // 直道 PD
    if (ImageStatus.Road_type == Straight) {
        
    }

    error += error > 0 ? 1 : -1 * param.gain;
    set_control_param(param);
    return idx;
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
    // 误差
    static int error = 0;
    // 舵机占空比相较目标点的位置值
    static int servo_duty_det = 0;
    
    if (now != 0) {
        error = target - now;
        error = low_pass_filter(&dir_low_pass, error);
    }
    
    // 计算模糊 pid
    calc_control_param(error);

    int gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
    gyro_y = low_pass_filter(&gyro_low_pass, gyro_y);

    // 舵机 向左+ 向右-
    // servo_duty_det = bangbang_pid_solve(&dir_pid, error, 20, 30);
    servo_duty_det = pid_slove(&dir_pid, error);
    // 角加速度 向左- 向右+
    int gyro_y_det = pid_slove(&gyro_pid, gyro_y);
    servo_duty_det += gyro_y_det;
    
    set_servo_duty(get_servo_param().base_duty + servo_duty_det);
    
    set_left_speed(80);
    set_right_speed(80);
}