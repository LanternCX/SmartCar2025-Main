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

low_pass_param servo_low_pass;
// 当前速度参数
speed_param speed;

std::vector<control_param> pid_left;
std::vector<control_param> pid_right;
std::vector<control_param> pid_left_ring;
std::vector<control_param> pid_right_ring;
control_param pid_ramp;

std::vector<std::vector<int>> pid_map;

const int max_speed = 100;
const int base_duty = 2400;

/**
 * @brief 模糊 PID 初始化
 * @return none
 * @author Cao Xin
 * @date 2025-07-06
 */
void init_fuzzy_pid() {
    pid_left.push_back(control_param(1.00, 3.20, 0, 0.005, 15));
    pid_left.push_back(control_param(1.20, 3.20, 0.001, 0.000, 20));
    pid_left.push_back(control_param(1.20, 3.20, 0.002, 0.000, 20));
    pid_left.push_back(control_param(1.20, 3.20, 0.003, 0.000, 20));

    pid_right.push_back(control_param(1.00, 3.20, 0, 0.005, 15));
    pid_right.push_back(control_param(1.40, 3.20, 0.002, 0.000, 20));
    pid_right.push_back(control_param(1.40, 3.20, 0.003, 0.000, 20));
    pid_right.push_back(control_param(1.40, 3.20, 0.004, 0.000, 20));

    pid_right_ring = {
        control_param(1.00, 4.00, 0.001, 0.000, 22), 
        control_param(1.00, 3.20, 0.004, 0.002, 24)
    };
    pid_left_ring = {
        control_param(1.00, 4.00, 0.001, 0.000, 22), 
        control_param(1.00, 3.20, 0.002, 0.003, 24)
    };

    pid_ramp = control_param(0.01, 0.00, 0, 0.0, 40);

    pid_map = {
        { 0, 0, 0, 0, },
        { 1, 1, 2, 2, },
        { 2, 2, 3, 3, },
        { 3, 3, 3, 3, },
    };
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
    lowpass.alpha = 0.5;
}

void init_servo_low_pass(low_pass_param &lowpass) {
    lowpass.alpha = 0.95;    
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
    init_servo_low_pass(servo_low_pass);
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
    ImageStatus.TowPoint = param.tow_point;
}

/**
 * @brief 通过舵机角度计算出差速
 * @param angle 舵机角度
 * @param v_center 目标线速度
 * @param v_in 内轮速度
 * @param v_out 外轮速度
 * @return none
 * @author Cao Xin
 * @date 2025-07-13
 */
void calc_speed_det(const int & angle, const int & v_center, int & v_in, int & v_out) {
    // C车差速: https://blog.csdn.net/weixin_43906861/article/details/123336728

    double angle_deg = std::abs(angle);
    double angle_rad = angle_deg * M_PI / 180.0;  // 角度转弧度

    const float T = 32.0;
    const float M = 4.0;
    const float L4 = 15.0;
    const float W = 155.0;
    const float L = 200.0;
    
    float L2 = T * std::sin(angle_rad);
    float L1 = M * std::cos(angle_rad);
    float L3 = M - (L1 - L2);

    // debug(L1, L2, L3);

    // 虽然理想状态下 α > β 但是由于 C 车模型限制可以认为 α = β
    float tan = L3 / std::sqrt(L4 * L4 - L3 * L3);

    debug(L1, L2, L3, tan);
    
    float R = L / tan;
    v_in = (R - W / 2) / R * v_center;
    v_out = (R + W / 2) / R * v_center;
}

/**
 * @brief 计算并设置当前元素的 PID 参数
 * @author Cao Xin
 * @date 2025-07-06
 */
control_param calc_control_param(int error) {
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

    if (idx == 0) {

    }

    // 圆环参数
    if (ImageFlag.image_element_rings_flag) {
        int idx = !(ImageFlag.image_element_rings_flag == 5 || ImageFlag.image_element_rings_flag == 6);
        if (ImageFlag.is_flip == 0) {
            set_control_param(pid_left_ring[idx]);
            return pid_left_ring[idx];
        }
        if (ImageFlag.is_flip == 1) {
            set_control_param(pid_right_ring[idx]);
            return pid_right_ring[idx];
        }
    }

    // 坡道参数
    if (ImageFlag.ramp_flag) {
        set_control_param(pid_ramp);
        return pid_ramp;
    }
    
    if (error >= 0) {
        set_control_param(pid_left[idx]);
        return pid_left[idx];
    } else {
        set_control_param(pid_right[idx]);
        return pid_right[idx];
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
    // 误差
    static int error = 0;
    // 舵机占空比相较目标点的位置值
    static float servo_duty_det = 0;
    // if (ImageFlag.image_element_rings_flag == 6) {
    //     error *= 0.95;
    // } else if (now != 0) {
    //     error = target - now;
    //     error = low_pass_filter(&dir_low_pass, error);
    // }

    if (now != 0) {
        error = target - now;
        error = low_pass_filter(&dir_low_pass, error);
    }

    if (ImageFlag.image_element_rings_flag == 5 || ImageFlag.image_element_rings_flag == 6) {
        // error *= 0.95;
    }
    
    // 计算模糊 pid
    control_param param = calc_control_param(error);

    int gyro_y = imu_get_raw(imu_file_path[GYRO_Y_RAW]);
    gyro_y = low_pass_filter(&gyro_low_pass, gyro_y);

    // 舵机 向左+ 向右-
    // servo_duty_det = bangbang_pid_solve(&dir_pid, error, 20, 30);
    servo_duty_det = pid_slove(&dir_pid, error);
    // 角加速度 向左- 向右+
    float gyro_y_det = pid_slove(&gyro_pid, gyro_y);
    servo_duty_det += gyro_y_det;

    servo_duty_det += abs(error) * error * param.kp2;

    if (abs(servo_duty_det) > 14) {
        servo_duty_det = servo_duty_det > 0 ? 14 : -14;
    }

    if (abs(servo_duty_det) < 1) {
        servo_duty_det = 0;
    }
    servo_duty_det = low_pass_filter(&servo_low_pass, servo_duty_det);

    set_speed(max_speed, servo_duty_det, base_duty);
    set_servo_duty(get_servo_param().base_duty + servo_duty_det);
}