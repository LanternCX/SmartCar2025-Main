#include <iostream>

#include "LowPass.h"
#include "Motor.h"
#include "Servo.h"
#include "Control.h"
#include "Debug.h"

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

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_dir_pid(pid_param &pid){
    // v 40 p 0.10 d 0.
    // lowpass 0.5 p 0.2 d 0.4
    // v 60 - 10 lowpass 0.5 0.1 p 0.6 d 0.4 line -10
    // v 70 - 20 lowpass 0.5 0.1 p 0.4 d 0.4 line -20
    // v 80 - 30 lowpass 0.5 0.1 p 0.4 d 0.6 line -70
    // v 100, 70 lowpass 0.6 0.1 p 0.3 d 0.95 line -50 no det
    pid.kp = 0.75;
    pid.ki = 0.00;
    pid.kd = 0.80;

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
 * @brief 初始化位置低通滤波器
 * @param lowpass 低通滤波器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_dir_lowpass(low_pass_param &lowpass){
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
void control_init(int line_speed, int curve_speed){
    init_dir_pid(dir_pid);
    init_dir_lowpass(dir_low_pass);
    speed.line_speed = line_speed;
    speed.curve_speed = curve_speed;
}

/**
 * @brief 寻线到中线
 * @param now 当前中线位置
 * @param target 目标中线位置 一般为 width / 2
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void to_center(int now, int target){
    static int error = 0;
    static int servo_duty_det = 0;
    if(now != -1){
        error = target - now;
        error = low_pass_filter(&dir_low_pass, error);
        servo_duty_det = pid_slove(&dir_pid, error);
    }
    debug(servo_duty_det);
    set_servo_duty(get_servo_param().base_duty + servo_duty_det);
    
    // 计算差速比 10% pre 5 degree 
    int det = speed.current * (servo_duty_det / 5.0 * 0.1);
    det = 0;
    set_left_speed(speed.current + det);
    set_right_speed(speed.current - det);

    if(SERVO_DEBUG){
        // std::cerr << "servo-target: " << target << ' ';
        std::cerr << "servo-now: " << now << ' '; 
        // std::cerr << "servo-duty-det: " << servo_duty_det << ' ';
        // std::cerr << "servo-error: " << error << ' ';
        // std::cerr << "speed-det: " << det << '\n';
    }
}

/**
 * @brief 速度决策状态机切换
 * @param type 当前赛道类型枚举值
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
// void set_statue(ElementType type){
    
//     if(type == LINE){
//         dir_low_pass.alpha = 0.6;
//         speed.current = speed.line_speed;
//         if(SERVO_DEBUG){
//             std::cerr << "servo-target: " << "LINE" << '\n';
//         }
//     }
//     if(type == L_CURVE || type == R_CURVE){
//         dir_low_pass.alpha = 0.6;
//         speed.current = speed.curve_speed;
//         if(SERVO_DEBUG){
//             std::cerr << "servo-target: " << "CURVE" << '\n';
//         }
//     }
// }