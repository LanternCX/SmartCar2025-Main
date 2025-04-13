#include <iostream>

#include "LowPass.h"
#include "Motor.h"
#include "Servo.h"
#include "Vision.h"
#include "Control.h"

pid_param dir_pid;
low_pass_param dir_low_pass;
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
    pid.kp = 0.40;
    pid.ki = 0.00;
    pid.kd = 0.6;

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

void init_dir_lowpass(low_pass_param &lowpass){
    lowpass.alpha = 0.5;
}

void control_init(int v){
    init_dir_pid(dir_pid);
    init_dir_lowpass(dir_low_pass);
    speed.line_speed = v;
    speed.curve_speed = v - 30;
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
    int error = target - now;
    error = low_pass_filter(&dir_low_pass, error);
    int servo_duty_det = 0;
    if(target != -1){
        servo_duty_det = pid_slove(&dir_pid, error);
    }
    set_servo_duty(get_servo_param().base_duty + servo_duty_det);
    
    // 计算差速比 10% pre 5 degree 
    int det = speed.current * (servo_duty_det / 5.0 * 0.1);
    // det = 0;
    set_left_speed(speed.current + det);
    set_right_speed(speed.current - det);

    if(SERVO_DEBUG){
        // std::cerr << "servo-target: " << target << ' ';
        std::cerr << "servo-now: " << now << ' '; 
        std::cerr << "servo-duty-det: " << servo_duty_det << ' ';
        std::cerr << "servo-error: " << error << ' ';
        std::cerr << "speed-det: " << det << '\n';
    }
}

void set_statue(Type type){
    if(type == LINE){
        dir_low_pass.alpha = 0.1;
        speed.current = speed.line_speed;
    }
    if(type == CURVE){
        dir_low_pass.alpha = 0.5;
        speed.current = speed.curve_speed;
    }
}