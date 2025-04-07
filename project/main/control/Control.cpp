#include <iostream>

#include "Motor.h"
#include "Servo.h"
pid_param dir_pid;

/**
 * @brief 初始化 PID 控制器
 * @param pid PID 控制器参数结构体指针
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void init_dir_pid(pid_param &pid){
    pid.kp = 0.2;
    pid.ki = 0.00;
    pid.kd = 0.0;

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

void control_init(){
    init_dir_pid(dir_pid);
}

/**
 * @brief 寻线到中线
 * @param now 当前中线位置
 * @param target 目标中线位置 一般为 width / 2
 * @return none
 * @author Cao Xin
 * @date 2025-04-06
 */
void to_center(int now, int target, int speed){
    int error = target - now;
    int servo_duty_det = 0;
    if(target != -1){
        servo_duty_det = pid_slove(&dir_pid, error);
    }
    set_servo_duty(get_servo_param().base_duty + servo_duty_det);
    
    // 计算差速比 10% pre 5 degree 
    int det = speed * (servo_duty_det / 5.0 * 0.1);
    set_left_speed(speed + det);
    set_right_speed(speed - det);

    if(SERVO_DEBUG){
        // std::cerr << "servo-target: " << target << ' ';
        // std::cerr << "servo-now: " << now << ' '; 
        std::cerr << "servo-duty-det: " << servo_duty_det << ' ';
        // std::cerr << "servo-error: " << error << ' ';
        std::cerr << "speed-det: " << det << '\n';
    }
}