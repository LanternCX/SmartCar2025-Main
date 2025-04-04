#include <iostream>

#include "Motor.h"
#include "PID.h"
#include "zf_common_headfile.h"

struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
pid_param_t pid;
static int duty = 10;

void motorInit(){
    // 获取PWM设备信息
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);
}

void leftMotorRun(int duty, int dir){
    gpio_set_level(MOTOR1_DIR, dir);
    pwm_set_duty(MOTOR1_PWM, duty * (MOTOR1_PWM_DUTY_MAX / 100));
}

void rightMotorRun(int duty, int dir){
    gpio_set_level(MOTOR2_DIR, dir);
    pwm_set_duty(MOTOR2_PWM, duty * (MOTOR2_PWM_DUTY_MAX / 100));
}

void initPID(){
    pid.kp = 0.03;
    pid.ki = 0.02;
    pid.kd = 0.02;
    pid.p_max = 30.0;
    pid.i_max = 30.0;
    pid.d_max = 30.0;
    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;
    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}
void setPID(float kp, float ki, float kd){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
}

pid_param_t getPID(){
    return pid;
}

void setSpeed(int speed){
    float now = encoder_get_count(ENCODER_2);
    float error = speed - now;
    float det = increment_pid_solve(&pid, error);

    duty += det;

    std::cout << "duty: " <<  duty << " now: " << now << " error: " << error << '\n';
    ips200_show_string(10, 160, std::string("error: " + std::to_string(error)).c_str());
    ips200_show_string(10, 180, std::string("det: " + std::to_string(det)).c_str());
    ips200_show_string(10, 200, std::string("target: " + std::to_string(speed)).c_str());
    leftMotorRun(abs(duty), duty > 0 ? 1 : -1);
}
