#include <algorithm>
#include <iostream>

#include "Motor.h"
#include "PID.h"
#include "zf_common_headfile.h"
#include "zf_device_ips200_fb.h"

struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
pid_param_t leftPID;
pid_param_t rightPID;

static int leftDuty = 10;
static int rightDuty = 10;
static int cnt = 0;
int leftOut[100];
int rightOut[100];
void initPID(pid_param_t &pid){
    pid.kp = 0.03;
    pid.ki = 0.021;
    pid.kd = 0.021;
    pid.p_max = 30.0;
    pid.i_max = 30.0;
    pid.d_max = 30.0;
    pid.out_p = 0.0;
    pid.out_i = 0.0;
    pid.out_d = 0.0;
    pid.pre_error = 0.0;
    pid.pre_pre_error = 0.0;
}

void motorInit(){
    // 获取PWM设备信息
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);

    initPID(leftPID);
    initPID(rightPID);
}

void leftMotorRun(int duty, int dir){
    duty = std::min(duty, 150);
    gpio_set_level(MOTOR1_DIR, dir);
    pwm_set_duty(MOTOR1_PWM, duty * (MOTOR1_PWM_DUTY_MAX / 100));
}

void rightMotorRun(int duty, int dir){
    duty = std::min(duty, 150);
    gpio_set_level(MOTOR2_DIR, dir);
    pwm_set_duty(MOTOR2_PWM, duty * (MOTOR2_PWM_DUTY_MAX / 100));
}

pid_param_t getPID(){
    return leftPID;
}

void setLeftSpeed(int speed){
    float now = encoder_get_count(ENCODER_2);
    float error = speed - now;
    float det = increment_pid_solve(&leftPID, error);

    leftDuty += det;

    cnt++;
    cnt %= 100;

    std::cout << "duty: " <<  leftDuty << " now: " << now << " error: " << error << '\n';
    ips200_show_string(100 + 10, 160, std::string("error: " + std::to_string(error)).c_str());
    ips200_show_string(100 + 10, 180, std::string("det: " + std::to_string(det)).c_str());
    ips200_show_string(100 + 10, 200, std::string("target: " + std::to_string(speed)).c_str());

    leftOut[cnt] = now;
    for(int i = 0; i < 100; i++){
        ips200_draw_point((10 + i), 320 - leftOut[i] / 3, 0x00FF);
        ips200_draw_point((10 + i), 320 - speed / 3, 0x00FF);
    }
    leftMotorRun(abs(leftDuty), leftDuty > 0 ? 1 : -1);
}

void setRightSpeed(int speed){
    float now = encoder_get_count(ENCODER_1);
    float error = speed - now;
    float det = increment_pid_solve(&rightPID, error);

    std::cout << "duty: " <<  rightDuty << " now: " << now << " error: " << error << '\n';
    ips200_show_string(100 + 10, 160, std::string("error: " + std::to_string(error)).c_str());
    ips200_show_string(100 + 10, 180, std::string("det: " + std::to_string(det)).c_str());
    ips200_show_string(100 + 10, 200, std::string("target: " + std::to_string(speed)).c_str());

    rightOut[cnt] = now;
    for(int i = 0; i < 100; i++){
        ips200_draw_point((10 + i), 220 - rightOut[i] / 3, 0xF800);
        ips200_draw_point((10 + i), 220 - speed / 3, 0xF800);
    }
    rightMotorRun(abs(rightDuty), rightDuty > 0 ? 1 : -1);
}
