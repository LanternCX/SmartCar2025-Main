#include "LowPass.h"
#include "image.hpp"
#include "zf_common_headfile.h"
#include <cstdlib>
#include <stdlib.h>

#include "Ramp.hpp"
#include "Debug.h"
#include "zf_device_imu_core.h"

low_pass_param ramp_low_pass;

void ramp_detect_init() {
    ramp_low_pass.alpha = 0.3;
    ramp_low_pass.last = 0;
}

void ramp_detect() {
    static int cnt_increase = 0;
    static int cnt_decrease = 0;
    static int cnt_euqal = 0;
    const int cnt_max = 4;
    int gyro_x = imu_get_raw(imu_file_path[GYRO_X_RAW]);
    gyro_x = low_pass_filter(&ramp_low_pass, gyro_x);
    cnt_increase += gyro_x > 300;
    cnt_decrease += gyro_x < -300;
    cnt_euqal += abs(gyro_x) <= 300;

    if (cnt_euqal > 2 * cnt_max) {
        cnt_euqal %= cnt_euqal;
        cnt_decrease = 0;
        cnt_increase = 0;
    }
    
    if (cnt_decrease >= cnt_max && ImageFlag.ramp_flag == 0) {
        ImageFlag.ramp_flag = 1;
        cnt_increase = 0;
    }
    if (cnt_increase >= cnt_max && ImageFlag.ramp_flag == 1) {
        ImageFlag.ramp_flag = 2;
        cnt_decrease = 0;
    }
    if (cnt_decrease >= cnt_max && ImageFlag.ramp_flag == 2) {
        ImageFlag.ramp_flag = 3;
    }
    if (std::abs(gyro_x) < 100 && ImageFlag.ramp_flag == 3) {
        ImageFlag.ramp_flag = 0;
    }
    cnt_decrease %= cnt_max;
    cnt_increase %= cnt_max;
    // debug(gyro_x, ImageFlag.ramp_flag);
}