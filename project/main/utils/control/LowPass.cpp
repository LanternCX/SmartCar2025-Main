#include "LowPass.h"

/**
 * @file LowPass.cpp
 * @brief 低通滤波相关操作
 * @author Cao Xin
 * @date 2025-04-05
 */

/**
 * @brief 低通滤波器
 * @param param 控制参数结构体
 * @param now 需要进行滤波的值
 * @return 滤波之后的数据
 * @author Cao Xin
 * @date 2025-04-05
 */
float low_pass_filter(low_pass_param * param, float now) {
    float alpha = param->alpha;
    float last = param->last;
    float output = alpha * now + (1 - alpha) * last;
    last = output;
    return output;
}