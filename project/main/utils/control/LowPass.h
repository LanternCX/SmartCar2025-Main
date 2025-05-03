#ifndef _LOW_PASS_H_
#define _LOW_PASS_H_

/**
 * @file LowPass.h
 * @brief 低通滤波相关操作头文具ian
 * @author Cao Xin
 * @date 2025-04-05
 */

typedef struct {
    /**
     * 上一次的输入值
     */
    float last;
    /**
     * 滤波系数
     */
    float alpha;
} low_pass_param;
float low_pass_filter(low_pass_param * param, float now);
#endif