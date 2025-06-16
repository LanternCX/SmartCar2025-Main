/**
 * @brief 赛道状态单例，主要维护前几帧率的赛道的各种状态提供给状态机使用
 * @author Cao Xin
 * @date 2025-05-31
 */
#pragma once

#include "Track.h"
void init_state();
void change_type_count(ElementType now);
ElementType get_track_type(void);
void calc_track_type(void);