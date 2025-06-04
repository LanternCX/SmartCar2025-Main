/**
 * @brief 圆环相关逻辑
 * @author Cao Xin
 * @date 2025-06-02
 */

#pragma once

#include "Track.h"

void calc_ring_target(track_result &track, ElementType type);
bool calc_is_ring_in(const track_result track);