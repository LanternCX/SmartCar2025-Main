#include <chrono>
#include <iostream>

 /**
 * @file Time.cpp
 * @brief 时间相关工具
 * @author Cao Xin
 * @date 2025-04-05
 */

int get_time() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}