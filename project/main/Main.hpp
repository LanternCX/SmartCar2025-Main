#pragma once
#include <chrono>
#define BEEP "/dev/zf_driver_gpio_beep"

const auto timer_interval = std::chrono::milliseconds(20);