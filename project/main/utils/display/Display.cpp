#include <opencv2/core/mat.hpp>
#include "zf_common_headfile.h"
#include "Display.h"

/**
 * 屏幕操作相关函数
 */

/**
 * @brief 在屏幕绘制彩色图像
 * @param frame 图像
 * @return None
 * @author Cao Xin
 * @date 2025-04-04
 */
void draw_rgb_img(cv::Mat frame){
    int width = std::min(frame.cols, IMG_WIDTH / IMG_SCALE);
    int height = std::min(frame.rows, IMG_HEIGHT / IMG_SCALE);

    const uchar* raw = frame.data;
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j++) {
            // calc index
            int idx = (i * frame.cols + j) * 3;

            // read bgr
            uint8_t b = raw[idx];
            uint8_t g = raw[idx + 1];
            uint8_t r = raw[idx + 2];

            // covert to RGB565
            uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            
            ips200_draw_point(j, i, color);
        }
    }
    for (int i = 1; i < height; i += 2) {
        for (int j = 0; j < width; j++) {
            // calc index
            int idx = (i * frame.cols + j) * 3;

            // read bgr
            uint8_t b = raw[idx];
            uint8_t g = raw[idx + 1];
            uint8_t r = raw[idx + 2];

            // covert to RGB565
            uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            
            ips200_draw_point(j, i, color);
        }
    }
}

/**
 * @brief 在屏幕绘制灰度图像
 * @param frame 图像
 * @return None
 * @author Cao Xin
 * @date 2025-04-04
 */
void draw_gray_img(cv::Mat frame){
    int width = std::min(frame.cols, IMG_WIDTH / IMG_SCALE);
    int height = std::min(frame.rows, IMG_HEIGHT / IMG_SCALE);

    const uchar* raw = frame.data;
    for (int i = 0; i < height; i += 2) {
        for (int j = 0; j < width; j++) {
            // calc index
            int idx = (i * frame.cols + j);

            // read bgr
            uint8_t val = raw[idx];

            // covert to RGB565
            uint16_t color = ((val >> 3) << 11) | ((val >> 2) << 5) | (val >> 3);
            
            ips200_draw_point(j, i, color);
        }
    }
    for (int i = 1; i < height; i += 2) {
        for (int j = 0; j < width; j++) {
            // calc index
            int idx = (i * frame.cols + j);

            // read bgr
            uint8_t val = raw[idx];

            // covert to RGB565
            uint16_t color = ((val >> 3) << 11) | ((val >> 2) << 5) | (val >> 3);
            
            ips200_draw_point(j, i, color);
        }
    }
}