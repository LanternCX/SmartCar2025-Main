#include "Color.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "image.hpp"
#include "Debug.h"
#include "Display.h"

#include "Main.hpp"
#include "zf_driver_gpio.h"

using namespace cv;
using namespace std;
/**
 * @brief 裁剪去除图像的上半部分
 * @param src RGB 图像
 */
cv::Mat crop_bottom_half(const cv::Mat& src) {
    int height = src.rows;
    int width = src.cols;
    // 定义从中间到底部的矩形区域
    cv::Rect roi(0, height / 2, width, height - height / 2);
    return src(roi).clone(); // 返回裁剪图像副本
}


/**
 * @brief 通用颜色识别函数：识别图像中指定 HSV 范围的色块
 * 
 * @param src 输入图像（BGR格式）
 * @param low 输出图像（绘制了检测结果）
 * @param low HSV下限，例如 Scalar(20, 100, 100)
 * @param high HSV上限，例如 Scalar(30, 255, 255)
 * @return vector<Rect> 返回识别到的色块矩形框
 */
vector<Rect> color_detect(const Mat& src, Mat& dst, const Scalar& low, const Scalar& high) {
    vector<Rect> result;

    Mat half = crop_bottom_half(src);
    // 1. 转换为 HSV 空间
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);

    // 2. 颜色掩膜
    Mat mask;
    inRange(hsv, low, high, mask);

    // 3. 形态学操作，消除噪声，填补空洞
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);   // 去小点
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);  // 闭合边界

    // 4. 查找轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 5. 筛选并绘制
    // 拷贝原图
    dst = src.clone();
    // 允许的最小红箱与原图的比例
    float scale = 0.05;
    int min_siz = (src.cols * src.rows) * (scale * scale);
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        // debug(area, min_siz);
        // 可调阈值，去除小轮廓
        if (area > min_siz) {
            Rect rect = boundingRect(contour);
            // 绘制结果
            rectangle(dst, rect, Scalar(0, 255, 0), 1);
            result.push_back(rect);
        }
    }

    return result;
}


/**
 * @brief 找到面积最大的方框并计算中心点的 x 坐标
 * @param rect 矩形列表
 * @return 最大矩形中心点的 x 坐标，若无矩形则返回 -1
 */
int get_center_x(const vector<Rect>& rect) {
    if (rect.empty()) return -1; // 没有矩形时返回 -1

    // 找出最大面积的矩形
    Rect max_rect = rect[0];
    double max_area = max_rect.area();

    for (const Rect& r : rect) {
        double area = r.area();
        if (area > max_area){
            max_area = area;
            max_rect = r;
        }
    }

    // 计算中心点的 x 坐标
    int center_x = max_rect.x + max_rect.width / 2;
    return center_x;
}

/**
 * @brief 识别红色色块
 * @param src RGB 图像
 * @param dst 识别结果
 */
vector<Rect> detect_red(const Mat& src, Mat& dst) {
    Mat half = crop_bottom_half(src);
    Mat temp1, temp2;
    vector<Rect> rects1 = color_detect(half, temp1, Scalar(0, 70, 50), Scalar(10, 255, 255));
    vector<Rect> rects2 = color_detect(half, temp2, Scalar(170, 70, 50), Scalar(180, 255, 255));

    dst = half.clone();
    for (const auto& r : rects1) rectangle(dst, r, Scalar(0, 255, 0), 2);
    for (const auto& r : rects2) rectangle(dst, r, Scalar(0, 255, 0), 2);

    vector<Rect> all = rects1;
    all.insert(all.end(), rects2.begin(), rects2.end());
    return all;
}

/**
 * @brief 基于红箱检测的圆环判断
 * @param frame RGB 图像
 */
void ring_judge(cv::Mat frame) {
    if (ImageFlag.image_element_rings_flag) {
        gpio_set_level(BEEP, 0x1);
    } else {
        gpio_set_level(BEEP, 0x0);
    }
    vector<Rect> red = detect_red(frame, frame);

    ImageFlag.is_red = red.size() != 0;
    // resize(frame, frame, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
    // draw_rgb_img(frame);
    // debug(red.size());
    // 如果状态来源是红箱进行滤除防止误判
    if (ImageFlag.image_element_rings_flag == 1 && red.empty() && ImageFlag.stat_from == 2) {
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.image_element_rings = 0;
        ImageFlag.ring_big_small = 0;
        ImageStatus.Road_type = Normol;
        ImageFlag.stat_from = 0;
        return;
    }

    // 状态来源是红箱那么结合红箱状态进行出环判断
    if (ImageFlag.image_element_rings_flag == 10 && red.empty() && ImageFlag.stat_from == 2) {
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.image_element_rings = 0;
        ImageFlag.ring_big_small = 0;
        ImageStatus.Road_type = Normol;
        ImageFlag.is_flip = false;
        ImageFlag.stat_from = 0;
        return;
    }
    // 状态来源不是红箱直接出环
    if (ImageFlag.image_element_rings_flag == 10 && red.empty() && ImageFlag.stat_from == 1) {
        ImageFlag.image_element_rings_flag = 0;
        ImageFlag.image_element_rings = 0;
        ImageFlag.ring_big_small = 0;
        ImageStatus.Road_type = Normol;
        ImageFlag.is_flip = false;
        ImageFlag.stat_from = 0;
        return;
    }
    // 识别不到红箱就直接退出
    if (red.empty()) {
        return;
    }

    int x = get_center_x(red);
    if (x > ImageStatus.MiddleLine && ImageFlag.image_element_rings_flag == 0) {
        // debug("Left Ring");
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 5;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = LeftCirque;
        ImageFlag.stat_from = 2;
    } else if (x < ImageStatus.MiddleLine && ImageFlag.image_element_rings_flag == 0) {
        // debug("Right Ring");
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 5;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = LeftCirque;
        ImageFlag.is_flip = true;
        ImageFlag.stat_from = 2;
    }
}