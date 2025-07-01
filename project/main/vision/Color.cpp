#include "Color.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "image.hpp"
#include "Debug.h"

using namespace cv;
using namespace std;

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
    Mat hsv, mask;

    int height = min(src.cols, src.rows);
    int width = max(src.cols, src.rows);
    double min_area = (height * 0.03) * (width * 0.03);
    src.copyTo(dst);

    // 1. BGR → HSV
    cvtColor(src, hsv, COLOR_BGR2HSV);

    // 2. 掩码生成
    inRange(hsv, low, high, mask);

    // 3. 去噪（形态学操作）
    // Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    // morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // morphologyEx(mask, mask, MORPH_DILATE, kernel);

    // 4. 查找轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 5. 计算边界框
    vector<Rect> colorRects;
    for (const vector<cv::Point> &contour : contours) {
        double area = contourArea(contour);
        if (area > min_area) {
            Rect rect = boundingRect(contour);
            colorRects.push_back(rect);
            rectangle(dst, rect, Scalar(0, 255, 255), 2); // 可调整颜色
        }
    }

    return colorRects;
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
        if (area > max_area) {
            max_area = area;
            max_rect = r;
        }
    }

    // 计算中心点的 x 坐标
    int center_x = max_rect.x + max_rect.width / 2;
    return center_x;
}

void ring_judge(cv::Mat frame) {
    std::vector<cv::Rect> red = colorDetect(frame, frame, cv::Scalar(0, 48, 48), cv::Scalar(30, 255, 255));
    if (red.empty()) {
        return;
    }
    
    int x = get_center_x(red);
    if (x < ImageStatus.MiddleLine && ImageFlag.image_element_rings_flag == 0) {
        debug("Left Ring");
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = LeftCirque;
    } else if (x > ImageStatus.MiddleLine && ImageFlag.image_element_rings_flag == 0) {
        debug("Right Ring");
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small = 1;
        ImageStatus.Road_type = RightCirque;
    }
}