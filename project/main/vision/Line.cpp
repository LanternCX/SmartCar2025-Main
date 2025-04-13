#include <opencv2/core/hal/interface.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Vision.h"

/**
 * 扫线相关函数
 */

std::vector<int> leftX;  
std::vector<int> rightX;

/**
 * @brief 给图像绘制黑色边框
 * @param image 输入的图像
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void draw_border(cv::Mat& image) {
    if (image.empty()) return;
    // 画矩形框
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols-1, image.rows-1), cv::Scalar(0), 1);
}

/**
 * @brief 八邻域找左右边线
 * @param binary 输入的二值化图像
 * @param src 原始图像指针, 会向原始图像绘制扫线结果
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
line_result line_detection(cv::Mat binary, cv::Mat& src) {
    line_result res;

    // 确保图像尺寸匹配
    if (binary.size() != src.size()) {
        std::cout << "resize\n";
        cv::resize(src, src, binary.size());
    }

    draw_border(binary);

    uchar * frame = binary.data;
    // 图像尺寸
    int width = binary.cols;
    int height = binary.rows;
    auto getIdx = [&](int i, int j){
        return width * i + j;
    };
    
    // 用于存放左右边界点的 x 值，索引为 y 值
    // -1 表示未找到
    std::vector<int> leftX = std::vector<int>(height - 1, -1);
    std::vector<int> rightX = std::vector<int>(height - 1, -1);

    // 选取扫描起始行
    int startRow = height - 10;
    cv::line(src, cv::Point(0, startRow), cv::Point(width - 1, startRow), cv::Scalar(255, 255, 0), 2);
    cv::line(src, cv::Point(width / 2, startRow), cv::Point(width / 2, 1), cv::Scalar(255, 255, 0), 2);
        
    int midCol = width / 2;

    // 寻找左边起点：从中间向左扫描，找到黑白跳变点
    cv::Point leftStart = cv::Point(width, startRow);
    bool foundLeft = false;
    for (int i = midCol; i > 0; i--) {
        if (frame[getIdx(startRow, i)] == 255 && frame[getIdx(startRow, i - 1)] == 0) {
            leftStart = cv::Point(i, startRow);
            foundLeft = true;
            break;
        }
    }

    // 寻找右边起点：从中间向右扫描，找到黑白跳变点
    cv::Point rightStart = cv::Point(0, startRow);
    bool foundRight = false;
    for (int i = midCol; i < width - 1; i++) {
        if (frame[getIdx(startRow, i)] == 255 && frame[getIdx(startRow, i + 1)] == 0) {
            rightStart = cv::Point(i, startRow);
            foundRight = true;
            break;
        }
    }

    // 丢失左边线就向上寻找
    if(!foundLeft){
        for(int i = startRow; i > 0; i--){
            if(frame[getIdx(i, 0)] == 255 && frame[getIdx(i - 1, 0)] == 0){
                leftStart = cv::Point(0, i - 1);
                break;
            }
        }
    }

    // 丢失右边线就向上寻找
    if(!foundRight){
        for(int i = startRow; i > 0; i--){
            if(frame[getIdx(i, width - 1)] == 255 && frame[getIdx(i - 1, width - 1)] == 0){
                rightStart = cv::Point(width - 1, i - 1);
                break;
            }
        }
    }

    // 初始化扫描中心点
    cv::Point leftCenter = leftStart;
    cv::Point rightCenter = rightStart;
    leftX[leftCenter.y] = leftCenter.x;
    rightX[rightCenter.y] = rightCenter.x;

    // 八邻域偏移数组
    // 左边逆时针
    int seedsL[8][2] = {
        {0, 1}, {-1,1}, {-1,0}, {-1, -1},
        {0, -1}, {1, -1}, {1, 0}, {1, 1}
    };
    // 右边顺时针
    int seedsR[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
    };

    int maxIter = 1000;
    while (maxIter--) {
        // 左边扫描
        std::vector<cv::Point> leftCandidates;
        for (int i = 0; i < 8; i++) {
            // 当前八邻域检查的点
            int nowX = leftCenter.x + seedsL[i][0];
            int nowY = leftCenter.y + seedsL[i][1];
            if (nowX < 0 || nowX >= width || nowY < 0 || nowY >= height){
                continue;
            }

            // 下一个八邻域检查的点
            int nextIdx = (i + 1) % 8;
            int nextX = leftCenter.x + seedsL[nextIdx][0];
            int nextY = leftCenter.y + seedsL[nextIdx][1];
            if (nextX < 0 || nextX >= width || nextY < 0 || nextY >= height){
                continue;
            }

            // 检查黑白跳变
            if (frame[getIdx(nowY, nowX)] == 0 && frame[getIdx(nextY, nextX)] == 255) {
                leftCandidates.push_back(cv::Point(nowX, nowY));
            }
        }
        // 选候选点中最靠上的点
        if (!leftCandidates.empty()) {
            cv::Point best = leftCandidates[0];
            for (auto pt : leftCandidates) {
                if (pt.y < best.y)
                    best = pt;
            }
            leftCenter = best;
            // 存储
            leftX[leftCenter.y] = leftCenter.x;
        }

        // 右边扫描
        std::vector<cv::Point> rightCandidates;
        for (int i = 0; i < 8; i++) {
            int nowX = rightCenter.x + seedsR[i][0];
            int nowY = rightCenter.y + seedsR[i][1];
            if (nowX < 0 || nowX >= width || nowY < 0 || nowY >= height) {
                continue;
            }

            int nextIdx = (i + 1) % 8;
            int nextX = rightCenter.x + seedsR[nextIdx][0];
            int nextY = rightCenter.y + seedsR[nextIdx][1];
            if (nextX < 0 || nextX >= width || nextY < 0 || nextY >= height) {
                continue;
            }

            if (frame[getIdx(nowY, nowX)] == 0 && frame[getIdx(nextY, nextX)] == 255) {
                rightCandidates.push_back(cv::Point(nowX, nowY));
            }
        }
        // 选候选点中最靠上的点
        if (!rightCandidates.empty()) {
            cv::Point best = rightCandidates[0];
            for (auto pt : rightCandidates) {
                if (pt.y < best.y)
                    best = pt;
            }
            rightCenter = best;
            // 存储
            rightX[rightCenter.y] = rightCenter.x;
        }

        // 判断左右边线是否相遇
        if (abs(leftCenter.x - rightCenter.x) < 2 && abs(leftCenter.y - rightCenter.y) < 2) {
            break;
        }
    }

    // 处理出中线
    std::vector<int> center = std::vector<int>(height - 1, -1);
    for(int y = 0; y < height; y++){
        center[y] = (rightX[y] + leftX[y]) / 2;
    }

    if(VISION_DEBUG){
        // 画出边线
        for (int y = 0; y < height; y++) {
            if (leftX[y] != -1) {
                cv::circle(src, cv::Point(leftX[y], y), 2, cv::Scalar(0, 0, 255), -1);
            }
            if (rightX[y] != -1) {
                cv::circle(src, cv::Point(rightX[y], y), 2, cv::Scalar(255, 0, 0), -1);
            }
            if (leftX[y] != -1 && rightX[y] != -1) {
                cv::circle(src, cv::Point((rightX[y] + leftX[y]) / 2, y), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
    }

    res = {leftX, rightX, center};
    // 返回中线
    return res;
}