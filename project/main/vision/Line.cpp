#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

#include "Line.h"
#include "Vision.h"
#include "Math.h"
#include "Debug.h"

/**
 * @file Line.cpp
 * @brief 边线处理相关操作
 * @author Cao Xin
 * @date 2025-05-03
 */

/**
 * @brief 点集三角滤波
 * @param pts_in 输入点集
 * @param pts_out 输出点集
 * @param kernel 滤波核大小 奇数
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void filter_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel) {
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    int num = pts_in.size();
    pts_out.clear();
    for (int i = half; i < num - half; i++) {
        double sum_x = 0;
        double sum_y = 0;
        double weight_sum = 0;
        for (int j = -half; j <= half; j++) {
            int index = clip(i + j, 0, num - 1);
            int weight = half + 1 - std::abs(j);
            sum_x += static_cast<double>(pts_in[index].x) * weight;
            sum_y += static_cast<double>(pts_in[index].y) * weight;
            weight_sum += weight;
        }
        int new_x = static_cast<int>(std::round(sum_x / weight_sum));
        int new_y = static_cast<int>(std::round(sum_y / weight_sum));
        pts_out.push_back(cv::Point(new_x, new_y));
    }
}

/**
 * @brief 点集等距采样
 * @param pts_in 输入点集
 * @param pts_out 输出点集
 * @param dist 采样距离
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist) {
    if (pts_in.empty()) {
        pts_out.clear();
        return;
    }

    pts_out.clear();
    pts_out.push_back(pts_in[0]);
    float remain = 0.0f;
    for (size_t i = 0; i < pts_in.size() - 1; ++i) {
        cv::Point p0 = pts_in[i];
        cv::Point p1 = pts_in[i + 1];
        float dx = static_cast<float>(p1.x - p0.x);
        float dy = static_cast<float>(p1.y - p0.y);
        float segment_length = std::sqrt(dx * dx + dy * dy);
        dx /= segment_length;
        dy /= segment_length;

        while (remain < segment_length) {
            float next_x = static_cast<float>(p0.x) + dx * remain;
            float next_y = static_cast<float>(p0.y) + dy * remain;
            pts_out.push_back(cv::Point(std::round(next_x), std::round(next_y)));
            segment_length -= remain;
            remain = dist;
        }
        remain -= segment_length;
    }
    // Ensure the last point is included
    if (!pts_in.empty() && (pts_out.empty() || pts_out.back() != pts_in.back())) {
        pts_out.push_back(pts_in.back());
    }
}

/**
 * @brief 局部角度变化率
 * @param pts_in 输入点集
 * @param angle_out 计算得的变化率集
 * @param angle_dist 斜率采样距离
 * @param sample_dist 等距采样距离
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void get_line_slope(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int angle_dist, int sample_dist) {
    const float epsilon = 1e-6f;
    int dist = angle_dist / sample_dist;
    int num = pts_in.size();
    angle_out.resize(num);

    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0.0f;
            continue;
        }

        cv::Point p_prev = pts_in[clip(i - dist, 0, num - 1)];
        cv::Point p_curr = pts_in[i];
        cv::Point p_next = pts_in[clip(i + dist, 0, num - 1)];

        float dx1 = static_cast<float>(p_curr.x - p_prev.x);
        float dy1 = static_cast<float>(p_curr.y - p_prev.y);
        float dn1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

        float dx2 = static_cast<float>(p_next.x - p_curr.x);
        float dy2 = static_cast<float>(p_next.y - p_curr.y);
        float dn2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

        if (dn1 < epsilon || dn2 < epsilon) {
            angle_out[i] = 0.0f;
            continue;
        }

        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;

        angle_out[i] = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

/**
 * @brief 角度变化率非极大值抑制
 * @param angle_in 输入变化率集
 * @param angle_out 抑制后的变化率集
 * @param kernel 滤波核大小
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void  filter_line_slope(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel) {
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    int num = angle_in.size();
    angle_out.resize(num);
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            int index = clip(i + j, 0, num - 1);
            if (std::abs(angle_in[index]) > std::abs(angle_out[i])) {
                angle_out[i] = 0.0f;
                break;
            }
        }
    }
}

/**
 * @brief 从角度变化率重建点集
 * @param angle_in 输入的角度变化率
 * @param pts_out 输出点集
 * @param dist 每步移动的距离
 * @param origin 初始点坐标
 * @return none
 * @author Cao Xin
 * @date 2025-05-04
 */
void rebuild_line(const std::vector<float>& angle_in, std::vector<cv::Point>& pts_out, int dist, cv::Point origin) {
    int num = angle_in.size();
    pts_out.resize(num);
    pts_out[0] = origin;

    float angle = 0.0f;

    for (int i = 1; i < num; i++) {
        angle += angle_in[i];
        float dx = std::cos(angle) * dist;
        float dy = std::sin(angle) * dist;
        pts_out[i] = cv::Point(
            static_cast<int>(std::round(pts_out[i - 1].x + dx)),
            static_cast<int>(std::round(pts_out[i - 1].y + dy))
        );
    }
}

/**
 * @brief 重新采样到 line[y] = x 边线表示
 * @param line 边线点集
 * @param size 图像大小
 */
std::vector<int> trans_line(const std::vector<cv::Point> &line, const cv::Size &size) {
    int height = size.height;
    int width = size.width;
    std::vector<int> res(height + 1, -1);
    int min_y = height;
    for (cv::Point pts : line) {
        if (res[pts.y] != -1 || pts.y > min_y) {
            continue;
        }
        res[pts.y] = pts.x;
        min_y = std::min(pts.y, min_y);
    }
    std::vector<int> temp;
    for (int x : res) {
        if (x == -1) {
            continue;
        }
        temp.push_back(x);
    }
    return temp;
}

/**
 * @brief 凹槽数量判断
 * @param line 边线
 * @param threshold 角点阈值
 */
int get_solt_count(const std::vector<int> &line) {
    int threshold = 30.0;
    int n = line.size();
    int dist = 15;
    int cnt = 0;
    for (int i = 0; i < n / 2 - dist; i++) {
        int next = i + dist;
        if (cnt % 2 == 0) {
            cnt += line[i] - line[next] > threshold;
        }
        if (cnt % 2 == 1) {
            cnt += line[next] - line[i] > threshold;
        }
    }
    return cnt / 2;
}

/**
 * @brief 角点数量判断
 * @param line 边线
 * @return pair(向外角点, 向内角点) 
 */
std::pair<int, int> get_corner_count(const std::vector<int> &line) {
    int threshold = 30;
    int n = line.size();
    int dist = 15;
    std::pair<int, int> res;
    int &a = res.first, &b = res.second;
    // 0 -> a, 1 -> b
    int pre = -1;

    for (int i = 0; i < n - dist; i++) {
        int next = i + dist;
        // a 和 b 需要成对出现因此 a 与 b 的差值必须小于等于 1
        int diff = line[i] - line[next];
        if (diff < -threshold && pre != 1) {
            a++;
            pre = 1;
        }
        if (diff > threshold && pre != 0) {
            b++;
            pre = 0;
        }
    }
    return res;
}

/**
 * @brief 最小二乘法判断直线
 * @param points 曲线点集
 * @param threshold 直线阈值
 * @return 是否为直线
 * @author Cao Xin
 * @date 2025-04-13
 */
bool is_line(const std::vector<cv::Point>& points, float threshold) {
    // 点数太少默认为曲线
    if (points.size() < 3) {
        return false;
    }

    // 拟合直线
    line_params params = fit_line(points);

    // 计算每个点到直线的距离
    float total_distance = 0;
    if (!params.is_vertical) {
        // 非垂直线：y = mx + b
        for (const auto& p : points) {
            // 点到直线距离公式：|mx - y + b| / sqrt(m^2 + 1)
            float distance = std::abs(params.slope * p.x - p.y + params.intercept) /
                           std::sqrt(params.slope * params.slope + 1);
            total_distance += distance;
        }
    } else {
        // 垂直线：x = c
        for (const auto& p : points) {
            float distance = std::abs(p.x - params.c);
            total_distance += distance;
        }
    }

    // 计算平均偏差
    float avg_distance = total_distance / points.size();
    std::cout << avg_distance << '\n';
    // 根据阈值判断
    return avg_distance < threshold;
}

/**
 * @brief 移除边界上的点
 */
void remove_bound_pts(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, cv::Size size) {
    int height = size.height;
    int width = size.width;
    if(width < height) {
        std::swap(height, width);
    }
    pts_out.clear();
    for (cv::Point pts : pts_in) {
        if (pts.x <= 15 || pts.y <= 15 || pts.x >= width - 15 || pts.y >= width - 15) {
            continue;
        }
        pts_out.push_back(pts);
    }
}

/**
 * @brief 镜像翻转边线
 */
std::vector<cv::Point> mirror_line(const std::vector<cv::Point>& line, cv::Size size) {
    std::vector<cv::Point> res;
    if (line.empty()) {
        return res;
    }

    int height = size.height;
    int width  = size.width;
    if (width < height) {
        std::swap(width, height);
    }

    res.reserve(line.size());

    // 对于每个点 (x, y)，新的 x 坐标为 (width - 1 - x)，y 坐标保持不变
    // 这样 0 -> width-1, width-1 -> 0, 1 -> width-2, width-2 -> 1, 以此类推
    for (const auto& pt : line) {
        int mirroredX = width - 1 - pt.x;
        mirroredX = clip(mirroredX, 0, width - 1);
        res.emplace_back(mirroredX, pt.y);
    }

    return res;
}

/**
 * @brief 计算单位向量
 * @param v 输入向量
 * @return 计算出的单位向量
 * @author Cao Xin
 * @date 2025-06-01
 */
cv::Point2f normalize(const cv::Point2f& v) {
    float length = std::sqrt(v.x * v.x + v.y * v.y);
    if (length == 0.0f) return cv::Point2f(0.0f, 0.0f);
    return cv::Point2f(v.x / length, v.y / length);
}

/**
 * @brief 计算向量长度
 * @param v 输入向量
 * @return 计算处的向量长度
 * @author Cao Xin
 * @date 2025-06-01
 */
static float length(const cv::Point2f& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief 沿切线方向平移点集
 * @param line 输入的曲线点集
 * @param dist 平移的距离
 * @return 平移后的点集 
 * @author Cao Xin
 * @date 2025-06-01
 */
std::vector<cv::Point> shift_line(
    const std::vector<cv::Point>& line,
    int shift_distance
) {
    size_t n = line.size();
    if (n < 2) {
        // 少于两个点时，无法确定切线，直接返回空
        return {};
    }

    // 先把所有点转换到浮点版本，方便计算
    std::vector<cv::Point2f> pts(n);
    for (size_t i = 0; i < n; ++i) {
        pts[i] = cv::Point2f(line[i].x, line[i].y);
    }

    std::vector<cv::Point> result(n);

    // 记录上一个非零切线方向的单位切线，防止连续相同点导致切线为零
    cv::Point2f prev_unit_tan(0, 0);

    for (size_t i = 0; i < n; ++i) {
        // 1) 计算当前顶点 i 处的切线方向
        //    对于内点，用中心差分：P[i+1] - P[i-1]
        //    对于第一个点，用前向差分：P[1] - P[0]
        //    对于最后一个点，用后向差分：P[n-1] - P[n-2]
        cv::Point2f raw_tan(0.f, 0.f);
        if (i == 0) {
            raw_tan = pts[1] - pts[0];
        } else if (i == n - 1) {
            raw_tan = pts[n - 1] - pts[n - 2];
        } else {
            raw_tan = pts[i + 1] - pts[i - 1];
        }

        // 2) 归一化为单位切线
        cv::Point2f unit_tan = normalize(raw_tan);
        if (length(unit_tan) < 1e-6f) {
            // 如果当前差分向量几乎为零（相邻点重合），则继续使用上一个非零切线方向
            unit_tan = prev_unit_tan;
        } else {
            prev_unit_tan = unit_tan;
        }

        // 3) 计算法线方向：逆时针旋转 90° 得到“左侧法线”
        //    (dx, dy) -> (-dy, dx)
        cv::Point2f unit_norm(-unit_tan.y, unit_tan.x);

        // 4) 当前点沿法线偏移 shift_distance
        result[i] = pts[i] + unit_norm * shift_distance;
    }

    return result;
}

std::vector<cv::Point> resample_to_single_y(const std::vector<cv::Point> & line, const cv::Size & size) {
    int height = std::min(size.height, size.width);
    int min_y = height;
    vector<cv::Point> res;
    for (cv::Point pt : line) {
        int y = pt.y;
        if (y > min_y) {
            continue;
        } else {
            res.push_back(pt);
            min_y = y;
        } 
    }
    return res;
}