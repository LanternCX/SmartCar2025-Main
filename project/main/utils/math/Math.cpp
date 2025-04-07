/**
 * @brief 限幅
 * @param a 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅之后的数据
 * @author Cao Xin
 * @date 2025-04-05
 */
float minmax(float a, float min, float max){
    if(a < min){
        return min;
    }
    if(a > max){
        return max;
    }
    return a;
}

/**
 * @brief 取最小值
 * @param a
 * @param b
 * @return a 和 b 的最小值
 * @author Cao Xin
 * @date 2025-04-05
 */
float min(float a, float b){
    return a > b ? b : a;
}

/**
 * @brief 取最大值
 * @param a
 * @param b
 * @return a 和 b 的最大值
 * @author Cao Xin
 * @date 2025-04-05
 */
float max(float a, float b){
    return a < b ? b : a;
}