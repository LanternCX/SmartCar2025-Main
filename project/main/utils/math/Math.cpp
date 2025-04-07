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