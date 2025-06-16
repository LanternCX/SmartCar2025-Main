#include <queue>
#include <vector>

#include "Track.h"
#include "Type.h"

/**
 * @brief 赛道状态单例，主要维护前几帧率的赛道的各种状态提供给状态机使用
 * @author Cao Xin
 * @date 2025-05-31
 */

/**
 * @brief 维护的前几帧的赛道状态辅助判断当前赛道状态
 */
typedef struct track_type {
    // 当前的元素类型
    ElementType type;

    // 前 10 帧内计算出的赛道类型，使用队列维护
    std::queue<ElementType> pre_frame_type;

    // 前 10 帧内赛道的元素类型数量
    std::vector<int> type_cnt;

    // 元素种类数量
    int element_count;

    // 需要维护的赛道类型数量
    const int length = 5;
} track_type;

static track_type state;

/**
 * @brief 初始化单例
 * @param element_cnt 元素种类数量
 */
void init_state() {
    int element_cnt = int_to_element.size();
    state.type = LINE;
    state.pre_frame_type = std::queue<ElementType>();
    state.type_cnt = std::vector<int>(element_cnt);
    state.element_count = element_cnt;

    state.pre_frame_type.push(LINE);
}

/**
 * @brief 维护赛道状态
 * @param now 当前帧计算出的赛道状态
 */
void change_type_count(ElementType now) {
    if (state.pre_frame_type.size() > state.length) {
        ElementType pre = state.pre_frame_type.front();
        state.pre_frame_type.pop();
        state.type_cnt[pre]--;
    }

    state.pre_frame_type.push(now);
    state.type_cnt[now]++;
}

/**
 * @brief 获取当前赛道状态
 * @return 当前赛道状态
 */
ElementType get_track_type(void) {
    return state.type;
}

/**
 * @brief 计算当前赛道状态
 */
void calc_track_type(void) {
    // 取前 10 帧计算中出现最多的赛道类型作为当前赛道元素
    int max_idx = 0;
    for (int i = 0; i < state.element_count; i++) {
        if (state.type_cnt[i] > state.type_cnt[max_idx]) {
            max_idx = i;
        }
    }
    state.type = int_to_element[max_idx];
}