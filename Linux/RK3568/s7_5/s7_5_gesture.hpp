/*
 * 文件作用：S7.5 单手静态手势分类与安全时序状态机接口。
 * 核心数据流：21 点图像坐标 -> 几何手势观测 -> 连续帧确认 -> ACTIVE/释放状态。
 * 关键约束：只有 ACTIVE 暴露活动手势；无手、未知、低置信度、超时或切换立即清空活动手势。
 */
#ifndef S7_5_GESTURE_HPP
#define S7_5_GESTURE_HPP

#include <cstdint>

namespace s7_5 {

enum class GestureId : uint8_t {
    UNKNOWN = 0,
    OPEN_PALM = 1,
    FIST = 2,
    POINT = 3,
    V_SIGN = 4,
};

enum class GestureState : uint8_t {
    NO_HAND,
    UNKNOWN,
    CANDIDATE,
    ACTIVE,
    RELEASED,
};

struct GestureObservation {
    GestureId id{GestureId::UNKNOWN};
    float confidence{};
};

struct GestureSnapshot {
    GestureState state{GestureState::NO_HAND};
    GestureId observed_id{GestureId::UNKNOWN};
    float observed_confidence{};
    GestureId active_id{GestureId::UNKNOWN};
    unsigned candidate_frames{};
};

const char* gesture_name(GestureId id);
const char* state_name(GestureState state);
GestureObservation classify(const float* landmarks_21x3_image, float presence_score);

class GestureStateMachine {
public:
    GestureSnapshot update(bool hand_valid, GestureObservation observation, uint64_t now_us);

private:
    void release(uint64_t now_us);

    GestureState state_{GestureState::NO_HAND};
    GestureId candidate_id_{GestureId::UNKNOWN};
    GestureId active_id_{GestureId::UNKNOWN};
    unsigned candidate_frames_{};
    uint64_t candidate_since_us_{};
    uint64_t release_until_us_{};
    uint64_t last_update_us_{};
};

bool self_test();

}  // namespace s7_5

#endif
