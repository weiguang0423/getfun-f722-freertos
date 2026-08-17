/*
 * 文件作用：实现 S7.5 的四类几何手势和五态防抖状态机。
 * 核心功能：以手指关节直线度/伸展长度区分张掌、握拳、单指和 V 字，并用置信度、
 * 连续 5 帧、150 ms 确认、250 ms 输入超时和 300 ms 释放冷却隔离误触发。
 * 关键约束：分类器不产生 RC；状态机在任何失效输入上立即把 active_id 置 UNKNOWN。
 */
#include "s7_5_gesture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <utility>

namespace s7_5 {
namespace {

constexpr float kMinClassificationConfidence = 0.58f;
constexpr float kMinClassificationMargin = 0.08f;
constexpr float kMinActiveConfidence = 0.75f;
constexpr unsigned kConfirmFrames = 5;
constexpr uint64_t kConfirmUs = 150000;
constexpr uint64_t kMaxGapUs = 250000;
constexpr uint64_t kCooldownUs = 300000;

struct Point {
    float x;
    float y;
};

float clamp01(float value) { return std::clamp(value, 0.0f, 1.0f); }

float distance(Point a, Point b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

float straightness(Point a, Point b, Point c) {
    const float ux = a.x - b.x, uy = a.y - b.y;
    const float vx = c.x - b.x, vy = c.y - b.y;
    const float length = std::hypot(ux, uy) * std::hypot(vx, vy);
    if (length < 1e-6f) return 0.0f;
    const float angle = std::acos(std::clamp((ux * vx + uy * vy) / length, -1.0f, 1.0f));
    return clamp01((angle - 1.75f) / 1.05f);  // 100..160 degrees -> 0..1
}

float extension(const std::array<Point, 21>& p, int mcp) {
    const float joints = std::min(straightness(p[mcp], p[mcp + 1], p[mcp + 2]),
                                  straightness(p[mcp + 1], p[mcp + 2], p[mcp + 3]));
    const float chain = distance(p[mcp], p[mcp + 1]) + distance(p[mcp + 1], p[mcp + 2])
                      + distance(p[mcp + 2], p[mcp + 3]);
    const float reach = chain > 1e-6f ? clamp01((distance(p[mcp], p[mcp + 3]) / chain - .55f) / .30f) : 0.0f;
    return .65f * joints + .35f * reach;
}

float thumb_extension(const std::array<Point, 21>& p) {
    const float joints = std::min(straightness(p[1], p[2], p[3]), straightness(p[2], p[3], p[4]));
    const float palm = std::max(distance(p[5], p[17]), 1e-6f);
    const float span = clamp01((distance(p[4], p[5]) / palm - .45f) / .75f);
    return .65f * joints + .35f * span;
}

float average(std::initializer_list<float> values) {
    float sum = 0.0f;
    for (float value : values) sum += clamp01(value);
    return sum / static_cast<float>(values.size());
}

std::array<float, 5> finger_extensions(const std::array<Point, 21>& p) {
    return {thumb_extension(p), extension(p, 5), extension(p, 9), extension(p, 13), extension(p, 17)};
}

std::array<Point, 21> make_test_hand(const std::array<bool, 5>& extended) {
    std::array<Point, 21> p{};
    p[0] = {0.0f, 0.0f};
    p[1] = {-.30f, -.30f};
    if (extended[0]) {
        p[2] = {-.58f, -.48f}; p[3] = {-.82f, -.68f}; p[4] = {-1.02f, -.88f};
    } else {
        p[2] = {-.45f, -.48f}; p[3] = {-.20f, -.62f}; p[4] = {.05f, -.55f};
    }
    constexpr float xs[4] = {-.36f, -.12f, .14f, .38f};
    constexpr float lengths[4] = {1.55f, 1.72f, 1.55f, 1.34f};
    for (int finger = 0; finger < 4; ++finger) {
        const int base = 5 + finger * 4;
        const float x = xs[finger];
        p[base] = {x, -.55f};
        if (extended[finger + 1]) {
            p[base + 1] = {x, -.55f - lengths[finger] * .36f};
            p[base + 2] = {x, -.55f - lengths[finger] * .68f};
            p[base + 3] = {x, -.55f - lengths[finger]};
        } else {
            p[base + 1] = {x, -1.00f};
            p[base + 2] = {x + .24f, -.78f};
            p[base + 3] = {x + .16f, -.48f};
        }
    }
    return p;
}

GestureObservation classify_points(const std::array<Point, 21>& points, float presence_score) {
    const auto e = finger_extensions(points);
    const float separation = clamp01((distance(points[8], points[12])
                                     / std::max(distance(points[5], points[17]), 1e-6f) - .20f) / .45f);
    const std::array<std::pair<GestureId, float>, 4> scores{{
        {GestureId::OPEN_PALM, average({e[0], e[1], e[2], e[3], e[4]})},
        {GestureId::FIST, average({1-e[0], 1-e[1], 1-e[2], 1-e[3], 1-e[4]})},
        {GestureId::POINT, average({e[1], 1-e[2], 1-e[3], 1-e[4]})},
        {GestureId::V_SIGN, average({e[1], e[2], 1-e[3], 1-e[4], separation})},
    }};
    auto best = scores.begin();
    float second = 0.0f;
    for (auto it = scores.begin(); it != scores.end(); ++it) {
        if (it->second > best->second) {
            second = best->second;
            best = it;
        } else if (it != best && it->second > second) {
            second = it->second;
        }
    }
    const float confidence = std::min(clamp01(presence_score), best->second);
    if (confidence < kMinClassificationConfidence || best->second - second < kMinClassificationMargin)
        return {GestureId::UNKNOWN, confidence};
    return {best->first, confidence};
}

}  // namespace

const char* gesture_name(GestureId id) {
    switch (id) {
    case GestureId::OPEN_PALM: return "OPEN_PALM";
    case GestureId::FIST: return "FIST";
    case GestureId::POINT: return "POINT";
    case GestureId::V_SIGN: return "V_SIGN";
    default: return "UNKNOWN";
    }
}

const char* state_name(GestureState state) {
    switch (state) {
    case GestureState::NO_HAND: return "NO_HAND";
    case GestureState::UNKNOWN: return "UNKNOWN";
    case GestureState::CANDIDATE: return "CANDIDATE";
    case GestureState::ACTIVE: return "ACTIVE";
    case GestureState::RELEASED: return "RELEASED";
    }
    return "UNKNOWN";
}

GestureObservation classify(const float* landmarks, float presence_score) {
    if (!landmarks || !std::isfinite(presence_score)) return {};
    std::array<Point, 21> points{};
    for (int i = 0; i < 21; ++i) {
        if (!std::isfinite(landmarks[3 * i]) || !std::isfinite(landmarks[3 * i + 1])) return {};
        points[i] = {landmarks[3 * i], landmarks[3 * i + 1]};
    }
    if (distance(points[5], points[17]) < 1e-5f) return {};
    return classify_points(points, presence_score);
}

void GestureStateMachine::release(uint64_t now_us) {
    state_ = GestureState::RELEASED;
    active_id_ = GestureId::UNKNOWN;
    candidate_id_ = GestureId::UNKNOWN;
    candidate_frames_ = 0;
    candidate_since_us_ = 0;
    release_until_us_ = now_us + kCooldownUs;
}

GestureSnapshot GestureStateMachine::update(bool hand_valid, GestureObservation observation,
                                            uint64_t now_us) {
    const bool gap = last_update_us_ && (now_us < last_update_us_ || now_us - last_update_us_ > kMaxGapUs);
    last_update_us_ = now_us;
    if (gap && (state_ == GestureState::ACTIVE || state_ == GestureState::CANDIDATE)) release(now_us);

    const bool recognized = hand_valid && observation.id != GestureId::UNKNOWN
                         && observation.confidence >= kMinActiveConfidence;
    if (!hand_valid || !recognized) {
        if (state_ == GestureState::ACTIVE || state_ == GestureState::CANDIDATE) release(now_us);
        else if (state_ != GestureState::RELEASED || now_us >= release_until_us_)
            state_ = hand_valid ? GestureState::UNKNOWN : GestureState::NO_HAND;
        active_id_ = GestureId::UNKNOWN;
    } else if (state_ == GestureState::RELEASED && now_us < release_until_us_) {
        active_id_ = GestureId::UNKNOWN;
    } else if (state_ == GestureState::ACTIVE) {
        if (observation.id != active_id_) release(now_us);
    } else {
        if (observation.id != candidate_id_) {
            candidate_id_ = observation.id;
            candidate_frames_ = 1;
            candidate_since_us_ = now_us;
        } else {
            ++candidate_frames_;
        }
        state_ = GestureState::CANDIDATE;
        if (candidate_frames_ >= kConfirmFrames && now_us - candidate_since_us_ >= kConfirmUs) {
            state_ = GestureState::ACTIVE;
            active_id_ = candidate_id_;
        }
    }
    return {state_, hand_valid ? observation.id : GestureId::UNKNOWN,
            hand_valid ? observation.confidence : 0.0f, active_id_, candidate_frames_};
}

bool self_test() {
    constexpr std::array<std::pair<std::array<bool, 5>, GestureId>, 4> cases{{
        {{{true, true, true, true, true}}, GestureId::OPEN_PALM},
        {{{false, false, false, false, false}}, GestureId::FIST},
        {{{false, true, false, false, false}}, GestureId::POINT},
        {{{false, true, true, false, false}}, GestureId::V_SIGN},
    }};
    for (const auto& item : cases)
        if (classify_points(make_test_hand(item.first), .99f).id != item.second) return false;

    GestureStateMachine machine;
    GestureSnapshot snapshot;
    for (int i = 0; i < 4; ++i)
        snapshot = machine.update(true, {GestureId::OPEN_PALM, .90f}, 1000000u + i * 50000u);
    if (snapshot.state != GestureState::CANDIDATE || snapshot.active_id != GestureId::UNKNOWN) return false;
    snapshot = machine.update(true, {GestureId::OPEN_PALM, .90f}, 1200000u);
    if (snapshot.state != GestureState::ACTIVE || snapshot.active_id != GestureId::OPEN_PALM) return false;
    snapshot = machine.update(true, {GestureId::OPEN_PALM, .70f}, 1250000u);
    if (snapshot.state != GestureState::RELEASED || snapshot.active_id != GestureId::UNKNOWN) return false;
    snapshot = machine.update(true, {GestureId::FIST, .90f}, 1400000u);
    if (snapshot.state != GestureState::RELEASED || snapshot.active_id != GestureId::UNKNOWN) return false;
    snapshot = machine.update(false, {}, 1600000u);
    if (snapshot.state != GestureState::NO_HAND || snapshot.active_id != GestureId::UNKNOWN) return false;

    GestureStateMachine timeout_machine;
    for (int i = 0; i < 5; ++i)
        snapshot = timeout_machine.update(true, {GestureId::POINT, .90f}, 2000000u + i * 50000u);
    if (snapshot.state != GestureState::ACTIVE) return false;
    snapshot = timeout_machine.update(true, {GestureId::POINT, .90f}, 2500001u);
    return snapshot.state == GestureState::RELEASED && snapshot.active_id == GestureId::UNKNOWN;
}

}  // namespace s7_5
