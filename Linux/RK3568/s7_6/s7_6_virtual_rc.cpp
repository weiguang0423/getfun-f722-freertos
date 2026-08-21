/*
 * 文件作用：实现 S7.6 事件式手势映射、变化率限制、固定小端帧和 CRC16 完整性检查。
 * 冻结映射：张掌=Pitch -0.30，单指=Pitch +0.30，V 字=Roll -0.30，握拳=释放。
 * 关键约束：手势只在 ACTIVE 边沿触发一次短脉冲，健康的无手/切换状态不截断脉冲也不退出 Linux；
 * Throttle/AUX 始终为 0，只有握拳、源过期或管线故障输出无效释放帧。
 */
#include "s7_6_virtual_rc.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace s7_6 {
namespace {

constexpr uint8_t kMagic0 = 'G';
constexpr uint8_t kMagic1 = 'R';
constexpr uint8_t kVersion = 1;
constexpr int16_t kAxisLimit = 300;
constexpr int16_t kThrottleLimit = 250;
constexpr int16_t kAuxLimit = 1000;
constexpr int32_t kAxisRatePerSecond = 600;
constexpr int32_t kThrottleRatePerSecond = 400;
constexpr int32_t kAuxRatePerSecond = 1000;

uint16_t crc16(const uint8_t* bytes, std::size_t size) {
    uint16_t crc = 0xffffu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<uint16_t>((crc & 0x8000u) ? (crc << 1) ^ 0x1021u : crc << 1);
    }
    return crc;
}

void put_u16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
}

void put_u32(uint8_t* out, uint32_t value) {
    for (int i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}

void put_u64(uint8_t* out, uint64_t value) {
    for (int i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
}

uint16_t get_u16(const uint8_t* in) {
    return static_cast<uint16_t>(in[0]) | static_cast<uint16_t>(in[1]) << 8;
}

uint32_t get_u32(const uint8_t* in) {
    uint32_t value{};
    for (int i = 0; i < 4; ++i) value |= static_cast<uint32_t>(in[i]) << (8 * i);
    return value;
}

uint64_t get_u64(const uint8_t* in) {
    uint64_t value{};
    for (int i = 0; i < 8; ++i) value |= static_cast<uint64_t>(in[i]) << (8 * i);
    return value;
}

bool newer(uint32_t value, uint32_t previous) {
    return value != previous && static_cast<int32_t>(value - previous) > 0;
}

int16_t approach(int16_t current, int16_t target, int32_t rate_per_second, uint64_t elapsed_us) {
    if (current == target) return current;
    const int64_t scaled = static_cast<int64_t>(rate_per_second) * elapsed_us;
    const int32_t step = static_cast<int32_t>(std::max<int64_t>(1, scaled / 1000000));
    if (target > current) return static_cast<int16_t>(std::min<int32_t>(target, current + step));
    return static_cast<int16_t>(std::max<int32_t>(target, current - step));
}

Channels target_for(s7_5::GestureId id) {
    switch (id) {
    case s7_5::GestureId::OPEN_PALM: return {0, -300, 0, 0, 0};
    case s7_5::GestureId::POINT: return {0, 300, 0, 0, 0};
    case s7_5::GestureId::V_SIGN: return {-300, 0, 0, 0, 0};
    default: return {};
    }
}

bool channels_in_range(const Channels& value) {
    return std::abs(static_cast<int>(value.roll)) <= kAxisLimit
        && std::abs(static_cast<int>(value.pitch)) <= kAxisLimit
        && std::abs(static_cast<int>(value.yaw)) <= kAxisLimit
        && value.throttle >= 0 && value.throttle <= kThrottleLimit
        && value.aux >= 0 && value.aux <= kAuxLimit;
}

bool channels_are_zero(const Channels& value) {
    return value.roll == 0 && value.pitch == 0 && value.yaw == 0
        && value.throttle == 0 && value.aux == 0;
}

bool channels_match_gesture(const VirtualRcFrame& frame) {
    if (frame.channels.yaw != 0 || frame.channels.throttle != 0 || frame.channels.aux != 0)
        return false;
    switch (frame.gesture_id) {
    case s7_5::GestureId::UNKNOWN:
        return frame.confidence_percent == 0 && channels_are_zero(frame.channels);
    case s7_5::GestureId::OPEN_PALM:
        return frame.confidence_percent >= 75 && frame.channels.roll == 0
            && frame.channels.pitch >= -kAxisLimit && frame.channels.pitch <= 0;
    case s7_5::GestureId::POINT:
        return frame.confidence_percent >= 75 && frame.channels.roll == 0
            && frame.channels.pitch >= 0 && frame.channels.pitch <= kAxisLimit;
    case s7_5::GestureId::V_SIGN:
        return frame.confidence_percent >= 75 && frame.channels.pitch == 0
            && frame.channels.roll >= -kAxisLimit && frame.channels.roll <= 0;
    default:
        return false;
    }
}

}  // namespace

VirtualRcFrame Mapper::update(const s7_5::GestureSnapshot& gesture, uint32_t source_sequence,
                              uint64_t source_timestamp_us, uint64_t send_timestamp_us) {
    VirtualRcFrame frame;
    frame.confidence_percent = std::isfinite(gesture.observed_confidence)
        ? static_cast<uint8_t>(std::clamp(std::lround(gesture.observed_confidence * 100.0f), 0l, 100l))
        : 0;
    frame.source_sequence = source_sequence;
    frame.heartbeat = ++heartbeat_;
    frame.source_timestamp_us = source_timestamp_us;
    frame.send_timestamp_us = send_timestamp_us;

    const bool fresh = source_timestamp_us && send_timestamp_us >= source_timestamp_us
                    && send_timestamp_us - source_timestamp_us <= kSourceTimeoutUs;
    const bool ordered = !last_send_us_ || send_timestamp_us >= last_send_us_;
    const bool active = gesture.state == s7_5::GestureState::ACTIVE
                     && gesture.active_id != s7_5::GestureId::UNKNOWN
                     && gesture.observed_id == gesture.active_id
                     && std::isfinite(gesture.observed_confidence)
                     && gesture.observed_confidence >= .75f;
    const bool release = active && gesture.active_id == s7_5::GestureId::FIST;

    if (!fresh || !ordered || release) {
        current_ = {};
        command_gesture_ = s7_5::GestureId::UNKNOWN;
        last_confidence_percent_ = 0;
        active_latched_ = false;
        pulse_until_us_ = 0;
        frame.confidence_percent = 0;
        frame.channels = current_;
        last_send_us_ = send_timestamp_us;
        return frame;
    }

    if (active) {
        last_confidence_percent_ = frame.confidence_percent;
        if (!active_latched_ || command_gesture_ != gesture.active_id) {
            current_ = {};
            command_gesture_ = gesture.active_id;
            pulse_until_us_ = send_timestamp_us + kCommandPulseUs;
        }
        active_latched_ = true;
    } else {
        active_latched_ = false;
    }

    frame.valid = true;
    const Channels target = send_timestamp_us < pulse_until_us_
                          ? target_for(command_gesture_) : Channels{};
    const uint64_t elapsed = last_send_us_ && send_timestamp_us >= last_send_us_
                           ? std::min<uint64_t>(send_timestamp_us - last_send_us_, kSourceTimeoutUs)
                           : kFramePeriodUs;
    current_.roll = approach(current_.roll, target.roll, kAxisRatePerSecond, elapsed);
    current_.pitch = approach(current_.pitch, target.pitch, kAxisRatePerSecond, elapsed);
    current_.yaw = approach(current_.yaw, target.yaw, kAxisRatePerSecond, elapsed);
    current_.throttle = approach(current_.throttle, target.throttle, kThrottleRatePerSecond, elapsed);
    current_.aux = approach(current_.aux, target.aux, kAuxRatePerSecond, elapsed);
    if (!channels_are_zero(current_)) {
        frame.gesture_id = command_gesture_;
        frame.confidence_percent = last_confidence_percent_;
    } else {
        frame.gesture_id = s7_5::GestureId::UNKNOWN;
        frame.confidence_percent = 0;
    }
    frame.channels = current_;
    last_send_us_ = send_timestamp_us;
    return frame;
}

std::array<uint8_t, kWireFrameSize> encode(const VirtualRcFrame& frame) {
    std::array<uint8_t, kWireFrameSize> out{};
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = static_cast<uint8_t>(kWireFrameSize);
    out[4] = frame.valid ? 1u : 0u;
    out[5] = static_cast<uint8_t>(frame.gesture_id);
    out[6] = frame.confidence_percent;
    put_u32(out.data() + 8, frame.source_sequence);
    put_u32(out.data() + 12, frame.heartbeat);
    put_u64(out.data() + 16, frame.source_timestamp_us);
    put_u64(out.data() + 24, frame.send_timestamp_us);
    put_u16(out.data() + 32, static_cast<uint16_t>(frame.channels.roll));
    put_u16(out.data() + 34, static_cast<uint16_t>(frame.channels.pitch));
    put_u16(out.data() + 36, static_cast<uint16_t>(frame.channels.yaw));
    put_u16(out.data() + 38, static_cast<uint16_t>(frame.channels.throttle));
    put_u16(out.data() + 40, static_cast<uint16_t>(frame.channels.aux));
    put_u16(out.data() + 42, crc16(out.data(), 42));
    return out;
}

bool decode(const uint8_t* bytes, std::size_t size, uint64_t receive_timestamp_us,
            uint32_t previous_source_sequence, uint32_t previous_heartbeat,
            VirtualRcFrame& frame) {
    if (!bytes || size != kWireFrameSize || bytes[0] != kMagic0 || bytes[1] != kMagic1
        || bytes[2] != kVersion || bytes[3] != kWireFrameSize || (bytes[4] & ~1u)
        || bytes[5] > static_cast<uint8_t>(s7_5::GestureId::V_SIGN) || bytes[6] > 100 || bytes[7]
        || get_u16(bytes + 42) != crc16(bytes, 42))
        return false;

    VirtualRcFrame decoded;
    decoded.valid = bytes[4] != 0;
    decoded.gesture_id = static_cast<s7_5::GestureId>(bytes[5]);
    decoded.confidence_percent = bytes[6];
    decoded.source_sequence = get_u32(bytes + 8);
    decoded.heartbeat = get_u32(bytes + 12);
    decoded.source_timestamp_us = get_u64(bytes + 16);
    decoded.send_timestamp_us = get_u64(bytes + 24);
    decoded.channels = {static_cast<int16_t>(get_u16(bytes + 32)),
                        static_cast<int16_t>(get_u16(bytes + 34)),
                        static_cast<int16_t>(get_u16(bytes + 36)),
                        static_cast<int16_t>(get_u16(bytes + 38)),
                        static_cast<int16_t>(get_u16(bytes + 40))};
    if (!channels_in_range(decoded.channels) || !newer(decoded.heartbeat, previous_heartbeat)
        || decoded.send_timestamp_us > receive_timestamp_us
        || receive_timestamp_us - decoded.send_timestamp_us > kLinkTimeoutUs)
        return false;
    if (decoded.valid
        && (!newer(decoded.source_sequence, previous_source_sequence)
            || !decoded.source_timestamp_us
            || decoded.send_timestamp_us < decoded.source_timestamp_us
            || decoded.send_timestamp_us - decoded.source_timestamp_us > kSourceTimeoutUs
            || !channels_match_gesture(decoded)))
        return false;
    if (!decoded.valid && (decoded.channels.roll || decoded.channels.pitch || decoded.channels.yaw
                           || decoded.channels.throttle || decoded.channels.aux))
        return false;
    frame = decoded;
    return true;
}

bool self_test() {
    Mapper mapper;
    s7_5::GestureSnapshot inactive;
    auto frame = mapper.update(inactive, 1, 1000000, 1010000);
    if (!frame.valid || frame.gesture_id != s7_5::GestureId::UNKNOWN
        || !channels_are_zero(frame.channels) || frame.heartbeat != 1)
        return false;
    const auto neutral_bytes = encode(frame);
    VirtualRcFrame neutral_decoded;
    if (!decode(neutral_bytes.data(), neutral_bytes.size(), 1011000, 0, 0,
                neutral_decoded)
        || !neutral_decoded.valid
        || neutral_decoded.gesture_id != s7_5::GestureId::UNKNOWN)
        return false;

    s7_5::GestureSnapshot palm{s7_5::GestureState::ACTIVE, s7_5::GestureId::OPEN_PALM,
                               .90f, s7_5::GestureId::OPEN_PALM, 5};
    frame = mapper.update(palm, 2, 1050000, 1060000);
    if (!frame.valid || frame.channels.pitch != -30 || frame.channels.throttle != 0
        || frame.channels.aux != 0)
        return false;
    for (uint32_t i = 3; i < 22; ++i)
        frame = mapper.update(palm, i, 950000u + i * 50000u, 960000u + i * 50000u);
    if (frame.channels.pitch != -300) return false;

    frame = mapper.update(palm, 22, 2050000, 2060000);
    if (!frame.valid || frame.channels.pitch != -270) return false;

    s7_5::GestureSnapshot transition{s7_5::GestureState::RELEASED,
                                     s7_5::GestureId::UNKNOWN, 0.0f,
                                     s7_5::GestureId::UNKNOWN, 0};
    frame = mapper.update(transition, 23, 2100000, 2110000);
    if (!frame.valid || frame.gesture_id != s7_5::GestureId::OPEN_PALM
        || frame.channels.pitch != -240)
        return false;

    s7_5::GestureSnapshot v_sign{s7_5::GestureState::ACTIVE, s7_5::GestureId::V_SIGN,
                                 .95f, s7_5::GestureId::V_SIGN, 5};
    frame = mapper.update(v_sign, 24, 2150000, 2160000);
    if (!frame.valid || frame.channels.roll != -30 || frame.channels.pitch != 0
        || frame.channels.yaw != 0)
        return false;

    const auto bytes = encode(frame);
    VirtualRcFrame decoded;
    if (!decode(bytes.data(), bytes.size(), frame.send_timestamp_us + 1000,
                frame.source_sequence - 1, frame.heartbeat - 1, decoded)
        || decoded.channels.roll != frame.channels.roll)
        return false;
    auto corrupt = bytes;
    corrupt[34] ^= 1u;
    if (decode(corrupt.data(), corrupt.size(), frame.send_timestamp_us + 1000,
               frame.source_sequence - 1, frame.heartbeat - 1, decoded))
        return false;
    if (decode(bytes.data(), bytes.size(), frame.send_timestamp_us + kLinkTimeoutUs + 1,
               frame.source_sequence - 1, frame.heartbeat - 1, decoded))
        return false;
    if (decode(bytes.data(), bytes.size(), frame.send_timestamp_us + 1000,
               frame.source_sequence, frame.heartbeat - 1, decoded))
        return false;

    Mapper grace_mapper;
    s7_5::GestureSnapshot point{s7_5::GestureState::ACTIVE, s7_5::GestureId::POINT,
                                .90f, s7_5::GestureId::POINT, 5};
    frame = grace_mapper.update(point, 1, 1990000, 2000000);
    if (!frame.valid || frame.channels.pitch != 30) return false;
    frame = grace_mapper.update(transition, 2, 2040000, 2050000);
    if (!frame.valid || frame.channels.pitch != 60) return false;
    frame = grace_mapper.update(transition, 3, 2990001, 3000001);
    if (!frame.valid || frame.channels.pitch != 0
        || frame.gesture_id != s7_5::GestureId::UNKNOWN)
        return false;
    frame = grace_mapper.update(point, 4, 3040000, 3050000);
    if (!frame.valid) return false;
    frame = grace_mapper.update(transition, 5, 0, 3100000);
    if (frame.valid || frame.channels.pitch != 0) return false;

    s7_5::GestureSnapshot fist{s7_5::GestureState::ACTIVE, s7_5::GestureId::FIST,
                               .95f, s7_5::GestureId::FIST, 5};
    frame = mapper.update(fist, 25, 2200000, 2210000);
    return !frame.valid && frame.channels.roll == 0 && frame.channels.throttle == 0;
}

}  // namespace s7_6
