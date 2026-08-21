/*
 * 文件作用：S7.6 手势到有界虚拟 RC 的纯逻辑与固定二进制帧接口。
 * 核心数据流：S7.5 ACTIVE 快照 -> 限幅/限速通道 -> 序号/心跳/时间戳 -> CRC16 帧。
 * 关键约束：每次 ACTIVE 边沿只触发一次短指令；健康的无手/切换状态持续发送有效中立；
 * 不生成 ARM 或总授权 AUX；不负责飞控仲裁。
 */
#ifndef S7_6_VIRTUAL_RC_HPP
#define S7_6_VIRTUAL_RC_HPP

#include "../s7_5/s7_5_gesture.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace s7_6 {

constexpr std::size_t kWireFrameSize = 44;
constexpr uint64_t kFramePeriodUs = 50000;
constexpr uint64_t kLinkTimeoutUs = 150000;
constexpr uint64_t kSourceTimeoutUs = 250000;
constexpr uint64_t kCommandPulseUs = 1000000;

struct Channels {
    int16_t roll{};
    int16_t pitch{};
    int16_t yaw{};
    int16_t throttle{};
    int16_t aux{};
};

struct VirtualRcFrame {
    bool valid{};
    s7_5::GestureId gesture_id{s7_5::GestureId::UNKNOWN};
    uint8_t confidence_percent{};
    uint32_t source_sequence{};
    uint32_t heartbeat{};
    uint64_t source_timestamp_us{};
    uint64_t send_timestamp_us{};
    Channels channels{};
};

class Mapper {
public:
    VirtualRcFrame update(const s7_5::GestureSnapshot& gesture, uint32_t source_sequence,
                          uint64_t source_timestamp_us, uint64_t send_timestamp_us);

private:
    Channels current_{};
    s7_5::GestureId command_gesture_{s7_5::GestureId::UNKNOWN};
    uint8_t last_confidence_percent_{};
    bool active_latched_{};
    uint32_t heartbeat_{};
    uint64_t pulse_until_us_{};
    uint64_t last_send_us_{};
};

std::array<uint8_t, kWireFrameSize> encode(const VirtualRcFrame& frame);
bool decode(const uint8_t* bytes, std::size_t size, uint64_t receive_timestamp_us,
            uint32_t previous_source_sequence, uint32_t previous_heartbeat,
            VirtualRcFrame& frame);
bool self_test();

}  // namespace s7_6

#endif
