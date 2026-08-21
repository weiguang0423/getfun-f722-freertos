/* S7.6 POSIX 写出最小回环：向临时文件发送两帧，再按接收契约解码。 */
#include "s7_6_serial.hpp"

#include <array>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

int main() {
    char path[] = "/tmp/s7_6_serial_XXXXXX";
    const int temporary = mkstemp(path);
    if (temporary < 0) return 1;
    close(temporary);

    {
        s7_6::SerialLink link(path);
        link.send({}, 1, 1000000, 1010000);
        const s7_5::GestureSnapshot point{s7_5::GestureState::ACTIVE, s7_5::GestureId::POINT,
                                          .90f, s7_5::GestureId::POINT, 5};
        link.send(point, 2, 1050000, 1060000);
    }

    std::array<uint8_t, s7_6::kWireFrameSize * 2> bytes{};
    const int input = open(path, O_RDONLY);
    const ssize_t count = input >= 0 ? read(input, bytes.data(), bytes.size()) : -1;
    if (input >= 0) close(input);
    unlink(path);
    if (count != static_cast<ssize_t>(bytes.size())) return 1;

    s7_6::VirtualRcFrame first, second;
    if (!s7_6::decode(bytes.data(), s7_6::kWireFrameSize, 1070000, 0, 0, first)
        || !first.valid || first.gesture_id != s7_5::GestureId::UNKNOWN
        || first.channels.roll != 0 || first.channels.pitch != 0
        || first.heartbeat != 1)
        return 1;
    if (!s7_6::decode(bytes.data() + s7_6::kWireFrameSize, s7_6::kWireFrameSize,
                      1070000, first.source_sequence, first.heartbeat, second)
        || !second.valid || second.channels.pitch != 30 || second.heartbeat != 2)
        return 1;
    std::cout << "s7.6 serial loopback self-test ok\n";
    return 0;
}
