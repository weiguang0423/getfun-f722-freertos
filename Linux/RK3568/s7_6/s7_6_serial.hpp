/* Linux/Buildroot 串口输出薄封装；设备名由运行环境提供，不在源码中猜测 RK3568 引脚。 */
#ifndef S7_6_SERIAL_HPP
#define S7_6_SERIAL_HPP

#include "s7_6_virtual_rc.hpp"

namespace s7_6 {

class SerialLink {
public:
    explicit SerialLink(const char* device);
    ~SerialLink();
    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    VirtualRcFrame send(const s7_5::GestureSnapshot& gesture, uint32_t source_sequence,
                        uint64_t source_timestamp_us, uint64_t send_timestamp_us);

private:
    int fd_{-1};
    Mapper mapper_;
};

}  // namespace s7_6

#endif
