/* 固定 115200 8N1 原始串口写出；写失败即终止进程，不自动重连或恢复有效动作。 */
#include "s7_6_serial.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace s7_6 {

SerialLink::SerialLink(const char* device) {
    if (!device || !*device) throw std::runtime_error("S7_6_RC_OUTPUT is empty");
    fd_ = open(device, O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("open RC output failed: " + std::string(std::strerror(errno)));
    termios options{};
    if (tcgetattr(fd_, &options) == 0) {
        cfmakeraw(&options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag = (options.c_cflag & ~(CSIZE | CSTOPB | PARENB | CRTSCTS)) | CS8 | CLOCAL;
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("configure RC output failed: " + std::string(std::strerror(errno)));
        }
    } else if (errno != ENOTTY) {
        close(fd_);
        fd_ = -1;
        throw std::runtime_error("inspect RC output failed: " + std::string(std::strerror(errno)));
    }
}

SerialLink::~SerialLink() {
    if (fd_ >= 0) close(fd_);
}

VirtualRcFrame SerialLink::send(const s7_5::GestureSnapshot& gesture, uint32_t source_sequence,
                                uint64_t source_timestamp_us, uint64_t send_timestamp_us) {
    const auto frame = mapper_.update(gesture, source_sequence, source_timestamp_us, send_timestamp_us);
    const auto bytes = encode(frame);
    std::size_t written{};
    while (written < bytes.size()) {
        const ssize_t count = write(fd_, bytes.data() + written, bytes.size() - written);
        if (count > 0) {
            written += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) continue;
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd output{fd_, POLLOUT, 0};
            if (poll(&output, 1, 50) > 0) continue;
        }
        throw std::runtime_error("write RC frame failed: " + std::string(std::strerror(errno)));
    }
    return frame;
}

}  // namespace s7_6
