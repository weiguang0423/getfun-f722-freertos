/*
 * 文件作用：S7.4 屏显观测后端（DRM/KMS 直显实现）。不依赖 libdrm/gstreamer/Qt，
 * 只使用 Linux 内核 uapi 头（drm.h/drm_mode.h/drm_fourcc.h）与裸 ioctl，参考
 * libdrm modetest 与 kmscube 的实现模式，由项目自主实现最小显示后端。
 * 核心函数：display_open（枚举 connector 首选模式、创建双 dumb framebuffer 并点亮）、
 * display_show（resize + XRGB8888 填充 + 双缓冲 page flip）、display_close（释放资源）。
 * 主要数据流：BGR888 帧 → cv::resize → 双缓冲交替填充 → DRM_MODE_PAGE_FLIP → 屏。
 * 关键约束：flip 完成事件用 poll+read 及时消费，避免事件队列满导致 EBUSY；
 * 所有失败只向 stderr 告警并返回可恢复状态，不得影响调用方推理链。
 */
#include "s7_4_display.hpp"

#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

// SDK 内核 uapi（老接口）未提供连接状态宏；1/2/3 为长期稳定 ABI 值
#ifndef DRM_MODE_CONNECTED
#define DRM_MODE_CONNECTED 1
#define DRM_MODE_DISCONNECTED 2
#define DRM_MODE_UNKNOWNCONNECTION 3
#endif

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

struct Display {
    int fd{-1};
    uint32_t crtc{};
    uint32_t connector{};
    drm_mode_modeinfo mode{};
    int screen_width{};
    int screen_height{};
    uint32_t fbs[2]{};
    uint32_t handles[2]{};
    uint32_t pitches[2]{};
    size_t sizes[2]{};
    void* maps[2]{};
    int front{0};
    bool flip_pending{false};
};

namespace {

int drm_ioctl(int fd, unsigned long request, void* arg) {
    int result;
    do result = ioctl(fd, request, arg); while (result != 0 && errno == EINTR);
    return result;
}

void warn(const char* what, int error) {
    std::fprintf(stderr, "display: %s: %s\n", what, std::strerror(error));
}

bool get_card_res(int fd, drm_mode_card_res& res, std::vector<uint32_t>& ids) {
    std::memset(&res, 0, sizeof(res));
    if (drm_ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) != 0) return false;
    ids.assign(res.count_crtcs + res.count_connectors + res.count_encoders, 0);
    res.crtc_id_ptr = reinterpret_cast<uint64_t>(ids.data());
    res.connector_id_ptr = reinterpret_cast<uint64_t>(ids.data() + res.count_crtcs);
    res.encoder_id_ptr = reinterpret_cast<uint64_t>(ids.data() + res.count_crtcs + res.count_connectors);
    return drm_ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) == 0;
}

// 找首个已连接且含可用模式的 connector，取其首选模式；找不到返回 false。
bool pick_connector(int fd, const drm_mode_card_res& res, uint32_t& connector_id,
                    drm_mode_modeinfo& mode) {
    const uint32_t* connectors = reinterpret_cast<const uint32_t*>(res.connector_id_ptr);
    for (uint32_t i = 0; i < res.count_connectors; ++i) {
        drm_mode_get_connector probe{};
        probe.connector_id = connectors[i];
        if (drm_ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &probe) != 0) continue;
        if (probe.connection != DRM_MODE_CONNECTED || probe.count_modes == 0) continue;
        std::vector<drm_mode_modeinfo> modes(probe.count_modes);
        drm_mode_get_connector full{};
        full.connector_id = connectors[i];
        full.count_modes = probe.count_modes;
        full.modes_ptr = reinterpret_cast<uint64_t>(modes.data());
        if (drm_ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &full) != 0) continue;
        connector_id = connectors[i];
        mode = modes[0];
        return true;
    }
    return false;
}

// 优先取 connector 绑定 encoder 的 crtc，否则用第一个 crtc；不可用时返回 0。
uint32_t pick_crtc(int fd, const drm_mode_card_res& res, uint32_t connector_id) {
    drm_mode_get_connector conn{};
    conn.connector_id = connector_id;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &conn) == 0 && conn.encoder_id != 0) {
        drm_mode_get_encoder enc{};
        enc.encoder_id = conn.encoder_id;
        if (drm_ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &enc) == 0 && enc.crtc_id != 0)
            return enc.crtc_id;
    }
    const uint32_t* crtcs = reinterpret_cast<const uint32_t*>(res.crtc_id_ptr);
    return res.count_crtcs > 0 ? crtcs[0] : 0;
}

bool create_fb(int fd, int width, int height, uint32_t& fb, uint32_t& handle,
               uint32_t& pitch, size_t& size, void*& map) {
    drm_mode_create_dumb create{};
    create.width = width;
    create.height = height;
    create.bpp = 32;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) return false;
    handle = create.handle;
    pitch = create.pitch;
    size = create.size;

    drm_mode_map_dumb map_request{};
    map_request.handle = handle;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map_request) != 0) return false;
    map = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map_request.offset);
    if (map == MAP_FAILED) {
        map = nullptr;
        return false;
    }

    drm_mode_fb_cmd2 add{};
    add.width = width;
    add.height = height;
    add.pixel_format = DRM_FORMAT_XRGB8888;
    add.handles[0] = handle;
    add.pitches[0] = pitch;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &add) != 0) return false;
    fb = add.fb_id;
    return true;
}

// 等待上一次 page flip 完成；事件队列中的 flip complete 事件在此消费。
bool wait_flip(Display* d, int timeout_ms) {
    if (!d->flip_pending) return true;
    for (;;) {
        pollfd descriptor{d->fd, POLLIN, 0};
        int ready;
        do ready = poll(&descriptor, 1, timeout_ms); while (ready < 0 && errno == EINTR);
        if (ready <= 0) return false;
        drm_event_vblank event{};
        const ssize_t got = read(d->fd, &event, sizeof(event));
        if (got < static_cast<ssize_t>(sizeof(drm_event))) return false;
        if (event.base.type == DRM_EVENT_FLIP_COMPLETE) {
            d->flip_pending = false;
            return true;
        }
        // 其他内核事件继续消费直到 flip complete 或超时
    }
}

} // namespace

Display* display_open(const char* device) {
    const int fd = open(device, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        warn("open", errno);
        return nullptr;
    }

    drm_mode_card_res res;
    std::vector<uint32_t> ids;
    if (!get_card_res(fd, res, ids)) {
        warn("MODE_GETRESOURCES", errno);
        close(fd);
        return nullptr;
    }

    uint32_t connector = 0;
    drm_mode_modeinfo mode{};
    if (!pick_connector(fd, res, connector, mode)) {
        std::fprintf(stderr, "display: no connected connector with modes on %s\n", device);
        close(fd);
        return nullptr;
    }
    const uint32_t crtc = pick_crtc(fd, res, connector);
    if (crtc == 0) {
        std::fprintf(stderr, "display: no usable crtc on %s\n", device);
        close(fd);
        return nullptr;
    }

    Display* d = new Display();
    d->fd = fd;
    d->crtc = crtc;
    d->connector = connector;
    d->mode = mode;
    d->screen_width = mode.hdisplay;
    d->screen_height = mode.vdisplay;
    for (int i = 0; i < 2; ++i) {
        if (!create_fb(fd, d->screen_width, d->screen_height, d->fbs[i], d->handles[i],
                       d->pitches[i], d->sizes[i], d->maps[i])) {
            warn("dumb framebuffer", errno);
            display_close(d);
            return nullptr;
        }
        std::memset(d->maps[i], 0, d->sizes[i]);
    }

    uint32_t connector_id = connector;
    drm_mode_crtc set{};
    set.set_connectors_ptr = reinterpret_cast<uint64_t>(&connector_id);
    set.count_connectors = 1;
    set.crtc_id = crtc;
    set.fb_id = d->fbs[0];
    set.x = 0;
    set.y = 0;
    set.mode_valid = 1;
    set.mode = mode;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &set) != 0) {
        warn("MODE_SETCRTC", errno);
        display_close(d);
        return nullptr;
    }
    std::fprintf(stderr, "display: %s %dx%d@%dHz crtc=%u connector=%u fb=%u\n",
                 device, mode.hdisplay, mode.vdisplay, mode.vrefresh, crtc, connector, d->fbs[0]);
    return d;
}

int display_width(const Display* d) { return d ? d->screen_width : 0; }
int display_height(const Display* d) { return d ? d->screen_height : 0; }

bool display_show(Display* d, const uint8_t* bgr, int width, int height) {
    if (!d || !bgr || width <= 0 || height <= 0) return false;
    if (!wait_flip(d, 250)) {
        std::fprintf(stderr, "display: previous flip not completed, frame dropped\n");
        return false;
    }
    const int back = 1 - d->front;

    const cv::Mat source(height, width, CV_8UC3, const_cast<uint8_t*>(bgr));
    // 保持源纵横比的 letterbox 缩放，居中显示，黑边填充（如 800x600 画面在 720x1280 竖屏上）
    const double scale = std::min(static_cast<double>(d->screen_width) / width,
                                  static_cast<double>(d->screen_height) / height);
    const int target_w = std::max(1, static_cast<int>(width * scale));
    const int target_h = std::max(1, static_cast<int>(height * scale));
    const int offset_x = (d->screen_width - target_w) / 2;
    const int offset_y = (d->screen_height - target_h) / 2;

    uint8_t* base = static_cast<uint8_t*>(d->maps[back]);
    const uint32_t pitch = d->pitches[back];
    std::memset(base, 0, d->sizes[back]);  // 黑边
    if (target_w != width || target_h != height) {
        cv::Mat resized;
        cv::resize(source, resized, cv::Size(target_w, target_h), 0, 0, cv::INTER_LINEAR);
        for (int y = 0; y < target_h; ++y) {
            const uint8_t* row = resized.ptr(y);
            uint32_t* out = reinterpret_cast<uint32_t*>(
                base + static_cast<size_t>(offset_y + y) * pitch + static_cast<size_t>(offset_x) * 4u);
            for (int x = 0; x < target_w; ++x) {
                const uint8_t b = row[3 * x], g = row[3 * x + 1], r = row[3 * x + 2];
                out[x] = 0xFF000000u | (static_cast<uint32_t>(r) << 16)
                       | (static_cast<uint32_t>(g) << 8) | b;
            }
        }
    } else {
        for (int y = 0; y < d->screen_height; ++y) {
            const uint8_t* row = source.ptr(y);
            uint32_t* out = reinterpret_cast<uint32_t*>(base + static_cast<size_t>(y) * pitch);
            for (int x = 0; x < d->screen_width; ++x) {
                const uint8_t b = row[3 * x], g = row[3 * x + 1], r = row[3 * x + 2];
                out[x] = 0xFF000000u | (static_cast<uint32_t>(r) << 16)
                       | (static_cast<uint32_t>(g) << 8) | b;
            }
        }
    }

    drm_mode_crtc_page_flip flip{};
    flip.crtc_id = d->crtc;
    flip.fb_id = d->fbs[back];
    flip.flags = DRM_MODE_PAGE_FLIP_EVENT;
    if (drm_ioctl(d->fd, DRM_IOCTL_MODE_PAGE_FLIP, &flip) != 0) {
        warn("MODE_PAGE_FLIP", errno);
        return false;
    }
    d->front = back;
    d->flip_pending = true;
    return true;
}

void display_close(Display* d) {
    if (!d) return;
    if (d->fd >= 0) {
        wait_flip(d, 250);
        for (int i = 0; i < 2; ++i) {
            if (d->maps[i]) munmap(d->maps[i], d->sizes[i]);
            if (d->fbs[i]) {
                uint32_t fb = d->fbs[i];
                if (drm_ioctl(d->fd, DRM_IOCTL_MODE_RMFB, &fb) != 0) warn("MODE_RMFB", errno);
            }
            if (d->handles[i]) {
                drm_mode_destroy_dumb destroy{};
                destroy.handle = d->handles[i];
                if (drm_ioctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy) != 0)
                    warn("MODE_DESTROY_DUMB", errno);
            }
        }
        close(d->fd);
    }
    delete d;
}
