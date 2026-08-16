/*
 * 文件作用：S7.4 屏显链路冒烟程序。不依赖 libdrm/OpenCV，只使用内核 uapi 头与裸 ioctl，
 * 在 DRM/KMS 显示屏上显示随时间变化的渐变色块，验证屏是否被内核识别、分辨率/刷新率
 * 是否可用以及 KMS 点亮链路是否正常。是屏显观测后端（s7_4_display.cpp）的前置验证。
 * 用法：s7_4_drm_smoke [DEVICE] [SECONDS]；默认 /dev/dri/card0、10 秒。
 * 输出：connector 状态、全部可用模式、crtc/fb 信息与每秒帧计数；退出码 0 表示链路可用。
 */
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_fourcc.h>

// SDK 内核 uapi（老接口）未提供连接状态宏；1/2/3 为长期稳定 ABI 值
#ifndef DRM_MODE_CONNECTED
#define DRM_MODE_CONNECTED 1
#define DRM_MODE_DISCONNECTED 2
#define DRM_MODE_UNKNOWNCONNECTION 3
#endif

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

int drm_ioctl(int fd, unsigned long request, void* arg) {
    int result;
    do result = ioctl(fd, request, arg); while (result != 0 && errno == EINTR);
    return result;
}

void fail(const char* what) {
    std::fprintf(stderr, "smoke: %s: %s\n", what, std::strerror(errno));
    std::exit(1);
}

struct ConnectorInfo {
    uint32_t id{};
    int type{};
    int connection{};
    std::vector<drm_mode_modeinfo> modes;
};

bool get_connector(int fd, uint32_t id, ConnectorInfo& info) {
    drm_mode_get_connector probe{};
    probe.connector_id = id;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &probe) != 0) return false;
    info.id = id;
    info.type = probe.connector_type;
    info.connection = probe.connection;
    if (probe.count_modes == 0) return true;
    info.modes.resize(probe.count_modes);
    drm_mode_get_connector full{};
    full.connector_id = id;
    full.count_modes = probe.count_modes;
    full.modes_ptr = reinterpret_cast<uint64_t>(info.modes.data());
    return drm_ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &full) == 0;
}

} // namespace

int main(int argc, char** argv) {
    const char* device = argc > 1 ? argv[1] : "/dev/dri/card0";
    const int seconds = argc > 2 ? std::atoi(argv[2]) : 10;
    if (seconds <= 0) {
        std::fprintf(stderr, "usage: s7_4_drm_smoke [DEVICE] [SECONDS]\n");
        return 2;
    }

    const int fd = open(device, O_RDWR | O_CLOEXEC);
    if (fd < 0) fail("open");

    drm_mode_card_res res{};
    if (drm_ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) != 0) fail("MODE_GETRESOURCES");
    std::vector<uint32_t> ids(res.count_crtcs + res.count_connectors + res.count_encoders);
    res.crtc_id_ptr = reinterpret_cast<uint64_t>(ids.data());
    res.connector_id_ptr = reinterpret_cast<uint64_t>(ids.data() + res.count_crtcs);
    res.encoder_id_ptr = reinterpret_cast<uint64_t>(ids.data() + res.count_crtcs + res.count_connectors);
    if (drm_ioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res) != 0) fail("MODE_GETRESOURCES");
    std::fprintf(stderr, "smoke: %s: %u crtc, %u connector, %u encoder\n",
                 device, res.count_crtcs, res.count_connectors, res.count_encoders);

    const uint32_t* connector_ids = reinterpret_cast<const uint32_t*>(res.connector_id_ptr);
    ConnectorInfo picked;
    for (uint32_t i = 0; i < res.count_connectors; ++i) {
        ConnectorInfo info;
        if (!get_connector(fd, connector_ids[i], info)) continue;
        std::fprintf(stderr, "smoke: connector %u type=%d connection=%d modes=%zu\n",
                     info.id, info.type, info.connection, info.modes.size());
        for (const auto& m : info.modes)
            std::fprintf(stderr, "  mode %ux%u@%uHz\n", m.hdisplay, m.vdisplay, m.vrefresh);
        if (info.connection == DRM_MODE_CONNECTED && !info.modes.empty() && picked.id == 0)
            picked = std::move(info);
    }
    if (picked.id == 0) {
        std::fprintf(stderr, "smoke: no connected connector with modes on %s\n", device);
        close(fd);
        return 2;
    }
    const drm_mode_modeinfo& mode = picked.modes[0];
    const int w = mode.hdisplay, h = mode.vdisplay;

    uint32_t crtc = 0;
    {
        drm_mode_get_connector conn{};
        conn.connector_id = picked.id;
        if (drm_ioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &conn) == 0 && conn.encoder_id != 0) {
            drm_mode_get_encoder enc{};
            enc.encoder_id = conn.encoder_id;
            if (drm_ioctl(fd, DRM_IOCTL_MODE_GETENCODER, &enc) == 0) crtc = enc.crtc_id;
        }
        if (crtc == 0) {
            const uint32_t* crtcs = reinterpret_cast<const uint32_t*>(res.crtc_id_ptr);
            crtc = res.count_crtcs > 0 ? crtcs[0] : 0;
        }
    }
    if (crtc == 0) {
        std::fprintf(stderr, "smoke: no usable crtc\n");
        close(fd);
        return 2;
    }

    drm_mode_create_dumb create{};
    create.width = w;
    create.height = h;
    create.bpp = 32;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) fail("MODE_CREATE_DUMB");
    drm_mode_map_dumb map_request{};
    map_request.handle = create.handle;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map_request) != 0) fail("MODE_MAP_DUMB");
    void* map = mmap(nullptr, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map_request.offset);
    if (map == MAP_FAILED) fail("mmap");

    drm_mode_fb_cmd2 add{};
    add.width = w;
    add.height = h;
    add.pixel_format = DRM_FORMAT_XRGB8888;
    add.handles[0] = create.handle;
    add.pitches[0] = create.pitch;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_ADDFB2, &add) != 0) fail("MODE_ADDFB2");

    uint32_t connector_id = picked.id;
    drm_mode_crtc set{};
    set.set_connectors_ptr = reinterpret_cast<uint64_t>(&connector_id);
    set.count_connectors = 1;
    set.crtc_id = crtc;
    set.fb_id = add.fb_id;
    set.x = 0;
    set.y = 0;
    set.mode_valid = 1;
    set.mode = mode;
    if (drm_ioctl(fd, DRM_IOCTL_MODE_SETCRTC, &set) != 0) fail("MODE_SETCRTC");
    std::fprintf(stderr, "smoke: displaying %dx%d@%dHz on crtc %u fb %u for %d s\n",
                 w, h, mode.vrefresh, crtc, add.fb_id, seconds);

    for (int t = 0; t < seconds; ++t) {
        for (int y = 0; y < h; ++y) {
            uint32_t* out = reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(map)
                                                        + static_cast<size_t>(y) * create.pitch);
            for (int x = 0; x < w; ++x) {
                const uint8_t r = static_cast<uint8_t>(255 * x / (w - 1));
                const uint8_t g = static_cast<uint8_t>(255 * y / (h - 1));
                const uint8_t b = static_cast<uint8_t>((t * 40) & 0xFF);
                out[x] = 0xFF000000u | (static_cast<uint32_t>(r) << 16)
                       | (static_cast<uint32_t>(g) << 8) | b;
            }
        }
        std::fprintf(stderr, "smoke: second %d shown\n", t + 1);
        usleep(1000000);
    }

    munmap(map, create.size);
    close(fd);
    std::fprintf(stderr, "smoke: ok, display link usable\n");
    return 0;
}
