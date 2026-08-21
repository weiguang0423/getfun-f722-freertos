#include <rknn_api.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "s7_4_display.hpp"
#ifdef S7_5_GESTURE
#include "../s7_5/s7_5_gesture.hpp"
#endif
#ifdef S7_6_VIRTUAL_RC
#include "../s7_6/s7_6_serial.hpp"
#endif

#include <cerrno>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/videodev2.h>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

struct Detection {
    float score{};
    cv::Vec4f box{};
    cv::Vec2f keypoints[7]{};
};

struct Timings {
    double preprocess_ms{};
    double detector_ms{};
    double postprocess_ms{};
    double landmark_ms{};
};

struct LiveFrame {
    const unsigned char* data{};
    size_t bytes{};
    uint32_t buffer_index{};
    uint32_t sequence{};
    uint64_t capture_timestamp_us{};
    bool monotonic_timestamp{};
    uint64_t dropped_frames{};
};

struct LiveMeta {
    uint32_t sequence{};
    uint64_t capture_timestamp_us{};
    bool monotonic_timestamp{};
    uint64_t dropped_frames{};
    double max_latency_ms{};
};

static volatile std::sig_atomic_t running = 1;

static void stop(int) { running = 0; }

static uint64_t now_us(bool monotonic) {
    if (!monotonic) {
        timeval value{};
        if (gettimeofday(&value, nullptr) != 0) throw std::runtime_error("gettimeofday failed");
        return static_cast<uint64_t>(value.tv_sec) * 1000000u + value.tv_usec;
    }
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) throw std::runtime_error("clock_gettime failed");
    return static_cast<uint64_t>(value.tv_sec) * 1000000u + value.tv_nsec / 1000u;
}

static uint64_t sequence_gap(uint32_t previous, uint32_t current) {
    return current > previous ? static_cast<uint64_t>(current - previous - 1u) : 0u;
}

class Camera {
public:
    Camera(const char* device, int width, int height, int fps, int vblank) {
        fd_ = open(device, O_RDWR | O_NONBLOCK);
        if (fd_ < 0) fail("open camera");

        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        format.fmt.pix_mp.width = width;
        format.fmt.pix_mp.height = height;
        format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        format.fmt.pix_mp.field = V4L2_FIELD_NONE;
        format.fmt.pix_mp.num_planes = 1;
        call(VIDIOC_S_FMT, &format, "VIDIOC_S_FMT");
        if (format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12 || format.fmt.pix_mp.num_planes != 1)
            throw std::runtime_error("camera did not accept NV12");
        width_ = static_cast<int>(format.fmt.pix_mp.width);
        height_ = static_cast<int>(format.fmt.pix_mp.height);

        v4l2_streamparm rate{};
        rate.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        rate.parm.capture.timeperframe.numerator = 1;
        rate.parm.capture.timeperframe.denominator = fps;
        int rate_result;
        do rate_result = ioctl(fd_, VIDIOC_S_PARM, &rate); while (rate_result != 0 && errno == EINTR);
        if (rate_result != 0 && errno != ENOTTY && errno != EINVAL) fail("VIDIOC_S_PARM");

        v4l2_requestbuffers request{};
        request.count = 4;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        request.memory = V4L2_MEMORY_MMAP;
        call(VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS");
        if (request.count < 2) throw std::runtime_error("camera returned fewer than two buffers");
        buffers_.resize(request.count);
        for (uint32_t i = 0; i < request.count; ++i) {
            v4l2_plane planes[VIDEO_MAX_PLANES]{};
            v4l2_buffer buffer{};
            buffer.type = request.type;
            buffer.memory = request.memory;
            buffer.index = i;
            buffer.length = VIDEO_MAX_PLANES;
            buffer.m.planes = planes;
            call(VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF");
            if (buffer.length != 1) throw std::runtime_error("camera returned multiple memory planes");
            buffers_[i].size = planes[0].length;
            buffers_[i].data = mmap(nullptr, planes[0].length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd_, planes[0].m.mem_offset);
            if (buffers_[i].data == MAP_FAILED) fail("mmap camera buffer");
            call(VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
        }
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        call(VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
        streaming_ = true;
        if (vblank >= 0) {
            v4l2_control control{};
            control.id = V4L2_CID_VBLANK;
            call(VIDIOC_G_CTRL, &control, "VIDIOC_G_CTRL VBLANK");
            original_vblank_ = control.value;
            control.value = vblank;
            call(VIDIOC_S_CTRL, &control, "VIDIOC_S_CTRL VBLANK");
            restore_vblank_ = true;
        }
    }

    ~Camera() {
        if (streaming_) {
            int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }
        if (restore_vblank_) {
            v4l2_control control{};
            control.id = V4L2_CID_VBLANK;
            control.value = original_vblank_;
            ioctl(fd_, VIDIOC_S_CTRL, &control);
        }
        for (const auto& buffer : buffers_)
            if (buffer.data && buffer.data != MAP_FAILED) munmap(buffer.data, buffer.size);
        if (fd_ >= 0) close(fd_);
    }

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    bool latest(LiveFrame& frame, int timeout_ms) {
        pollfd descriptor{fd_, POLLIN | POLLERR | POLLHUP, 0};
        int ready;
        do ready = poll(&descriptor, 1, timeout_ms); while (ready < 0 && errno == EINTR);
        if (ready < 0) fail("poll camera");
        if (ready == 0) return false;
        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
            throw std::runtime_error("camera disconnected");

        v4l2_plane latest_planes[VIDEO_MAX_PLANES]{};
        v4l2_buffer latest{};
        bool found = false;
        uint64_t discarded = 0;
        for (;;) {
            v4l2_plane next_planes[VIDEO_MAX_PLANES]{};
            v4l2_buffer next{};
            next.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            next.memory = V4L2_MEMORY_MMAP;
            next.length = VIDEO_MAX_PLANES;
            next.m.planes = next_planes;
            if (ioctl(fd_, VIDIOC_DQBUF, &next) != 0) {
                if (errno == EAGAIN) break;
                if (errno == EINTR) continue;
                fail("VIDIOC_DQBUF");
            }
            if (found) {
                call(VIDIOC_QBUF, &latest, "VIDIOC_QBUF stale");
                ++discarded;
            }
            latest = next;
            std::copy(next_planes, next_planes + next.length, latest_planes);
            latest.m.planes = latest_planes;
            found = true;
        }
        if (!found) return false;
        if (have_sequence_) dropped_ += sequence_gap(last_sequence_, latest.sequence);
        else dropped_ += discarded;
        last_sequence_ = latest.sequence;
        have_sequence_ = true;
        frame.data = static_cast<const unsigned char*>(buffers_.at(latest.index).data)
                   + latest_planes[0].data_offset;
        frame.bytes = latest_planes[0].bytesused;
        frame.buffer_index = latest.index;
        frame.sequence = latest.sequence;
        frame.capture_timestamp_us = static_cast<uint64_t>(latest.timestamp.tv_sec) * 1000000u
                                   + latest.timestamp.tv_usec;
        frame.monotonic_timestamp = (latest.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) != 0;
        frame.dropped_frames = dropped_;
        return true;
    }

    void release(const LiveFrame& frame) {
        v4l2_plane planes[1]{};
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = frame.buffer_index;
        buffer.length = 1;
        buffer.m.planes = planes;
        call(VIDIOC_QBUF, &buffer, "VIDIOC_QBUF current");
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    struct Buffer { void* data{}; size_t size{}; };

    [[noreturn]] static void fail(const char* operation) {
        throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
    }

    void call(unsigned long request, void* value, const char* operation) {
        int result;
        do result = ioctl(fd_, request, value); while (result != 0 && errno == EINTR);
        if (result != 0) fail(operation);
    }

    int fd_{-1};
    int width_{};
    int height_{};
    bool streaming_{};
    bool have_sequence_{};
    bool restore_vblank_{};
    int original_vblank_{};
    uint32_t last_sequence_{};
    uint64_t dropped_{};
    std::vector<Buffer> buffers_;
};

static double ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

static std::string json_string(const std::string& value) {
    std::string out = "\"";
    for (unsigned char c : value) {
        if (c == '\\' || c == '"') out += '\\';
        if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else if (c >= 0x20) out += static_cast<char>(c);
    }
    return out + '"';
}

static std::vector<unsigned char> read_file(const fs::path& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream) throw std::runtime_error("cannot open model: " + path.string());
    std::vector<unsigned char> data(static_cast<size_t>(stream.tellg()));
    stream.seekg(0);
    if (!stream.read(reinterpret_cast<char*>(data.data()), data.size()))
        throw std::runtime_error("cannot read model: " + path.string());
    return data;
}

class Model {
public:
    explicit Model(const fs::path& path) : data_(read_file(path)) {
        check(rknn_init(&ctx_, data_.data(), data_.size(), 0, nullptr), "rknn_init");
        check(rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_, sizeof(io_)), "query io");
        if (io_.n_input != 1) throw std::runtime_error("model must have one input");
        rknn_tensor_attr input{};
        check(rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input, sizeof(input)), "query input");
        std::cerr << "input name=" << input.name << " elems=" << input.n_elems
                  << " type=" << input.type << " fmt=" << input.fmt << '\n';
        outputs_.resize(io_.n_output);
        for (uint32_t i = 0; i < io_.n_output; ++i) {
            outputs_[i].index = i;
            check(rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &outputs_[i], sizeof(outputs_[i])), "query output");
            std::cerr << "output[" << i << "] name=" << outputs_[i].name
                      << " elems=" << outputs_[i].n_elems << '\n';
        }
    }

    ~Model() { if (ctx_) rknn_destroy(ctx_); }
    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;

    std::vector<std::vector<float>> run(const cv::Mat& rgb) {
        if (!rgb.isContinuous() || rgb.type() != CV_8UC3)
            throw std::runtime_error("model input must be continuous RGB uint8");
        rknn_input input{};
        input.index = 0;
        input.buf = rgb.data;
        input.size = rgb.total() * rgb.elemSize();
        input.type = RKNN_TENSOR_UINT8;
        input.fmt = RKNN_TENSOR_NHWC;
        check(rknn_inputs_set(ctx_, 1, &input), "inputs_set");
        check(rknn_run(ctx_, nullptr), "run");

        std::vector<rknn_output> raw(io_.n_output);
        for (uint32_t i = 0; i < io_.n_output; ++i) {
            raw[i].index = i;
            raw[i].want_float = 1;
        }
        check(rknn_outputs_get(ctx_, io_.n_output, raw.data(), nullptr), "outputs_get");
        std::vector<std::vector<float>> result(io_.n_output);
        for (uint32_t i = 0; i < io_.n_output; ++i) {
            const auto* begin = static_cast<const float*>(raw[i].buf);
            result[i].assign(begin, begin + raw[i].size / sizeof(float));
        }
        rknn_outputs_release(ctx_, io_.n_output, raw.data());
        return result;
    }

private:
    static void check(int ret, const char* operation) {
        if (ret != RKNN_SUCC) throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(ret));
    }
    rknn_context ctx_{};
    rknn_input_output_num io_{};
    std::vector<unsigned char> data_;
    std::vector<rknn_tensor_attr> outputs_;
};

static std::vector<cv::Vec2f> make_anchors() {
    std::vector<cv::Vec2f> result;
    const int strides[] = {8, 16, 16, 16};
    for (int layer = 0; layer < 4;) {
        int end = layer;
        int per_cell = 0;
        while (end < 4 && strides[end] == strides[layer]) { per_cell += 2; ++end; }
        const int grid = (192 + strides[layer] - 1) / strides[layer];
        for (int y = 0; y < grid; ++y)
            for (int x = 0; x < grid; ++x)
                for (int n = 0; n < per_cell; ++n)
                    result.emplace_back((x + 0.5f) / grid, (y + 0.5f) / grid);
        layer = end;
    }
    if (result.size() != 2016) throw std::runtime_error("anchor generation failed");
    return result;
}

static float iou(const Detection& a, const Detection& b) {
    const float w = std::max(0.f, std::min(a.box[2], b.box[2]) - std::max(a.box[0], b.box[0]));
    const float h = std::max(0.f, std::min(a.box[3], b.box[3]) - std::max(a.box[1], b.box[1]));
    const float intersection = w * h;
    const float aa = std::max(0.f, a.box[2] - a.box[0]) * std::max(0.f, a.box[3] - a.box[1]);
    const float bb = std::max(0.f, b.box[2] - b.box[0]) * std::max(0.f, b.box[3] - b.box[1]);
    return aa + bb > intersection ? intersection / (aa + bb - intersection) : 0.f;
}

static std::vector<Detection> decode(const std::vector<float>& boxes, const std::vector<float>& logits) {
    if (boxes.size() != 2016 * 18 || logits.size() != 2016)
        throw std::runtime_error("unexpected detector output shape");
    static const auto anchors = make_anchors();
    std::vector<Detection> pending;
    for (size_t i = 0; i < logits.size(); ++i) {
        const float score = 1.f / (1.f + std::exp(-std::clamp(logits[i], -100.f, 100.f)));
        if (score < .75f) continue;
        const float* raw = boxes.data() + i * 18;
        Detection d;
        d.score = score;
        const float cx = raw[0] / 192.f + anchors[i][0];
        const float cy = raw[1] / 192.f + anchors[i][1];
        d.box = {cx - raw[2] / 384.f, cy - raw[3] / 384.f,
                 cx + raw[2] / 384.f, cy + raw[3] / 384.f};
        for (int k = 0; k < 7; ++k)
            d.keypoints[k] = {raw[4 + 2 * k] / 192.f + anchors[i][0],
                              raw[5 + 2 * k] / 192.f + anchors[i][1]};
        pending.push_back(d);
    }
    std::sort(pending.begin(), pending.end(), [](const auto& a, const auto& b) { return a.score > b.score; });
    std::vector<Detection> result;
    while (!pending.empty()) {
        const Detection first = pending.front();
        std::vector<Detection> overlap, remaining;
        for (const auto& item : pending) (iou(first, item) > .3f ? overlap : remaining).push_back(item);
        pending.swap(remaining);
        Detection merged;
        float weights = 0.f;
        for (const auto& item : overlap) {
            weights += item.score;
            merged.score += item.score;
            merged.box += item.box * item.score;
            for (int k = 0; k < 7; ++k) merged.keypoints[k] += item.keypoints[k] * item.score;
        }
        merged.score /= overlap.size();
        merged.box *= 1.f / weights;
        for (auto& point : merged.keypoints) point *= 1.f / weights;
        result.push_back(merged);
    }
    return result;
}

static cv::Mat detector_input(const cv::Mat& rgb, float& scale, int& left, int& top) {
    scale = 192.f / std::max(rgb.rows, rgb.cols);
    const int width = std::max(1, static_cast<int>(rgb.cols * scale));
    const int height = std::max(1, static_cast<int>(rgb.rows * scale));
    left = (192 - width) / 2;
    top = (192 - height) / 2;
    cv::Mat resized, out = cv::Mat::zeros(192, 192, CV_8UC3);
    cv::resize(rgb, resized, {width, height}, 0, 0, cv::INTER_LINEAR);
    resized.copyTo(out({left, top, width, height}));
    return out;
}

static cv::Mat roi_input(const cv::Mat& rgb, const Detection& d, cv::Mat& inverse,
                         cv::Vec4f& roi_info) {
    const float width = static_cast<float>(rgb.cols), height = static_cast<float>(rgb.rows);
    const cv::Vec4f box(d.box[0] * width, d.box[1] * height, d.box[2] * width, d.box[3] * height);
    cv::Point2f center((box[0] + box[2]) / 2.f, (box[1] + box[3]) / 2.f);
    const float box_size = box[2] - box[0];
    center.y -= .5f * box_size;
    const float size = box_size * 2.6f;
    const cv::Point2f p0(d.keypoints[0][0] * width, d.keypoints[0][1] * height);
    const cv::Point2f p2(d.keypoints[2][0] * width, d.keypoints[2][1] * height);
    const float theta = std::atan2(p0.y - p2.y, p0.x - p2.x) - static_cast<float>(CV_PI / 2);
    const float c = std::cos(theta), s = std::sin(theta);
    const cv::Point2f units[] = {{-1,-1}, {-1,1}, {1,-1}};
    cv::Point2f corners[3];
    for (int i = 0; i < 3; ++i) {
        const float x = units[i].x * size / 2.f, y = units[i].y * size / 2.f;
        corners[i] = {x * c - y * s + center.x, x * s + y * c + center.y};
    }
    const cv::Point2f target[] = {{0,0}, {0,223}, {223,0}};
    const cv::Mat affine = cv::getAffineTransform(corners, target);
    cv::invertAffineTransform(affine, inverse);
    cv::Mat roi;
    cv::warpAffine(rgb, roi, affine, {224,224});
    roi_info = {center.x, center.y, size, theta};
    return roi;
}

static void print_array(std::ostream& out, const float* values, size_t count) {
    out << '[';
    for (size_t i = 0; i < count; ++i) {
        if (i) out << ',';
        out << values[i];
    }
    out << ']';
}

static void print_record(const std::string& source, int frame_index, const cv::Mat& bgr,
                         Model& detector, Model& landmark, const LiveMeta* live = nullptr,
                         const char* annotate_dir = nullptr, int annotate_every = 5,
                         Display* display = nullptr
#ifdef S7_5_GESTURE
                         , s7_5::GestureStateMachine* gesture_state = nullptr
#endif
#ifdef S7_6_VIRTUAL_RC
                         , s7_6::SerialLink* rc_link = nullptr
#endif
                         ) {
    Timings timing;
    auto start = Clock::now();
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    float scale;
    int left, top;
    cv::Mat input = detector_input(rgb, scale, left, top);
    timing.preprocess_ms = ms(start);

    start = Clock::now();
    auto detector_outputs = detector.run(input);
    timing.detector_ms = ms(start);
    start = Clock::now();
    auto detections = decode(detector_outputs.at(0), detector_outputs.at(1));

    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << "{\"source\":" << json_string(source)
        << ",\"frame_index\":" << frame_index;
    bool valid = false;
    const char* reject_reason = "no_hand";
    cv::Mat annotated;
#ifdef S7_5_GESTURE
    s7_5::GestureObservation gesture_observation;
    s7_5::GestureSnapshot gesture_snapshot;
#endif
    if (detections.empty()) {
        timing.postprocess_ms = ms(start);
    } else {
        auto d = detections.front();
        d.box[0] = (d.box[0] * 192 - left) / scale / bgr.cols;
        d.box[2] = (d.box[2] * 192 - left) / scale / bgr.cols;
        d.box[1] = (d.box[1] * 192 - top) / scale / bgr.rows;
        d.box[3] = (d.box[3] * 192 - top) / scale / bgr.rows;
        for (auto& p : d.keypoints) {
            p[0] = (p[0] * 192 - left) / scale / bgr.cols;
            p[1] = (p[1] * 192 - top) / scale / bgr.rows;
        }
        cv::Mat inverse;
        cv::Vec4f roi_info;
        cv::Mat roi = roi_input(rgb, d, inverse, roi_info);
        timing.postprocess_ms = ms(start);
        start = Clock::now();
        auto landmark_outputs = landmark.run(roi);
        timing.landmark_ms = ms(start);
        start = Clock::now();
        if (landmark_outputs.size() < 3 || landmark_outputs[0].size() != 63)
            throw std::runtime_error("unexpected landmark output shape/order");
        const float presence = landmark_outputs[1][0];
        const float handedness = landmark_outputs[2][0];
        std::vector<float> image_landmarks(63);
        for (int i = 0; i < 21; ++i) {
            const float x = landmark_outputs[0][3*i], y = landmark_outputs[0][3*i+1];
            image_landmarks[3*i] = static_cast<float>((inverse.at<double>(0,0)*x + inverse.at<double>(0,1)*y + inverse.at<double>(0,2)) / bgr.cols);
            image_landmarks[3*i+1] = static_cast<float>((inverse.at<double>(1,0)*x + inverse.at<double>(1,1)*y + inverse.at<double>(1,2)) / bgr.rows);
            image_landmarks[3*i+2] = landmark_outputs[0][3*i+2];
        }
        timing.postprocess_ms += ms(start);
        out << ",\"detector_score\":" << d.score << ",\"bbox_xyxy_normalized\":";
        print_array(out, d.box.val, 4);
        out << ",\"detector_keypoints_7x2\":";
        print_array(out, reinterpret_cast<float*>(d.keypoints), 14);
        out << ",\"roi_center_scale_rotation\":";
        print_array(out, roi_info.val, 4);
        out << ",\"presence_score\":" << presence << ",\"handedness_score\":" << handedness
            << ",\"landmarks_21x3_roi\":";
        print_array(out, landmark_outputs[0].data(), 63);
        out << ",\"landmarks_21x3_image\":";
        print_array(out, image_landmarks.data(), 63);
        valid = presence >= .5f;
        reject_reason = valid ? "" : "low_presence";
#ifdef S7_5_GESTURE
        if (valid) gesture_observation = s7_5::classify(image_landmarks.data(), presence);
        if (display || (annotate_dir && frame_index % annotate_every == 0)) {
#else
        if (annotate_dir && frame_index % annotate_every == 0) {
#endif
            annotated = bgr.clone();
            const cv::Point corner_a(static_cast<int>(d.box[0] * bgr.cols),
                                     static_cast<int>(d.box[1] * bgr.rows));
            const cv::Point corner_b(static_cast<int>(d.box[2] * bgr.cols),
                                     static_cast<int>(d.box[3] * bgr.rows));
            cv::rectangle(annotated, corner_a, corner_b, {0, 255, 0}, 2);
            static const int hand_connections[21][2] = {
                {0,1},{1,2},{2,3},{3,4}, {0,5},{5,6},{6,7},{7,8},
                {5,9},{9,10},{10,11},{11,12}, {9,13},{13,14},{14,15},{15,16},
                {13,17},{17,18},{18,19},{19,20}, {0,17}};
            for (const auto& edge : hand_connections) {
                const int a = edge[0], b = edge[1];
                const cv::Point pa(static_cast<int>(image_landmarks[3 * a] * bgr.cols),
                                   static_cast<int>(image_landmarks[3 * a + 1] * bgr.rows));
                const cv::Point pb(static_cast<int>(image_landmarks[3 * b] * bgr.cols),
                                   static_cast<int>(image_landmarks[3 * b + 1] * bgr.rows));
                cv::line(annotated, pa, pb, {0, 255, 255}, 1);
            }
            for (int i = 0; i < 21; ++i) {
                const cv::Point pt(static_cast<int>(image_landmarks[3 * i] * bgr.cols),
                                   static_cast<int>(image_landmarks[3 * i + 1] * bgr.rows));
                cv::circle(annotated, pt, 3, {0, 0, 255}, -1);
            }
#ifndef S7_5_GESTURE
            std::ostringstream label;
            label << "seq=" << frame_index << " valid=" << (valid ? 1 : 0)
                  << " presence=" << std::fixed << std::setprecision(2) << presence
                  << " handed=" << handedness;
            cv::putText(annotated, label.str(), {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        {0, 255, 0}, 1);
            const fs::path dir(annotate_dir);
            std::error_code ec;
            fs::create_directories(dir, ec);
            std::ostringstream name;
            name << "annot_" << std::setw(6) << std::setfill('0') << frame_index << ".jpg";
            if (!cv::imwrite((dir / name.str()).string(), annotated))
                std::cerr << "annotate write failed: " << name.str() << '\n';
#endif
        }
    }
    out << ",\"timing_ms\":{\"preprocess\":" << timing.preprocess_ms
        << ",\"detector\":" << timing.detector_ms << ",\"postprocess\":" << timing.postprocess_ms
        << ",\"landmark\":" << timing.landmark_ms << '}';
    if (live) {
        const uint64_t complete = now_us(live->monotonic_timestamp);
        const double latency = complete >= live->capture_timestamp_us
                             ? (complete - live->capture_timestamp_us) / 1000.0 : 0.0;
        if (latency > live->max_latency_ms) {
            valid = false;
            reject_reason = "deadline_miss";
        }
        out << ",\"sequence\":" << live->sequence
            << ",\"capture_timestamp_us\":" << live->capture_timestamp_us
            << ",\"complete_timestamp_us\":" << complete
            << ",\"timestamp_clock\":"
            << (live->monotonic_timestamp ? "\"monotonic\"" : "\"realtime\"")
            << ",\"end_to_end_ms\":" << latency
            << ",\"dropped_frames\":" << live->dropped_frames;
    }
#ifdef S7_5_GESTURE
    if (gesture_state) {
        gesture_snapshot = gesture_state->update(valid, valid ? gesture_observation
                                                              : s7_5::GestureObservation{},
                                                 now_us(true));
        out << ",\"gesture_id\":" << static_cast<unsigned>(gesture_snapshot.observed_id)
            << ",\"gesture_name\":" << json_string(s7_5::gesture_name(gesture_snapshot.observed_id))
            << ",\"gesture_confidence\":" << gesture_snapshot.observed_confidence
            << ",\"gesture_state\":" << json_string(s7_5::state_name(gesture_snapshot.state))
            << ",\"active_gesture_id\":" << static_cast<unsigned>(gesture_snapshot.active_id)
            << ",\"active_gesture_name\":" << json_string(s7_5::gesture_name(gesture_snapshot.active_id))
            << ",\"candidate_frames\":" << gesture_snapshot.candidate_frames;
#ifdef S7_6_VIRTUAL_RC
        if (rc_link && live) {
            const auto rc = rc_link->send(gesture_snapshot, live->sequence,
                                          live->monotonic_timestamp ? live->capture_timestamp_us : 0,
                                          now_us(true));
            out << ",\"virtual_rc_valid\":" << (rc.valid ? "true" : "false")
                << ",\"virtual_rc_source_sequence\":" << rc.source_sequence
                << ",\"virtual_rc_heartbeat\":" << rc.heartbeat
                << ",\"virtual_rc_send_timestamp_us\":" << rc.send_timestamp_us
                << ",\"virtual_rc_channels\":[" << rc.channels.roll << ',' << rc.channels.pitch
                << ',' << rc.channels.yaw << ',' << rc.channels.throttle << ',' << rc.channels.aux
                << ']';
        }
#endif
    }
#endif
    out << ",\"valid\":" << (valid ? "true" : "false")
        << ",\"reject_reason\":" << json_string(reject_reason) << "}\n";
    std::cout << out.str() << std::flush;
#ifdef S7_5_GESTURE
    if (gesture_state && (display || (annotate_dir && frame_index % annotate_every == 0))) {
        if (annotated.empty()) annotated = bgr.clone();
        std::ostringstream label;
        label << "seq=" << frame_index << " hand=" << (valid ? 1 : 0)
              << " observed=" << s7_5::gesture_name(gesture_snapshot.observed_id)
              << " conf=" << std::fixed << std::setprecision(2)
              << gesture_snapshot.observed_confidence;
        cv::putText(annotated, label.str(), {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    {0, 255, 0}, 1);
        std::ostringstream state_label;
        state_label << "state=" << s7_5::state_name(gesture_snapshot.state)
                    << " active=" << s7_5::gesture_name(gesture_snapshot.active_id)
                    << " frames=" << gesture_snapshot.candidate_frames;
        cv::putText(annotated, state_label.str(), {8, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    gesture_snapshot.state == s7_5::GestureState::ACTIVE
                        ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 1);
        if (annotate_dir && frame_index % annotate_every == 0) {
            const fs::path dir(annotate_dir);
            std::error_code ec;
            fs::create_directories(dir, ec);
            std::ostringstream name;
            name << "annot_" << std::setw(6) << std::setfill('0') << frame_index << ".jpg";
            if (!cv::imwrite((dir / name.str()).string(), annotated))
                std::cerr << "annotate write failed: " << name.str() << '\n';
        }
    }
#endif
    if (display) {
        if (annotated.empty())
            display_show(display, bgr.data, bgr.cols, bgr.rows);
        else
            display_show(display, annotated.data, annotated.cols, annotated.rows);
    }
}

static void print_invalid(const char* source, const char* reason, uint32_t sequence,
                          uint64_t dropped_frames
#ifdef S7_5_GESTURE
                          , s7_5::GestureStateMachine* gesture_state = nullptr
#endif
#ifdef S7_6_VIRTUAL_RC
                          , s7_6::SerialLink* rc_link = nullptr
#endif
                          ) {
    std::ostringstream out;
    out << "{\"source\":" << json_string(source) << ",\"frame_index\":" << sequence
        << ",\"sequence\":" << sequence << ",\"dropped_frames\":" << dropped_frames;
#ifdef S7_5_GESTURE
    if (gesture_state) {
        const auto snapshot = gesture_state->update(false, {}, now_us(true));
        out << ",\"gesture_id\":0,\"gesture_name\":\"UNKNOWN\",\"gesture_confidence\":0"
            << ",\"gesture_state\":" << json_string(s7_5::state_name(snapshot.state))
            << ",\"active_gesture_id\":0,\"active_gesture_name\":\"UNKNOWN\""
            << ",\"candidate_frames\":0";
#ifdef S7_6_VIRTUAL_RC
        if (rc_link) {
            const auto send_timestamp = now_us(true);
            const auto rc = rc_link->send(snapshot, sequence, send_timestamp, send_timestamp);
            out << ",\"virtual_rc_valid\":false,\"virtual_rc_heartbeat\":" << rc.heartbeat
                << ",\"virtual_rc_channels\":[0,0,0,0,0]";
        }
#endif
    }
#endif
    out << ",\"valid\":false,\"reject_reason\":" << json_string(reason) << "}\n";
    std::cout << out.str() << std::flush;
}

static int parse_int(const char* text) {
    int value{};
    const char* end = text + std::strlen(text);
    const auto result = std::from_chars(text, end, value);
    if (result.ec != std::errc{} || result.ptr != end)
        throw std::runtime_error("invalid integer argument: " + std::string(text));
    return value;
}

#ifdef S7_6_VIRTUAL_RC
static int run_uart_test(int argc, char** argv) {
    if (argc < 3 || argc > 5) {
        std::cerr << "usage: s7_6_live --uart-test DEVICE [FRAME_COUNT] [cycle|neutral|point|vsign|release]\n";
        return 2;
    }
    const int frame_count = argc >= 4 ? parse_int(argv[3]) : 20;
    if (frame_count <= 0) throw std::runtime_error("FRAME_COUNT must be positive");

    s7_6::SerialLink link(argv[2]);
    const s7_5::GestureSnapshot gestures[] = {
        {},
        {s7_5::GestureState::ACTIVE, s7_5::GestureId::OPEN_PALM,
         .95f, s7_5::GestureId::OPEN_PALM, 5},
        {s7_5::GestureState::ACTIVE, s7_5::GestureId::POINT,
         .95f, s7_5::GestureId::POINT, 5},
        {s7_5::GestureState::ACTIVE, s7_5::GestureId::V_SIGN,
         .95f, s7_5::GestureId::V_SIGN, 5},
        {s7_5::GestureState::ACTIVE, s7_5::GestureId::FIST,
         .95f, s7_5::GestureId::FIST, 5},
    };
    int fixed_gesture = -1;
    if (argc == 5) {
        const std::string name(argv[4]);
        if (name == "neutral") fixed_gesture = 1;
        else if (name == "point") fixed_gesture = 2;
        else if (name == "vsign") fixed_gesture = 3;
        else if (name == "release") fixed_gesture = 4;
        else if (name != "cycle") throw std::runtime_error("unknown UART test gesture");
    }
    for (int index = 0; index < frame_count; ++index) {
        const uint64_t timestamp = now_us(true);
        const int gesture_index = index == 0 ? 0
                                : (fixed_gesture >= 0 ? fixed_gesture : index % 5);
        const auto frame = link.send(gestures[gesture_index],
                                     static_cast<uint32_t>(index),
                                     timestamp, timestamp);
        const auto bytes = s7_6::encode(frame);
        std::cout << "UART_TX frame=" << index + 1 << " bytes=" << bytes.size()
                  << " heartbeat=" << frame.heartbeat << " valid=" << (frame.valid ? 1 : 0)
                  << " hex=" << std::hex << std::setfill('0');
        for (uint8_t byte : bytes) std::cout << std::setw(2) << static_cast<unsigned>(byte);
        std::cout << std::dec << '\n' << std::flush;
        usleep(static_cast<useconds_t>(s7_6::kFramePeriodUs));
    }
    return 0;
}
#endif

static int run_camera(int argc, char** argv) {
    if (argc < 8 || argc > 14) {
#ifdef S7_6_VIRTUAL_RC
        std::cerr << "usage: S7_6_RC_OUTPUT=DEVICE s7_6_live --camera DETECTOR.rknn LANDMARK.rknn DEVICE WIDTH HEIGHT FPS [SECONDS] [MAX_LATENCY_MS] [VBLANK] [ANNOTATE_DIR] [EVERY_N] [DISPLAY_DEVICE]\n";
#elif defined(S7_5_GESTURE)
        std::cerr << "usage: s7_5_live --camera DETECTOR.rknn LANDMARK.rknn DEVICE WIDTH HEIGHT FPS [SECONDS] [MAX_LATENCY_MS] [VBLANK] [ANNOTATE_DIR] [EVERY_N] [DISPLAY_DEVICE]\n";
#else
        std::cerr << "usage: s7_4_live --camera DETECTOR.rknn LANDMARK.rknn DEVICE WIDTH HEIGHT FPS [SECONDS] [MAX_LATENCY_MS] [VBLANK] [ANNOTATE_DIR] [EVERY_N] [DISPLAY_DEVICE]\n";
#endif
        return 2;
    }
    const int width = parse_int(argv[5]);
    const int height = parse_int(argv[6]);
    const int fps = parse_int(argv[7]);
    const int seconds = argc >= 9 ? parse_int(argv[8]) : 0;
    const double max_latency_ms = argc >= 10 ? std::stod(argv[9]) : 200.0;
    const int vblank = argc >= 11 ? parse_int(argv[10]) : -1;
    const char* annotate_dir = argc >= 12 ? argv[11] : nullptr;
    const int annotate_every = argc >= 13 ? parse_int(argv[12]) : 5;
    const char* display_device = argc >= 14 ? argv[13] : nullptr;
    if (width <= 0 || height <= 0 || fps <= 0 || seconds < 0 || max_latency_ms <= 0
        || annotate_every <= 0)
        throw std::runtime_error("camera arguments must be positive");

    Display* display = nullptr;
    if (display_device) {
        display = display_open(display_device);
        if (!display)
            std::cerr << "display disabled: " << display_device << "\n";
        else
            std::cerr << "display: live view on " << display_device << " "
                      << display_width(display) << "x" << display_height(display) << "\n";
    }

    Model detector(argv[2]), landmark(argv[3]);
    Camera camera(argv[4], width, height, fps, vblank);
#ifdef S7_5_GESTURE
    s7_5::GestureStateMachine gesture_state;
#endif
#ifdef S7_6_VIRTUAL_RC
    const char* rc_device = std::getenv("S7_6_RC_OUTPUT");
    if (!rc_device || !*rc_device)
        throw std::runtime_error("set S7_6_RC_OUTPUT to the verified RK3568 serial device or /dev/null");
    auto rc_link = std::make_unique<s7_6::SerialLink>(rc_device);
    const auto startup_timestamp = now_us(true);
    rc_link->send({}, 0, startup_timestamp, startup_timestamp);
#endif
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    const auto deadline = seconds ? Clock::now() + std::chrono::seconds(seconds) : Clock::time_point::max();
    while (running && Clock::now() < deadline) {
        LiveFrame frame;
        bool captured;
        try {
            captured = camera.latest(frame, 1000);
        } catch (...) {
            print_invalid(argv[4], "capture_failure", 0, 0
#ifdef S7_5_GESTURE
                          , &gesture_state
#endif
#ifdef S7_6_VIRTUAL_RC
                          , rc_link.get()
#endif
                          );
            throw;
        }
        if (!captured) {
            print_invalid(argv[4], "capture_timeout", 0, 0
#ifdef S7_5_GESTURE
                          , &gesture_state
#endif
#ifdef S7_6_VIRTUAL_RC
                          , rc_link.get()
#endif
                          );
            continue;
        }
        if (frame.bytes < static_cast<size_t>(camera.width() * camera.height() * 3 / 2)) {
            camera.release(frame);
            print_invalid(argv[4], "bad_frame", frame.sequence, frame.dropped_frames
#ifdef S7_5_GESTURE
                          , &gesture_state
#endif
#ifdef S7_6_VIRTUAL_RC
                          , rc_link.get()
#endif
                          );
            continue;
        }
        cv::Mat nv12(camera.height() * 3 / 2, camera.width(), CV_8UC1,
                     const_cast<unsigned char*>(frame.data));
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        camera.release(frame);
        const LiveMeta meta{frame.sequence, frame.capture_timestamp_us, frame.monotonic_timestamp,
                            frame.dropped_frames, max_latency_ms};
        try {
            print_record(argv[4], static_cast<int>(frame.sequence), bgr, detector, landmark,
                         &meta, annotate_dir, annotate_every, display
#ifdef S7_5_GESTURE
                         , &gesture_state
#endif
#ifdef S7_6_VIRTUAL_RC
                         , rc_link.get()
#endif
                         );
        } catch (const std::exception& error) {
            std::cerr << "frame " << frame.sequence << ": " << error.what() << '\n';
            print_invalid(argv[4], "inference_failure", frame.sequence, frame.dropped_frames
#ifdef S7_5_GESTURE
                          , &gesture_state
#endif
#ifdef S7_6_VIRTUAL_RC
                          , rc_link.get()
#endif
                          );
        }
    }
    display_close(display);
    return 0;
}

static std::vector<fs::path> media(const fs::path& root) {
    std::vector<fs::path> result;
    for (const auto& entry : fs::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".jpg") result.push_back(entry.path());
    }
    std::sort(result.begin(), result.end());
    return result;
}

int main(int argc, char** argv) try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
        if (make_anchors().size() != 2016) return 1;
        Detection a, b;
        a.box = {0, 0, 1, 1};
        b.box = {.5f, .5f, 1.5f, 1.5f};
        if (std::abs(iou(a, b) - 1.f / 7.f) > 1e-6f) return 1;
        if (sequence_gap(10, 13) != 2 || sequence_gap(13, 13) != 0) return 1;
        if (parse_int("20") != 20 || parse_int("-1") != -1) return 1;
        try {
            parse_int("20x");
            return 1;
        } catch (const std::runtime_error&) {
        }
#ifdef S7_5_GESTURE
        if (!s7_5::self_test()) return 1;
#endif
#ifdef S7_6_VIRTUAL_RC
        if (!s7_6::self_test()) return 1;
#endif
        std::cout << "self-test ok\n";
        return 0;
    }
#ifdef S7_6_VIRTUAL_RC
    if (argc >= 2 && std::string(argv[1]) == "--uart-test") return run_uart_test(argc, argv);
#endif
    if (argc >= 2 && std::string(argv[1]) == "--camera") return run_camera(argc, argv);
    if (argc != 4 && argc != 5) {
        std::cerr << "usage: s7_3_board_infer DETECTOR.rknn LANDMARK.rknn TESTSET [SECONDS]\n";
        return 2;
    }
    Model detector(argv[1]), landmark(argv[2]);
    const fs::path root = fs::canonical(argv[3]);
    const auto paths = media(root);
    const int seconds = argc == 5 ? parse_int(argv[4]) : 0;
    const auto deadline = Clock::now() + std::chrono::seconds(seconds);
    do {
        for (const auto& path : paths) {
            const std::string source = fs::relative(path, root).generic_string();
            cv::Mat image = cv::imread(path.string());
            if (image.empty()) std::cout << "{\"source\":" << json_string(source) << ",\"frame_index\":0,\"valid\":false,\"reject_reason\":\"invalid_image\"}\n";
            else print_record(source, 0, image, detector, landmark);
            if (seconds && Clock::now() >= deadline) break;
        }
    } while (seconds && Clock::now() < deadline);
    return 0;
} catch (const std::exception& error) {
    std::cerr << "fatal: " << error.what() << '\n';
    return 1;
}
