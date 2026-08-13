#include <rknn_api.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
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

static void print_array(const float* values, size_t count) {
    std::cout << '[';
    for (size_t i = 0; i < count; ++i) {
        if (i) std::cout << ',';
        std::cout << values[i];
    }
    std::cout << ']';
}

static void print_record(const std::string& source, int frame_index, const cv::Mat& bgr,
                         Model& detector, Model& landmark) {
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

    std::cout << std::fixed << std::setprecision(6) << "{\"source\":" << json_string(source)
              << ",\"frame_index\":" << frame_index;
    if (detections.empty()) {
        timing.postprocess_ms = ms(start);
        std::cout << ",\"valid\":false,\"reject_reason\":\"no_hand\"";
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
        std::cout << ",\"detector_score\":" << d.score << ",\"bbox_xyxy_normalized\":";
        print_array(d.box.val, 4);
        std::cout << ",\"detector_keypoints_7x2\":";
        print_array(reinterpret_cast<float*>(d.keypoints), 14);
        std::cout << ",\"roi_center_scale_rotation\":";
        print_array(roi_info.val, 4);
        std::cout << ",\"presence_score\":" << presence << ",\"handedness_score\":" << handedness
                  << ",\"landmarks_21x3_roi\":";
        print_array(landmark_outputs[0].data(), 63);
        std::cout << ",\"landmarks_21x3_image\":";
        print_array(image_landmarks.data(), 63);
        std::cout << ",\"valid\":" << (presence >= .5f ? "true" : "false")
                  << ",\"reject_reason\":" << (presence >= .5f ? "\"\"" : "\"low_presence\"");
    }
    std::cout << ",\"timing_ms\":{\"preprocess\":" << timing.preprocess_ms
              << ",\"detector\":" << timing.detector_ms << ",\"postprocess\":" << timing.postprocess_ms
              << ",\"landmark\":" << timing.landmark_ms << "}}\n";
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
        std::cout << "self-test ok\n";
        return 0;
    }
    if (argc != 4 && argc != 5) {
        std::cerr << "usage: s7_3_board_infer DETECTOR.rknn LANDMARK.rknn TESTSET [SECONDS]\n";
        return 2;
    }
    Model detector(argv[1]), landmark(argv[2]);
    const fs::path root = fs::canonical(argv[3]);
    const auto paths = media(root);
    const int seconds = argc == 5 ? std::stoi(argv[4]) : 0;
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
