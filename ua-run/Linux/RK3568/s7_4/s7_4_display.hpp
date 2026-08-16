/*
 * 文件作用：S7.4 屏显观测后端的对外接口。把任意尺寸的 BGR888 帧缩放并显示到
 * DRM/KMS 显示屏上，供真人手部识别结果在板端屏幕上实时观察。
 * 核心函数：display_open（打开 DRM 设备并枚举已连接 connector 的首选模式）、
 * display_width/display_height（屏分辨率查询）、display_show（缩放+双缓冲 page flip）、
 * display_close（释放全部资源）。
 * 主要数据流：BGR888 帧 → cv::resize 到屏分辨率 → XRGB8888 双缓冲交替填充 →
 * DRM_MODE_PAGE_FLIP → 显示屏。
 * 关键约束：显示失败只告警并返回可恢复状态，绝不影响调用方推理与 JSONL 契约；
 * page flip 完成事件必须及时读取，否则内核事件队列满会导致后续 flip 返回 EBUSY。
 */
#ifndef S7_4_DISPLAY_HPP
#define S7_4_DISPLAY_HPP

#include <cstdint>

struct Display;

// 打开 DRM 设备并枚举首个已连接且含可用模式的 connector；失败返回 nullptr 并输出原因到 stderr。
Display* display_open(const char* device);

// 屏的像素尺寸（connector 首选模式）。
int display_width(const Display* display);
int display_height(const Display* display);

// 显示一帧连续 BGR888（width/height 为源帧尺寸）；内部缩放、转 XRGB8888 并 page flip。
// 失败返回 false（仅告警）。flip 未完成或超时返回 false 并丢弃本帧。
bool display_show(Display* display, const uint8_t* bgr, int width, int height);

// 释放 framebuffer、映射和 fd；允许传入 nullptr。
void display_close(Display* display);

#endif
