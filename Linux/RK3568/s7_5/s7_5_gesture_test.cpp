/*
 * 文件作用：S7.5 纯 C++ 最小回归入口，无 RKNN、OpenCV 或板端依赖。
 * 核心功能：运行四类几何分类和 ACTIVE 进入/低置信度立即释放的确定性自检。
 * 关键约束：返回码非零表示分类或状态机契约已破坏。
 */
#include "s7_5_gesture.hpp"

#include <iostream>

int main() {
    if (!s7_5::self_test()) return 1;
    std::cout << "s7.5 gesture self-test ok\n";
    return 0;
}
