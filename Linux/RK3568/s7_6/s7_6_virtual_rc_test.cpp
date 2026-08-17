/* S7.6 无 RKNN/OpenCV/串口依赖的最小回归入口。 */
#include "s7_6_virtual_rc.hpp"

#include <iostream>

int main() {
    if (!s7_6::self_test()) return 1;
    std::cout << "s7.6 virtual RC self-test ok\n";
    return 0;
}
