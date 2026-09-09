#pragma once
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

namespace slam {

struct ControlRotation {
    double v{0.0};  // linear velocity (cm/s)
    double w{0.0};  // angular velocity (rad/s)

    std::string to_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1)
           << "v:" << v << " cm/s, w:"
           << std::setprecision(4) << (w * 180.0 / M_PI) << " deg";
        return ss.str();
    }
};

}  // namespace slam
