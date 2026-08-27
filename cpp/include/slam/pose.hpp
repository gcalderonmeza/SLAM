#pragma once
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

namespace slam {

struct Pose {
    double x;
    double y;
    double theta;

    Pose() : x(0.0), y(0.0), theta(0.0) {}
    Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    std::string to_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "x:" << x << " cm, y:" << y << " cm, theta:"
           << (theta * 180.0 / M_PI) << " deg";
        return ss.str();
    }
};

}  // namespace slam
