#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>

namespace slam {

struct GridCell {
    double xi{0.0};
    double yi{0.0};
    double cell_size{0.0};
    double occupancy_log_odds{0.0};
    double prior_occupancy_prob{0.0};

    static double convert_prob_to_log_odds(double prob) {
        if (prob < 0.0 || prob > 1.0)
            throw std::invalid_argument("Probability must be in the range [0..1]");
        return std::log(prob / (1.0 - prob));
    }

    static double convert_log_odds_to_prob(double log_odds) {
        double e = std::exp(log_odds);
        return e / (1.0 + e);
    }

    std::string to_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "x:" << xi << " cm, y:" << yi
           << " cm, locc:" << occupancy_log_odds
           << ", prioLocc:" << prior_occupancy_prob;
        return ss.str();
    }
};

}  // namespace slam
