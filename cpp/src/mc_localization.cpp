#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include "slam/mc_localization.hpp"
#include <random>

namespace slam {

std::vector<PoseWeightPair> MCLocalization::iterate(
    const std::vector<Pose>& poses,
    const ControlRotation& u_t,
    const UltraSonicMeasurement& z_t)
{
    // 1. Propagate each pose through the motion model and score against the fixed map.
    std::vector<PoseWeightPair> proposal;
    proposal.reserve(poses.size());
    for (const auto& pose : poses) {
        PoseWeightPair pwp;
        pwp.pose   = motion_model.sample_model(u_t, pose);
        pwp.weight = measurement_model.beam_range_finder(z_t, pwp.pose, *map_);
        proposal.push_back(std::move(pwp));
    }

    // 2. Low-variance (systematic) resampling — Probabilistic Robotics Table 4.4.
    const int N = static_cast<int>(proposal.size());
    std::vector<PoseWeightPair> result;
    if (N == 0) return result;
    result.reserve(N);

    double total = 0.0;
    for (const auto& p : proposal) total += p.weight;

    if (total <= 0.0) {
        // Weights all zero — filter has collapsed; return uniform sample.
        for (int i = 0; i < N; i++)
            result.push_back(proposal[static_cast<size_t>(i) % proposal.size()]);
        return result;
    }

    const double step = total / static_cast<double>(N);
    std::uniform_real_distribution<double> dist(0.0, step);
    double r = dist(rng_);   // single random offset in [0, step)

    double cumulative = proposal[0].weight;
    size_t idx = 0;
    for (int m = 0; m < N; m++) {
        double threshold = r + static_cast<double>(m) * step;
        while (cumulative < threshold && idx + 1 < proposal.size()) {
            cumulative += proposal[++idx].weight;
        }
        result.push_back(proposal[idx]);
    }
    return result;
}

}  // namespace slam
