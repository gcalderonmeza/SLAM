#include "slam/fast_slam.hpp"
#include <algorithm>

namespace slam {

std::vector<BeliefWeightPair> FastSLAM::iterate(
    const std::vector<BeliefeOccupancyGrid>& chi_t_1,
    const ControlRotation& u_t,
    const UltraSonicMeasurement& z_t)
{
    std::vector<BeliefWeightPair> chi_t_bar;
    chi_t_bar.reserve(chi_t_1.size());

    for (const auto& belief : chi_t_1) {
        // 1. Sample new pose from motion model.
        Pose x_t = motion_model.sample_model(u_t, belief.pose);

        // 2. Weight against the PRE-update map (sensor model on current belief).
        double w_t = measurement_model.beam_range_finder(z_t, x_t, *belief.map);

        // 3. Deep-copy the map, then update the copy.
        //    Without cloning, particles resampled multiple times share one map object:
        //    every subsequent update corrupts all copies simultaneously.
        auto map_copy = std::make_shared<OccupancyGridMap>(*belief.map);
        map_copy->update_map(x_t, z_t, measurement_model);

        BeliefWeightPair bwp;
        bwp.grid   = BeliefeOccupancyGrid(x_t, map_copy, &belief.path);
        bwp.weight = w_t;
        chi_t_bar.push_back(std::move(bwp));
    }

    return sample_prob_distribution(static_cast<int>(chi_t_1.size()), chi_t_bar);
}

std::vector<BeliefWeightPair> FastSLAM::sample_prob_distribution(
    int total_samples,
    const std::vector<BeliefWeightPair>& chi_t_bar)
{
    std::vector<BeliefWeightPair> chi_t;
    if (chi_t_bar.empty()) return chi_t;
    chi_t.reserve(total_samples);

    // Build cumulative-weight ladder.
    std::vector<double> limits;
    limits.reserve(chi_t_bar.size());
    double cum = 0.0;
    for (const auto& p : chi_t_bar) {
        cum += p.weight;
        limits.push_back(cum);
    }
    double total = limits.back();

    std::uniform_real_distribution<double> dist(0.0, total);
    while (static_cast<int>(chi_t.size()) < total_samples) {
        double r = dist(rng_);
        if (r == 0.0) continue;  // guard against degenerate edge case

        // First limit >= r → the particle whose weight interval contains r.
        auto it  = std::lower_bound(limits.begin(), limits.end(), r);
        size_t idx = static_cast<size_t>(std::distance(limits.begin(), it));
        if (idx >= chi_t_bar.size()) idx = chi_t_bar.size() - 1;

        chi_t.push_back(chi_t_bar[idx]);
    }
    return chi_t;
}

}  // namespace slam
