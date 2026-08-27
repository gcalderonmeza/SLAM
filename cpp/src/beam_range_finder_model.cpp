#include "slam/beam_range_finder_model.hpp"
#include <cmath>
#include <stdexcept>

namespace slam {

BeamRangeFinderModel::BeamRangeFinderModel(
    double beta_deg, double alpha,
    double z_hit, double z_short, double z_max, double z_rand,
    double sigma_hit, double lambda_short)
    : z_max_prob(z_max),
      beta(beta_deg * M_PI / 180.0),
      z_hit_(z_hit),
      z_short_(z_short),
      z_rand_(z_rand),
      lambda_short_(lambda_short),
      alpha_(alpha),
      sigma_hit_(sigma_hit)
{
    l_free_ = std::log(0.2 / (1.0 - 0.2));  // log-odds for p=0.2 (free)
    l_occ_  = std::log(0.8 / (1.0 - 0.8));  // log-odds for p=0.8 (occupied)
}

// ─────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────

double BeamRangeFinderModel::beam_range_finder(
    const UltraSonicMeasurement& z_t,
    const Pose& x_t,
    const OccupancyGridMap& m) const
{
    double q = 1.0;
    for (int k = 0; k < z_t.beams(); k++) {
        double k_t_star = compute_z_star(x_t, m, z_t.theta[k]);
        double p = z_hit_   * p_hit(z_t.z[k], k_t_star)
                 + z_short_ * p_short(z_t.z[k], k_t_star)
                 + z_max_prob * p_max(z_t.z[k])
                 + z_rand_  * p_rand(z_t.z[k]);
        q *= p;
    }
    return q;
}

double BeamRangeFinderModel::inverse_range_sensor_model(
    const GridCell& mi, const Pose& x_t,
    const UltraSonicMeasurement& z_t,
    int index_closest,
    double angle_closest,
    double dist_cm) const
{
    if (index_closest == -1) {
        dist_cm = z_t.find_range_and_angle_to_closest_beam(
            mi, x_t, angle_closest, index_closest);
    }

    double z_k = z_t.z[index_closest];

    // Outside range or field of view → no information.
    if (dist_cm > std::min(MAX_RANGE, z_k + alpha_ / 2.0) || angle_closest > beta / 2.0)
        return mi.prior_occupancy_prob;

    // Within alpha/2 of measured range → likely occupied.
    if (z_k < MAX_RANGE && std::abs(dist_cm - z_k) < alpha_ / 2.0)
        return l_occ_;

    // Between robot and measured range → likely free.
    if (dist_cm <= z_k)
        return l_free_;

    throw std::runtime_error("inverse_range_sensor_model: no case matched");
}

bool BeamRangeFinderModel::in_perceptual_field(
    const GridCell& mi, const Pose& x_t,
    const UltraSonicMeasurement& z_t,
    int& index_closest,
    double& angle_closest,
    double& dist_cm) const
{
    dist_cm = z_t.find_range_and_angle_to_closest_beam(
        mi, x_t, angle_closest, index_closest);
    return dist_cm < MAX_RANGE && angle_closest < beta / 2.0;
}

void BeamRangeFinderModel::learn_intrinsic_parameters(
    const std::vector<UltraSonicMeasurement>& z,
    const std::vector<Pose>& x,
    const OccupancyGridMap& m)
{
    const double threshold = 0.05;
    const int beam_idx = 1;  // uses second beam, matching the C# original

    std::vector<double> z_star(z.size());
    std::vector<double> e_hit(z.size()), e_short(z.size());
    std::vector<double> e_max(z.size()), e_rand(z.size());

    double prev_z_hit, prev_z_short, prev_z_max, prev_z_rand;
    double prev_sigma, prev_lambda;

    do {
        prev_z_hit   = z_hit_;
        prev_z_short = z_short_;
        prev_z_max   = z_max_prob;
        prev_z_rand  = z_rand_;
        prev_sigma   = sigma_hit_;
        prev_lambda  = lambda_short_;

        for (size_t i = 0; i < z.size(); i++) {
            z_star[i] = compute_z_star(x[i], m, z[i].theta[beam_idx]);
            double zi  = z[i].z[beam_idx];

            double hit   = p_hit(zi, z_star[i]);
            double shrt  = p_short(zi, z_star[i]);
            double mx    = p_max(zi);
            double rnd   = p_rand(zi);
            double eta_1 = hit + shrt + mx + rnd;

            e_hit[i]   = hit   / eta_1;
            e_short[i] = shrt  / eta_1;
            e_max[i]   = mx    / eta_1;
            e_rand[i]  = rnd   / eta_1;
        }

        double sum_hit = 0.0, sum_short = 0.0, sum_max = 0.0, sum_rand = 0.0;
        for (size_t i = 0; i < z.size(); i++) {
            sum_hit   += e_hit[i];
            sum_short += e_short[i];
            sum_max   += e_max[i];
            sum_rand  += e_rand[i];
        }
        z_hit_    = sum_hit   / static_cast<double>(z.size());
        z_short_  = sum_short / static_cast<double>(z.size());
        z_max_prob = sum_max  / static_cast<double>(z.size());
        z_rand_   = sum_rand  / static_cast<double>(z.size());

        // sigma_hit = sqrt( sum(e_hit * (z - z*)^2) / sum(e_hit) )
        double num_sigma = 0.0, den_sigma = 0.0;
        double num_lambda = 0.0, den_lambda = 0.0;
        for (size_t i = 0; i < z.size(); i++) {
            double diff = z[i].z[beam_idx] - z_star[i];
            num_sigma  += e_hit[i] * diff * diff;
            den_sigma  += e_hit[i];
            num_lambda += e_short[i];
            den_lambda += e_short[i] * z[i].z[beam_idx];
        }
        sigma_hit_    = std::sqrt(num_sigma / den_sigma);
        lambda_short_ = num_lambda / den_lambda;

    } while (
        std::abs((prev_z_hit   - z_hit_)   / z_hit_)   > threshold ||
        std::abs((prev_z_short - z_short_) / z_short_) > threshold ||
        std::abs((prev_z_max   - z_max_prob) / z_max_prob) > threshold ||
        std::abs((prev_z_rand  - z_rand_)  / z_rand_)  > threshold ||
        std::abs((prev_sigma   - sigma_hit_) / sigma_hit_) > threshold ||
        std::abs((prev_lambda  - lambda_short_) / lambda_short_) > threshold
    );
}

// ─────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────

double BeamRangeFinderModel::p_hit(double z_t, double k_t_star) const {
    if (z_t >= 0.0 && z_t <= MAX_RANGE) {
        double var = sigma_hit_ * sigma_hit_;
        double eta = compute_eta(k_t_star, var);
        double norm = std::exp(-0.5 * (z_t - k_t_star) * (z_t - k_t_star) / var)
                      / std::sqrt(2.0 * M_PI * var);
        return eta * norm;
    }
    return 0.0;
}

double BeamRangeFinderModel::p_short(double z_t, double k_t_star) const {
    if (z_t >= 0.0 && z_t < k_t_star) {
        double eta = 1.0 - std::exp(-lambda_short_ * k_t_star);
        return lambda_short_ * std::exp(-lambda_short_ * z_t) / eta;
    }
    return 0.0;
}

double BeamRangeFinderModel::p_max(double z_t) const {
    return (z_t == MAX_RANGE) ? 1.0 : 0.0;
}

double BeamRangeFinderModel::p_rand(double z_t) const {
    return (z_t >= 0.0 && z_t < MAX_RANGE) ? 1.0 / MAX_RANGE : 0.0;
}

double BeamRangeFinderModel::compute_eta_iterative(double mean, double variance) const {
    double cumulative = 0.0;
    double low = 0.0;
    double delta = MAX_RANGE / 500.0;
    auto gauss = [&](double z) {
        return std::exp(-0.5 * (z - mean) * (z - mean) / variance)
               / std::sqrt(2.0 * M_PI * variance);
    };
    double prev = gauss(low);
    while (low + delta <= MAX_RANGE) {
        double next = gauss(low + delta);
        cumulative += delta * (prev + next);
        low += delta;
        prev = next;
    }
    if (low + delta > MAX_RANGE) {
        cumulative += delta * (prev + gauss(MAX_RANGE));
    }
    return (cumulative > 0.0) ? 2.0 / cumulative : 0.0;
}

double BeamRangeFinderModel::compute_eta(double mean, double variance) const {
    return compute_eta_iterative(mean, variance);
}

// Ray-cast along beam direction; returns first occupied-cell distance (cm),
// or MAX_RANGE if no obstacle found.
double BeamRangeFinderModel::compute_z_star(
    const Pose& x_t, const OccupancyGridMap& m,
    double angle_robot_coords) const
{
    double angle_rad = x_t.theta + angle_robot_coords;
    double delta_x = std::cos(angle_rad);
    double delta_y = std::sin(angle_rad);
    double cs = m.cell_size();

    double rx2, ry2;
    Pose x_temp = x_t;
    move_to_next_cell(x_temp, cs, delta_x, delta_y, rx2, ry2);
    x_temp.x += rx2;
    x_temp.y += ry2;
    x_temp.theta = angle_rad;

    // log-odds of 50% = 0; cells with log-odds > 0 are occupied.
    const double threshold = GridCell::convert_prob_to_log_odds(0.5);  // = 0

    auto cell = m.get_cell_from_pose(x_temp);
    while (cell.first >= 0 && cell.first < m.x_cells &&
           cell.second >= 0 && cell.second < m.y_cells)
    {
        // Fixed C# bug: stride must be x_cells, not y_cells.
        int idx = cell.first + cell.second * m.x_cells;
        if (m.m[idx].occupancy_log_odds > threshold) {
            double dx = x_temp.x - x_t.x;
            double dy = x_temp.y - x_t.y;
            return std::sqrt(dx * dx + dy * dy);
        }
        move_to_next_cell(x_temp, cs, delta_x, delta_y, rx2, ry2);
        x_temp.x += rx2;
        x_temp.y += ry2;
        cell = m.get_cell_from_pose(x_temp);
    }
    return MAX_RANGE;
}

void BeamRangeFinderModel::move_to_next_cell(
    const Pose& x_temp, double cell_size,
    double delta_x, double delta_y,
    double& rx2, double& ry2)
{
    double dx, dy;
    if (delta_x < 0.0)
        dx = (cell_size * std::floor(x_temp.x / cell_size) - x_temp.x) / delta_x;
    else if (delta_x > 0.0)
        dx = (cell_size * std::floor((x_temp.x + cell_size) / cell_size) - x_temp.x) / delta_x;
    else
        dx = 0.0;

    if (delta_y < 0.0)
        dy = (cell_size * std::floor(x_temp.y / cell_size) - x_temp.y) / delta_y;
    else if (delta_y > 0.0)
        dy = (cell_size * std::floor((x_temp.y + cell_size) / cell_size) - x_temp.y) / delta_y;
    else
        dy = 0.0;

    double factor;
    if (delta_x != 0.0 && delta_y != 0.0)
        factor = std::min(dx, dy);
    else if (delta_x == 0.0)
        factor = dy;
    else
        factor = dx;

    // Slight overshoot in the negative direction to reliably cross cell boundaries.
    rx2 = delta_x * factor * (delta_x < 0.0 ? 1.00001 : 1.0);
    ry2 = delta_y * factor * (delta_y < 0.0 ? 1.00001 : 1.0);
}

}  // namespace slam
