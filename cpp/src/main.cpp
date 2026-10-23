// slam_runner — standalone console demo of FastSLAM with occupancy grid.
//
// Optional: build with -DWITH_OPENCV=ON to enable a live 500×500 canvas
// that mirrors the original Windows Forms visualizer.

// M_PI and friends are POSIX extensions; MSVC requires this before <cmath>.
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <random>
#include <sstream>

#include "slam/fast_slam.hpp"
#include "slam/mc_localization.hpp"
#include "slam/map_io.hpp"
#include "slam/distributions.hpp"

#ifdef WITH_OPENCV
#  include <opencv2/opencv.hpp>
#endif

using namespace slam;

// ─────────────────────────────────────────────────────────────────────────────
// Configuration (override via command-line flags)
// ─────────────────────────────────────────────────────────────────────────────
struct Config {
    int    n_particles    = 50;
    int    x_cells        = 200;
    int    y_cells        = 200;
    double cell_size      = 6.0;   // cm
    double v              = 3.0;   // cm/s
    double w              = 0.0;   // rad/s
    double z1             = 80.0;  // cm  (left beam at pi/2)
    double z2             = 80.0;  // cm  (right beam at -pi/2)
    int    iterations     = 0;     // 0 = interactive (press Enter)
    std::string save_map_path;     // if non-empty, save best map after SLAM
    std::string load_map_path;     // if non-empty, load map and run localization
};

static void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n"
        << "  --particles N   Number of particles         (default 50)\n"
        << "  --cells-x N     Grid columns                (default 200)\n"
        << "  --cells-y N     Grid rows                   (default 200)\n"
        << "  --cell-size F   Cell size in cm             (default 6)\n"
        << "  --v F           Linear velocity cm/s        (default 3)\n"
        << "  --w F           Angular velocity rad/s      (default 0)\n"
        << "  --z1 F          Left beam reading cm        (default 80)\n"
        << "  --z2 F          Right beam reading cm       (default 80)\n"
        << "  --iterations N  Run N steps then exit       (0 = interactive)\n"
        << "  --save-map FILE Save best-particle map to FILE after SLAM\n"
        << "  --load-map FILE Load map from FILE; run localization instead of SLAM\n";
}

static Config parse_args(int argc, char** argv) {
    Config cfg;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next  = [&]() -> double { return std::stod(argv[++i]); };
        auto nexti = [&]() -> int   { return std::stoi(argv[++i]); };
        auto nexts = [&]() -> std::string { return std::string(argv[++i]); };
        if      (a == "--particles")  cfg.n_particles    = nexti();
        else if (a == "--cells-x")    cfg.x_cells        = nexti();
        else if (a == "--cells-y")    cfg.y_cells        = nexti();
        else if (a == "--cell-size")  cfg.cell_size      = next();
        else if (a == "--v")          cfg.v              = next();
        else if (a == "--w")          cfg.w              = next();
        else if (a == "--z1")         cfg.z1             = next();
        else if (a == "--z2")         cfg.z2             = next();
        else if (a == "--iterations") cfg.iterations     = nexti();
        else if (a == "--save-map")   cfg.save_map_path  = nexts();
        else if (a == "--load-map")   cfg.load_map_path  = nexts();
        else { print_usage(argv[0]); std::exit(1); }
    }
    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Shared algorithm setup
// ─────────────────────────────────────────────────────────────────────────────
static void configure_motion(MotionVelocity& m) {
    m.alpha1 = 0.005;
    m.alpha2 = 0.005;
    m.alpha3 = 0.08 * M_PI / 180.0;
    m.alpha4 = 0.08 * M_PI / 180.0;
    m.alpha5 = 0.0;
    m.alpha6 = 0.0;
    m.sampler = [](double v) { return Distributions::sample_normal(v); };
    m.dt      = 0.5;
}

static BeamRangeFinderModel make_sensor() {
    // sigmaHit=5 cm is appropriate for HC-SR04 ultrasonic noise (~1-3 cm hardware + grid error).
    return BeamRangeFinderModel(
        /*beta_deg=*/40, /*alpha=*/10,
        /*z_hit=*/0.71, /*z_short=*/0.08, /*z_max=*/0.09, /*z_rand=*/0.12,
        /*sigma_hit=*/5.0, /*lambda_short=*/0.5);
}

static FastSLAM make_slam() {
    FastSLAM slam;
    configure_motion(slam.motion_model);
    slam.measurement_model = make_sensor();
    return slam;
}

static MCLocalization make_localizer(std::shared_ptr<OccupancyGridMap> map) {
    MCLocalization loc(std::move(map));
    configure_motion(loc.motion_model);
    loc.measurement_model = make_sensor();
    return loc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Particle initialisation
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<BeliefeOccupancyGrid> make_slam_particles(const Config& cfg) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> rx(0, cfg.x_cells * cfg.cell_size);
    std::uniform_real_distribution<double> ry(0, cfg.y_cells * cfg.cell_size);
    std::uniform_real_distribution<double> rth(0, 2.0 * M_PI);

    std::vector<BeliefeOccupancyGrid> particles;
    particles.reserve(cfg.n_particles);
    for (int i = 0; i < cfg.n_particles; i++) {
        auto map = std::make_shared<OccupancyGridMap>(cfg.cell_size, cfg.x_cells, cfg.y_cells);
        Pose pose(rx(rng), ry(rng), rth(rng));
        particles.emplace_back(pose, map, nullptr);
    }
    return particles;
}

static std::vector<Pose> make_loc_particles(int n, const OccupancyGridMap& map) {
    std::mt19937 rng(std::random_device{}());
    double xmax = map.x_cells * map.cell_size();
    double ymax = map.y_cells * map.cell_size();
    std::uniform_real_distribution<double> rx(0, xmax);
    std::uniform_real_distribution<double> ry(0, ymax);
    std::uniform_real_distribution<double> rth(0, 2.0 * M_PI);

    std::vector<Pose> poses;
    poses.reserve(n);
    for (int i = 0; i < n; i++)
        poses.emplace_back(rx(rng), ry(rng), rth(rng));
    return poses;
}

// ─────────────────────────────────────────────────────────────────────────────
// Console reporting — SLAM
// ─────────────────────────────────────────────────────────────────────────────

static void print_map(const std::vector<BeliefWeightPair>& result,
                      const std::vector<slam::Pose>& best_history) {
    auto it = std::max_element(result.begin(), result.end(),
        [](const BeliefWeightPair& a, const BeliefWeightPair& b) {
            return a.weight < b.weight;
        });
    if (it == result.end()) return;

    const auto& map  = *it->grid.map;
    const auto& pose = it->grid.pose;
    const int xc = map.x_cells;
    const int yc = map.y_cells;
    const double cs = map.cell_size();

    std::vector<char> overlay(static_cast<size_t>(xc * yc), '\0');
    for (size_t p = 0; p < best_history.size(); ++p) {
        int col = static_cast<int>(std::floor(best_history[p].x / cs));
        int row = static_cast<int>(std::floor(best_history[p].y / cs));
        if (col < 0 || col >= xc || row < 0 || row >= yc) continue;
        char ch = (p == 0)                    ? 'i'
                : (p == best_history.size()-1) ? 'R'
                :                               'o';
        overlay[static_cast<size_t>(col + row * xc)] = ch;
    }

    std::cout << "\nFinal map (best particle: " << pose.to_string() << ")\n";
    std::cout << "Path: i=start  o=waypoint  R=end\n";
    std::cout << std::string(static_cast<size_t>(xc + 2), 'b') << "\n";
    for (int row = yc - 1; row >= 0; --row) {
        std::cout << "b";
        for (int col = 0; col < xc; ++col) {
            char ch = overlay[static_cast<size_t>(col + row * xc)];
            if (ch != '\0') { std::cout << ch; continue; }
            double log_odds = map.m[static_cast<size_t>(col + row * xc)].occupancy_log_odds;
            if      (log_odds > 0.0) std::cout << "x";
            else if (log_odds < 0.0) std::cout << " ";
            else                     std::cout << ".";
        }
        std::cout << "b\n";
    }
    std::cout << std::string(static_cast<size_t>(xc + 2), 'b') << "\n" << std::flush;
}

static const slam::Pose* print_best(const std::vector<BeliefWeightPair>& result, int step) {
    auto it = std::max_element(result.begin(), result.end(),
        [](const BeliefWeightPair& a, const BeliefWeightPair& b) {
            return a.weight < b.weight;
        });
    if (it == result.end()) return nullptr;
    std::cout << "Step " << std::setw(4) << step
              << "  best particle: " << it->grid.pose.to_string()
              << " grid: (" << (int)(it->grid.pose.x / 6) << "," << (int)(it->grid.pose.y / 6) << ")"
              << "  w=" << std::scientific << std::setprecision(3) << it->weight
              << "\n" << std::flush;
    return &(it->grid.pose);
}

// ─────────────────────────────────────────────────────────────────────────────
// Console reporting — Localization
// ─────────────────────────────────────────────────────────────────────────────

static const slam::Pose* print_best_loc(const std::vector<PoseWeightPair>& result, int step) {
    auto it = std::max_element(result.begin(), result.end(),
        [](const PoseWeightPair& a, const PoseWeightPair& b) {
            return a.weight < b.weight;
        });
    if (it == result.end()) return nullptr;
    std::cout << "Step " << std::setw(4) << step
              << "  best pose: " << it->pose.to_string()
              << "  w=" << std::scientific << std::setprecision(3) << it->weight
              << "\n" << std::flush;
    return &(it->pose);
}

static void print_loc_map(const OccupancyGridMap& map,
                          const std::vector<slam::Pose>& best_history) {
    const int xc = map.x_cells;
    const int yc = map.y_cells;
    const double cs = map.cell_size();

    std::vector<char> overlay(static_cast<size_t>(xc * yc), '\0');
    for (size_t p = 0; p < best_history.size(); ++p) {
        int col = static_cast<int>(std::floor(best_history[p].x / cs));
        int row = static_cast<int>(std::floor(best_history[p].y / cs));
        if (col < 0 || col >= xc || row < 0 || row >= yc) continue;
        char ch = (p == 0)                    ? 'i'
                : (p == best_history.size()-1) ? 'R'
                :                               'o';
        overlay[static_cast<size_t>(col + row * xc)] = ch;
    }

    std::cout << "\nFixed map with localization path\n";
    std::cout << "Path: i=start  o=waypoint  R=end\n";
    std::cout << std::string(static_cast<size_t>(xc + 2), 'b') << "\n";
    for (int row = yc - 1; row >= 0; --row) {
        std::cout << "b";
        for (int col = 0; col < xc; ++col) {
            char ch = overlay[static_cast<size_t>(col + row * xc)];
            if (ch != '\0') { std::cout << ch; continue; }
            double log_odds = map.m[static_cast<size_t>(col + row * xc)].occupancy_log_odds;
            if      (log_odds > 0.0) std::cout << "x";
            else if (log_odds < 0.0) std::cout << " ";
            else                     std::cout << ".";
        }
        std::cout << "b\n";
    }
    std::cout << std::string(static_cast<size_t>(xc + 2), 'b') << "\n" << std::flush;
}

// ─────────────────────────────────────────────────────────────────────────────
// Optional OpenCV visualisation
// ─────────────────────────────────────────────────────────────────────────────
#ifdef WITH_OPENCV
static void draw_frame(const std::vector<BeliefWeightPair>& result,
                       const Config& cfg, int selected)
{
    const int W = 500, H = 500;
    double fx = W / (cfg.x_cells * cfg.cell_size);
    double fy = H / (cfg.y_cells * cfg.cell_size);

    cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(200, 200, 200));

    if (selected >= 0 && selected < (int)result.size()) {
        auto& map = *result[selected].grid.map;
        for (auto& cell : map.m) {
            if (cell.occupancy_log_odds == 0.0) continue;
            cv::Scalar colour = (cell.occupancy_log_odds < 0.0)
                                ? cv::Scalar(255, 255, 255)
                                : cv::Scalar(30,  30,  30);
            int px = static_cast<int>((cell.xi - cell.cell_size / 2) * fx);
            int py = H - static_cast<int>((cell.yi + cell.cell_size / 2) * fy);
            int pw = static_cast<int>(cell.cell_size * fx);
            int ph = static_cast<int>(cell.cell_size * fy);
            cv::rectangle(canvas, cv::Rect(px, py, pw, ph), colour, cv::FILLED);
        }
        auto& path = result[selected].grid.path;
        for (size_t i = 1; i < path.size(); i++) {
            cv::Point p1(static_cast<int>(path[i-1].x * fx),
                         H - static_cast<int>(path[i-1].y * fy));
            cv::Point p2(static_cast<int>(path[i].x * fx),
                         H - static_cast<int>(path[i].y * fy));
            cv::line(canvas, p1, p2, cv::Scalar(0, 0, 0), 1);
        }
    }

    for (int i = 1; i < cfg.x_cells; i++)
        cv::line(canvas, {static_cast<int>(i * cfg.cell_size * fx), 0},
                         {static_cast<int>(i * cfg.cell_size * fx), H},
                         cv::Scalar(180, 180, 180), 1);
    for (int i = 1; i < cfg.y_cells; i++)
        cv::line(canvas, {0, H - static_cast<int>(i * cfg.cell_size * fy)},
                         {W, H - static_cast<int>(i * cfg.cell_size * fy)},
                         cv::Scalar(180, 180, 180), 1);

    for (int i = 0; i < (int)result.size(); i++) {
        auto& p = result[i].grid.pose;
        int cx = static_cast<int>(p.x * fx);
        int cy = H - static_cast<int>(p.y * fy);
        int ex = cx + static_cast<int>(8 * std::cos(p.theta));
        int ey = cy - static_cast<int>(8 * std::sin(p.theta));
        cv::Scalar colour = (i == selected) ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 200);
        cv::circle(canvas, {cx, cy}, 3, colour, cv::FILLED);
        cv::line(canvas, {cx, cy}, {ex, ey}, colour, 1);
    }

    cv::imshow("FastSLAM", canvas);
}
#endif  // WITH_OPENCV

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Config cfg = parse_args(argc, argv);

    ControlRotation control{cfg.v, cfg.w};
    UltraSonicMeasurement meas;
    meas.theta = {M_PI / 2.0, -M_PI / 2.0};  // left and right beams
    meas.z     = {cfg.z1, cfg.z2};

    // ── Localization mode ────────────────────────────────────────────────────
    if (!cfg.load_map_path.empty()) {
        auto shared_map = std::make_shared<OccupancyGridMap>(load_map(cfg.load_map_path));
        std::cout << "Loaded map (" << shared_map->x_cells << "x" << shared_map->y_cells
                  << ", " << shared_map->cell_size() << " cm cells) from "
                  << cfg.load_map_path << "\n";

        MCLocalization loc = make_localizer(shared_map);
        std::vector<Pose> poses = make_loc_particles(cfg.n_particles, *shared_map);

        std::cout << "MCLocalization — " << cfg.n_particles << " particles\n";

        std::vector<PoseWeightPair> loc_result;
        std::vector<slam::Pose> best_history;
        int step = 0;

        auto do_loc_step = [&]() {
            loc_result = loc.iterate(poses, control, meas);
            poses.clear();
            for (auto& pwp : loc_result) poses.push_back(pwp.pose);
            step++;
            if (const slam::Pose* p = print_best_loc(loc_result, step))
                best_history.push_back(*p);
        };

        if (cfg.iterations > 0) {
            for (int i = 0; i < cfg.iterations; i++) do_loc_step();
            print_loc_map(*shared_map, best_history);
        } else {
            std::cout << "Press Enter to step, 'q' + Enter to quit.\n";
            std::string line;
            while (true) {
                do_loc_step();
                std::getline(std::cin, line);
                if (!line.empty() && line[0] == 'q') break;
            }
            print_loc_map(*shared_map, best_history);
        }
        return 0;
    }

    // ── SLAM mode ────────────────────────────────────────────────────────────
    std::cout << "FastSLAM - " << cfg.n_particles << " particles, "
              << cfg.x_cells << "x" << cfg.y_cells
              << " grid (" << cfg.cell_size << " cm cells)\n"
              << std::flush;

#ifdef WITH_OPENCV
    std::cout << "OpenCV visualisation enabled. Press 'q' to quit, any other key to step.\n";
    cv::namedWindow("FastSLAM", cv::WINDOW_AUTOSIZE);
    int selected = 0;
#endif

    FastSLAM slam_algo = make_slam();
    auto particles = make_slam_particles(cfg);

    std::vector<BeliefWeightPair> result;
    std::vector<slam::Pose> best_history;
    int step = 0;

    auto do_step = [&]() {
        result = slam_algo.iterate(particles, control, meas);
        particles.clear();
        for (auto& bwp : result) particles.push_back(bwp.grid);
        step++;
        if (const slam::Pose* p = print_best(result, step))
            best_history.push_back(*p);
    };

    if (cfg.iterations > 0) {
        for (int i = 0; i < cfg.iterations; i++) do_step();
        print_map(result, best_history);
    } else {
        std::cout << "Press Enter to step, 'q' + Enter to quit.\n";
        std::string line;
        while (true) {
            do_step();
#ifdef WITH_OPENCV
            draw_frame(result, cfg, selected);
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q') break;
#else
            std::getline(std::cin, line);
            if (!line.empty() && line[0] == 'q') break;
#endif
        }
        print_map(result, best_history);
    }

#ifdef WITH_OPENCV
    if (cfg.iterations > 0 && !result.empty()) {
        draw_frame(result, cfg, 0);
        cv::waitKey(0);
    }
#endif

    // Save map if requested.
    if (!cfg.save_map_path.empty() && !result.empty()) {
        auto it = std::max_element(result.begin(), result.end(),
            [](const BeliefWeightPair& a, const BeliefWeightPair& b) {
                return a.weight < b.weight;
            });
        save_map(*it->grid.map, cfg.save_map_path);
        std::cout << "Map saved to " << cfg.save_map_path << "\n";
    }

    return 0;
}
