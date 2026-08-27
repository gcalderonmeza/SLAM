// slam_runner — standalone console demo of FastSLAM with occupancy grid.
//
// Optional: build with -DWITH_OPENCV=ON to enable a live 500×500 canvas
// that mirrors the original Windows Forms visualizer.

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <random>
#include <sstream>

#include "slam/fast_slam.hpp"
#include "slam/distributions.hpp"

#ifdef WITH_OPENCV
#  include <opencv2/opencv.hpp>
#endif

using namespace slam;

// ─────────────────────────────────────────────────────────────────────────────
// Configuration (override via command-line flags)
// ─────────────────────────────────────────────────────────────────────────────
struct Config {
    int    n_particles = 50;
    int    x_cells     = 20;
    int    y_cells     = 20;
    double cell_size   = 10.0;   // cm
    double v           = 5.0;    // cm/s
    double w           = 0.1;    // rad/s
    double z1          = 80.0;   // cm  (left beam at pi/2)
    double z2          = 80.0;   // cm  (right beam at -pi/2)
    int    iterations  = 0;      // 0 = interactive (press Enter)
};

static void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n"
        << "  --particles N   Number of particles   (default 50)\n"
        << "  --cells-x N     Grid columns           (default 20)\n"
        << "  --cells-y N     Grid rows              (default 20)\n"
        << "  --cell-size F   Cell size in cm        (default 10)\n"
        << "  --v F           Linear velocity cm/s   (default 5)\n"
        << "  --w F           Angular velocity rad/s (default 0.1)\n"
        << "  --z1 F          Left beam reading cm   (default 80)\n"
        << "  --z2 F          Right beam reading cm  (default 80)\n"
        << "  --iterations N  Run N steps then exit  (0 = interactive)\n";
}

static Config parse_args(int argc, char** argv) {
    Config cfg;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> double { return std::stod(argv[++i]); };
        auto nexti = [&]() -> int   { return std::stoi(argv[++i]); };
        if      (a == "--particles")  cfg.n_particles = nexti();
        else if (a == "--cells-x")    cfg.x_cells     = nexti();
        else if (a == "--cells-y")    cfg.y_cells     = nexti();
        else if (a == "--cell-size")  cfg.cell_size   = next();
        else if (a == "--v")          cfg.v           = next();
        else if (a == "--w")          cfg.w           = next();
        else if (a == "--z1")         cfg.z1          = next();
        else if (a == "--z2")         cfg.z2          = next();
        else if (a == "--iterations") cfg.iterations  = nexti();
        else { print_usage(argv[0]); std::exit(1); }
    }
    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Particle initialisation
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<BeliefeOccupancyGrid> make_particles(const Config& cfg) {
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

// ─────────────────────────────────────────────────────────────────────────────
// FastSLAM algorithm setup
// ─────────────────────────────────────────────────────────────────────────────
static FastSLAM make_slam() {
    FastSLAM slam;
    slam.motion_model.alpha1 = 0.005;
    slam.motion_model.alpha2 = 0.005;
    slam.motion_model.alpha3 = 0.08 * M_PI / 180.0;
    slam.motion_model.alpha4 = 0.08 * M_PI / 180.0;
    slam.motion_model.alpha5 = 0.0;
    slam.motion_model.alpha6 = 0.0;
    slam.motion_model.sampler = [](double v) { return Distributions::sample_normal(v); };
    slam.motion_model.dt      = 0.5;

    // sigmaHit=5 cm is appropriate for HC-SR04 ultrasonic noise (~1-3 cm hardware + grid error).
    slam.measurement_model = BeamRangeFinderModel(
        /*beta_deg=*/40, /*alpha=*/10,
        /*z_hit=*/0.71, /*z_short=*/0.08, /*z_max=*/0.09, /*z_rand=*/0.12,
        /*sigma_hit=*/5.0, /*lambda_short=*/0.5);
    return slam;
}

// ─────────────────────────────────────────────────────────────────────────────
// Console reporting
// ─────────────────────────────────────────────────────────────────────────────
static void print_best(const std::vector<BeliefWeightPair>& result, int step) {
    auto it = std::max_element(result.begin(), result.end(),
        [](const BeliefWeightPair& a, const BeliefWeightPair& b) {
            return a.weight < b.weight;
        });
    if (it == result.end()) return;
    std::cout << "Step " << std::setw(4) << step
              << "  best particle: " << it->grid.pose.to_string()
              << "  w=" << std::scientific << std::setprecision(3) << it->weight
              << "\n";
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

    // Draw selected particle's map.
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
        // Draw path.
        auto& path = result[selected].grid.path;
        for (size_t i = 1; i < path.size(); i++) {
            cv::Point p1(static_cast<int>(path[i-1].x * fx),
                         H - static_cast<int>(path[i-1].y * fy));
            cv::Point p2(static_cast<int>(path[i].x * fx),
                         H - static_cast<int>(path[i].y * fy));
            cv::line(canvas, p1, p2, cv::Scalar(0, 0, 0), 1);
        }
    }

    // Grid lines.
    for (int i = 1; i < cfg.x_cells; i++)
        cv::line(canvas, {static_cast<int>(i * cfg.cell_size * fx), 0},
                         {static_cast<int>(i * cfg.cell_size * fx), H},
                         cv::Scalar(180, 180, 180), 1);
    for (int i = 1; i < cfg.y_cells; i++)
        cv::line(canvas, {0, H - static_cast<int>(i * cfg.cell_size * fy)},
                         {W, H - static_cast<int>(i * cfg.cell_size * fy)},
                         cv::Scalar(180, 180, 180), 1);

    // Particles.
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

    std::cout << "FastSLAM — " << cfg.n_particles << " particles, "
              << cfg.x_cells << "x" << cfg.y_cells
              << " grid (" << cfg.cell_size << " cm cells)\n";

#ifdef WITH_OPENCV
    std::cout << "OpenCV visualisation enabled. Press 'q' to quit, any other key to step.\n";
    cv::namedWindow("FastSLAM", cv::WINDOW_AUTOSIZE);
    int selected = 0;
#endif

    FastSLAM slam = make_slam();

    auto particles = make_particles(cfg);

    ControlRotation control{cfg.v, cfg.w};
    UltraSonicMeasurement meas;
    meas.theta = {M_PI / 2.0, -M_PI / 2.0};  // left and right beams
    meas.z     = {cfg.z1, cfg.z2};

    std::vector<BeliefWeightPair> result;
    int step = 0;

    auto do_step = [&]() {
        result = slam.iterate(particles, control, meas);
        particles.clear();
        for (auto& bwp : result) particles.push_back(bwp.grid);
        step++;
        print_best(result, step);
    };

    if (cfg.iterations > 0) {
        for (int i = 0; i < cfg.iterations; i++) do_step();
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
    }

#ifdef WITH_OPENCV
    if (cfg.iterations > 0 && !result.empty()) {
        draw_frame(result, cfg, 0);
        cv::waitKey(0);
    }
#endif

    return 0;
}
