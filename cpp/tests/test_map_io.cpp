// Tests for map_io — save_map / load_map round-trip.
#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include "slam/map_io.hpp"
#include "slam/grid_cell.hpp"

using namespace slam;

namespace {

// Temporary file that deletes itself on destruction.
struct TmpFile {
    std::string path;
    explicit TmpFile(const std::string& name) : path(name) {}
    ~TmpFile() { std::remove(path.c_str()); }
};

OccupancyGridMap make_known_map() {
    OccupancyGridMap m(10.0, 5, 4);   // 5 cols × 4 rows, 10 cm cells
    // Stamp a simple pattern so we can tell cells apart after loading.
    for (int i = 0; i < static_cast<int>(m.m.size()); i++)
        m.m[static_cast<size_t>(i)].occupancy_log_odds =
            GridCell::convert_prob_to_log_odds(0.1 + 0.05 * i);
    return m;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────

TEST(MapIO, RoundTrip_Dimensions) {
    TmpFile tmp("test_map_dims.txt");
    OccupancyGridMap orig = make_known_map();
    save_map(orig, tmp.path);

    OccupancyGridMap loaded = load_map(tmp.path);
    EXPECT_EQ(orig.x_cells, loaded.x_cells);
    EXPECT_EQ(orig.y_cells, loaded.y_cells);
    EXPECT_NEAR(orig.cell_size(), loaded.cell_size(), 1e-9);
}

TEST(MapIO, RoundTrip_CellCount) {
    TmpFile tmp("test_map_cells.txt");
    OccupancyGridMap orig = make_known_map();
    save_map(orig, tmp.path);

    OccupancyGridMap loaded = load_map(tmp.path);
    EXPECT_EQ(orig.m.size(), loaded.m.size());
}

TEST(MapIO, RoundTrip_LogOddsValues) {
    TmpFile tmp("test_map_logodds.txt");
    OccupancyGridMap orig = make_known_map();
    save_map(orig, tmp.path);

    OccupancyGridMap loaded = load_map(tmp.path);
    for (size_t i = 0; i < orig.m.size(); i++)
        EXPECT_NEAR(orig.m[i].occupancy_log_odds,
                    loaded.m[i].occupancy_log_odds, 1e-9)
            << "Cell " << i << " log_odds mismatch.";
}

TEST(MapIO, RoundTrip_CellGeometry) {
    TmpFile tmp("test_map_geom.txt");
    OccupancyGridMap orig = make_known_map();
    save_map(orig, tmp.path);

    OccupancyGridMap loaded = load_map(tmp.path);
    // xi, yi, cell_size are recomputed from dimensions — they must match.
    for (size_t i = 0; i < orig.m.size(); i++) {
        EXPECT_NEAR(orig.m[i].xi,        loaded.m[i].xi,        1e-9) << "xi  cell " << i;
        EXPECT_NEAR(orig.m[i].yi,        loaded.m[i].yi,        1e-9) << "yi  cell " << i;
        EXPECT_NEAR(orig.m[i].cell_size, loaded.m[i].cell_size, 1e-9) << "cs  cell " << i;
    }
}

TEST(MapIO, SaveToNonexistentPath_Throws) {
    OccupancyGridMap m = make_known_map();
    EXPECT_THROW(
        save_map(m, "/nonexistent/directory/file.txt"),
        std::runtime_error);
}

TEST(MapIO, LoadFromMissingFile_Throws) {
    EXPECT_THROW(
        load_map("no_such_file_xyz.txt"),
        std::runtime_error);
}
