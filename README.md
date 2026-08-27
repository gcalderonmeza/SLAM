# SLAM

A C# implementation of **FastSLAM with Occupancy Grid Maps**, based on the algorithms described in *Probabilistic Robotics* (Thrun, Burgard, Fox). The robot uses an ultrasonic sensor (sonar) to simultaneously build a map of its environment and localize itself within it.

---

## What is SLAM?

**SLAM** (Simultaneous Localization and Mapping) solves a chicken-and-egg problem in robotics: to build a map you need to know where you are, but to know where you are you need a map. SLAM algorithms solve both problems at the same time using probabilistic inference.

This implementation uses the **FastSLAM** variant, which represents the robot's uncertainty about its position as a set of weighted *particles* (hypotheses), each carrying its own version of the map. Over time, particles that fit the sensor data better accumulate more weight and survive resampling, while poor-fitting particles are discarded.

---

## Project Structure

```
SLAM/
├── Models/                    # Core data structures and probabilistic models
│   ├── Pose.cs                # Robot position and orientation (x, y, theta)
│   ├── GridCell.cs            # Single cell in the occupancy grid
│   ├── OccupancyGridMap.cs    # 2D grid map with log-odds occupancy values
│   ├── BeliefeOccupancyGrid.cs# One particle: a pose hypothesis + its map + path history
│   ├── BeamRangeFinder.cs     # Sensor model for an ultrasonic range finder
│   ├── MotionVelocity.cs      # Motion model using velocity commands (v, w)
│   ├── MotionOdometry.cs      # Motion model using odometry readings
│   ├── ControlRotation.cs     # Velocity control command (v = linear, w = angular)
│   ├── ControlOdometry.cs     # Odometry control command
│   ├── UltraSonicMeasurement.cs # Sensor measurement: distances + beam angles
│   ├── MapBase.cs             # Abstract base class for maps
│   └── Distributions.cs       # Statistical utilities (Normal, Triangular sampling)
│
├── Algorithms/
│   └── FastSLAMOccupancyGrid.cs  # The main FastSLAM algorithm
│
├── UnitTests/
│   ├── UnitTest1.cs           # Tests for motion models, sensor learning, full SLAM loop
│   ├── FastSLAMModel.cs       # Helper that wires together the algorithm components
│   └── RobotWorldForm.cs      # Windows Forms visualization of the robot and map
│
├── FastSLAMOccuppancyGrid/
│   └── Program.cs             # Entry point (currently empty; algorithm runs via tests)
│
└── Robotics.sln               # Visual Studio solution
```

---

## How the Algorithm Works

### 1. Particles (Beliefs)

The robot's uncertainty about its state is represented by **N particles** (`BeliefeOccupancyGrid`). Each particle holds:
- A **pose** hypothesis `(x, y, theta)` — where the robot might be.
- A **map** (`OccupancyGridMap`) — what the world looks like *from that pose hypothesis*.
- A **path** — the history of poses this particle has traveled through.

A typical run uses 1000 particles (`numBeliefs: 1000`), each maintaining a 100×100 cell map (1 cm per cell by default).

### 2. The Main Loop — `FastSLAMOccupancyGrid.Iterate()`

Each time step receives:
- `ChiT_1` — the current set of particles (prior beliefs).
- `uT` — the motion command (velocity or rotation).
- `zT` — the sensor reading (distances from the sonar beams).

For each particle it:

1. **Samples a new pose** from the motion model — adds realistic noise to predict where the robot moved.
2. **Scores the particle** using the measurement model — how well does the sonar reading match the map at this hypothesized pose?
3. **Updates the map** — incorporates the new sensor reading into this particle's occupancy grid.
4. **Resamples** — draws N particles from the scored set, with probability proportional to weight, so well-fitting particles survive and poor ones are discarded.

### 3. Motion Model — `MotionVelocity.SampleModel()`

Takes a velocity command `(v, w)` (linear velocity + angular velocity) and the previous pose, and returns a new sampled pose. Gaussian noise is added to both linear and angular velocity using six `alpha` parameters that encode how much the robot's motion drifts:

- `alpha1`, `alpha2` — translational noise due to translation and rotation respectively.
- `alpha3`, `alpha4` — rotational noise due to translation and rotation.
- `alpha5`, `alpha6` — additional drift noise.

This is the standard velocity motion model from *Probabilistic Robotics*, Table 5.3.

### 4. Sensor Model — `BeamRangeFinderModel`

Models an ultrasonic sensor with multiple beams. Each beam's measured distance is explained as a mixture of four phenomena:

| Component | Parameter | Meaning |
|-----------|-----------|---------|
| `zHit`    | Weight    | Correct detection — range matches expected obstacle distance |
| `zShort`  | Weight    | Unexpectedly short reading — an unknown obstacle in the way |
| `zMax`    | Weight    | Max-range reading — sensor returned its ceiling value (255) |
| `zRand`   | Weight    | Random noise — completely spurious measurement |

The **expected distance** for each beam (`z*`) is computed by **ray casting** on the current particle's occupancy grid: a ray is traced from the robot's hypothesized pose in the beam's direction until it hits a cell with log-odds occupancy above 50%.

The overall weight of a particle is the product of probabilities across all beams:  
`w = p(z1 | x, m) × p(z2 | x, m) × ... × p(zK | x, m)`

#### Sensor Parameter Learning

`BeamRangeFinderModel.Learn_intrinsic_parameters()` implements an EM-style iterative algorithm to estimate the four z-weights, `sigmaHit`, and `lambdaShort` from a set of real measurements and known poses. Useful for calibrating the sensor to a specific hardware setup.

### 5. Occupancy Grid — `OccupancyGridMap`

The environment is discretized into a 2D grid of `GridCell` objects. Each cell stores its occupancy as a **log-odds** value, which avoids numerical instability near 0 and 1:

```
log_odds = log( p / (1 - p) )
```

Cells are initialized to `log_odds(0.5) = 0` (unknown). When a sensor reading is processed via `UpdateMap()`, each cell in the sensor's perceptual field is updated using the **inverse sensor model**:
- Cell is beyond the measured range → **free** (log-odds decreases).
- Cell is at the measured range → **occupied** (log-odds increases).
- Cell is outside the sensor's field of view → **unchanged**.

### 6. Resampling — `SampleProbDistribution()`

After scoring all particles, the resampler draws N new particles by:
1. Computing a cumulative weight array.
2. Drawing a uniform random number scaled to total weight.
3. Using binary search to find the corresponding particle.

This is the standard importance resampling step that gives FastSLAM its particle-filter foundation.

---

## Key Parameters

| Parameter | Where set | Effect |
|-----------|-----------|--------|
| `numBeliefs` | `FastSLAMModel` constructor | Number of particles; more = more accurate but slower |
| `xCells`, `yCells` | `FastSLAMModel` constructor | Map dimensions in cells |
| `cellSize` | `FastSLAMModel` constructor | Physical size of each cell (cm) |
| `alpha1–alpha6` | `MotionVelocity` | Motion noise; tune to match real robot odometry error |
| `beta` | `BeamRangeFinderModel` | Sensor field-of-view angle (degrees, converted to radians internally) |
| `alpha` (sensor) | `BeamRangeFinderModel` | Obstacle thickness (cm); affects the occupied/free boundary |
| `sigmaHit` | `BeamRangeFinderModel` | Spread of the Gaussian hit distribution |
| `zHit/zShort/zMax/zRand` | `BeamRangeFinderModel` | Mixture weights for the four noise components |

---

## Running the Code

The project is a Visual Studio solution (`.NET Framework 4.8`). Open `Robotics.sln` and run the unit tests via the **Test Explorer**.

Key tests in `UnitTests/UnitTest1.cs`:

| Test | What it does |
|------|-------------|
| `SampleNormalDistribution_Test` | Samples 500 values from a Normal distribution; outputs to CSV for plotting |
| `SampleTriangularDistribution_Test` | Same for the Triangular distribution |
| `SampleMotionModelVelocity_Test` | Samples 500 poses from the velocity motion model; outputs to CSV |
| `SampleMotionModelOdometry_Test` | Same using the odometry motion model |
| `FastSLAM_Test` | Runs the full SLAM loop with a live Windows Forms visualization |
| `Learn_Parameters_Test` | Runs the EM sensor calibration algorithm on a CSV of sonar readings |

> **Note:** Some tests write output files to `C:\temp\TestingRobotics\` or `C:\temp\Robotics\UnitTests\`. Create those directories or update the paths before running.

---

## Sensor Setup

The implementation assumes an ultrasonic (sonar) sensor with multiple beams. In the tests, two beams are used:

```csharp
theta = new double[] { Math.PI / 2, -Math.PI / 2 }  // Left and right of robot heading
z     = new double[] { 49, 29 }                       // Distances in cm
```

Sensor range: `5 cm` (minimum) to `255 cm` (maximum, treated as "no obstacle detected").

---

## Dependencies

- **.NET Framework 4.8**
- **MSTest** (Visual Studio unit test framework)
- **System.Windows.Forms** (used only in `RobotWorldForm` for visualization)
- No external NuGet packages required

---

## C++ Port (ROS2-ready)

A complete C++ port of the algorithm lives in `cpp/`. It compiles as a standalone library and console runner on Linux (Ubuntu 22.04 / 24.04 with ROS2 Humble/Jazzy). The C# Windows Forms visualizer is replaced by optional OpenCV and native ROS2 RViz2 integration.

### Directory layout

```
cpp/
├── CMakeLists.txt
├── include/slam/
│   ├── pose.hpp                  # Pose struct (x, y, theta)
│   ├── grid_cell.hpp             # GridCell + log-odds helpers
│   ├── occupancy_grid_map.hpp    # 2-D occupancy grid
│   ├── control_rotation.hpp      # Velocity command (v, w)
│   ├── ultrasonic_measurement.hpp# Beam readings + geometry
│   ├── distributions.hpp         # Normal/Triangular samplers (std::mt19937)
│   ├── motion_velocity.hpp       # Velocity motion model (Table 5.3)
│   ├── beam_range_finder_model.hpp # Sensor model + ray-cast
│   ├── belief_occupancy_grid.hpp # One particle (pose + map + path)
│   └── fast_slam.hpp             # FastSLAM main loop + resampling
├── src/
│   ├── occupancy_grid_map.cpp
│   ├── beam_range_finder_model.cpp
│   ├── fast_slam.cpp
│   └── main.cpp                  # Console runner (optional OpenCV display)
└── tests/
    ├── CMakeLists.txt
    ├── test_motion_model.cpp     # Translations of MotionModelTests.cs
    ├── test_sensor_model.cpp     # Translations of SensorModelTuningTests.cs
    └── test_algorithm.cpp        # Translations of AlgorithmFixTests.cs
```

### Prerequisites (Ubuntu)

```bash
sudo apt update
sudo apt install -y build-essential cmake git
# Optional: live visualisation window
sudo apt install -y libopencv-dev
```

Google Test is downloaded automatically by CMake via `FetchContent`; no manual install is needed.

### Build (console runner + tests)

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

With OpenCV visualisation enabled:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DWITH_OPENCV=ON
cmake --build build -j$(nproc)
```

### Run the unit tests

```bash
cd cpp/build
ctest --output-on-failure
```

Or run the test binary directly (more verbose output):

```bash
./tests/slam_tests --gtest_color=yes
```

All 26 tests correspond 1-to-1 with the C# test suite. They cover:

| Suite | Tests | What is verified |
|---|---|---|
| `MotionModel` | 10 | Division-by-zero guard, straight-line/arc formulas, continuity at ε, noise wiring |
| `SensorModelTuning` | 6 | sigmaHit effect on weight ratios, PRand floor, convergence signal |
| `AlgorithmFix` | 10 | Map-clone isolation, particle count invariant, weight non-negativity, path history, sensor scoring |

### Run the algorithm locally (console)

```bash
./slam_runner --particles 100 --cells-x 20 --cells-y 20 --cell-size 10 \
              --v 5 --w 0.1 --z1 80 --z2 80 --iterations 50
```

Each step prints the highest-weight particle's pose:

```
Step    1  best particle: x:52.34 cm, y:48.11 cm, theta:6.04 deg  w=3.921e-04
Step    2  best particle: x:55.12 cm, y:47.88 cm, theta:6.03 deg  w=4.113e-04
...
```

Interactive mode (press Enter to step, `q` to quit):

```bash
./slam_runner --particles 100 --cells-x 20 --cells-y 20
```

With OpenCV (built with `-DWITH_OPENCV=ON`), a 500×500 window opens showing:
- **Red dots with heading arrows** — all particles.
- **Occupancy grid** of the highest-weight particle (black = occupied, white = free, grey = unknown).
- **Path line** of the highest-weight particle's trajectory.

### Visualization — OS considerations

| Environment | Solution |
|---|---|
| **Linux desktop** (Wayland/X11) | Build with `-DWITH_OPENCV=ON`; `imshow` uses GTK or Qt backend |
| **Headless Linux / SSH** | Console mode only (omit OpenCV flag); pipe output to a file for offline analysis |
| **ROS2 (Humble / Jazzy)** | Publish `nav_msgs/OccupancyGrid` + `geometry_msgs/PoseArray`; visualize in **RViz2** — no extra GUI code needed |
| **Windows (native build)** | Keep using the C# Windows Forms visualizer in `UnitTests/RobotWorldForm.cs` |

#### ROS2 node (future work)

Wire the C++ library into a ROS2 package:

```
ros2_ws/src/slam_node/
├── CMakeLists.txt          (ament_cmake, links slam library)
└── src/slam_node.cpp       (rclcpp node)
    Subscribers:
      sensor_msgs/msg/Range         → UltraSonicMeasurement
      geometry_msgs/msg/Twist       → ControlRotation (v, w)
    Publishers:
      nav_msgs/msg/OccupancyGrid    → best particle's map
      geometry_msgs/msg/PoseArray   → all particle poses
      nav_msgs/msg/Path             → best particle's path
```

The FastSLAM iteration runs inside the sensor callback (or a timer callback at fixed Hz). RViz2 subscribes to the three publishers and renders the map and particles natively.

---

## References

- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.  
  — The primary reference for all algorithms implemented here (FastSLAM, beam range finder model, velocity motion model, occupancy grid mapping).
