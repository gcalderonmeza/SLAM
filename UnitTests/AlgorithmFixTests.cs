using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Models;
using Algorithms;

namespace UnitTests
{
    [TestClass]
    public class AlgorithmFixTests
    {
        // -----------------------------------------------------------------------
        // Helpers
        // -----------------------------------------------------------------------

        private static ControlRotation MakeControl()
        {
            return new ControlRotation { v = 1.0, w = 0.1 };
        }

        private static UltraSonicMeasurement MakeMeasurement(double[] z, double[] theta)
        {
            return new UltraSonicMeasurement { z = z, theta = theta };
        }

        /// <summary>
        /// Builds a FastSLAMOccupancyGrid with realistic but numerically safe parameters.
        /// sigmaHit = 5 cm (much larger than the bug-era 0.13 cm) so that PRand does not
        /// dominate and weights are meaningfully different across particles.
        /// </summary>
        private static FastSLAMOccupancyGrid MakeFastSLAM()
        {
            FastSLAMOccupancyGrid slam = new FastSLAMOccupancyGrid();
            slam.motionVelocityModel = new MotionVelocity
            {
                alpha1 = 0.005,
                alpha2 = 0.005,
                alpha3 = 0.001,
                alpha4 = 0.001,
                alpha5 = 0.0,
                alpha6 = 0.0,
                sampler = Distributions.SampleNormal,
                dt = 0.5,
            };
            slam.measurementModel = new BeamRangeFinderModel(
                beta: 40,
                alpha: 10,
                zHit: 0.71,
                zShort: 0.08,
                zMax: 0.09,
                zRand: 0.12,
                sigmaHit: 5.0,
                lambdaShort: 0.5);
            return slam;
        }

        /// <summary>
        /// Returns a 10x10 grid (cellSize 10 cm) with every cell in column 3 (xi = 35 cm)
        /// marked as occupied.  A robot at (5, 5, 0) pointing right ray-casts to ~25 cm.
        /// </summary>
        private static OccupancyGridMap MakeMapWithObstacleAtColumn3()
        {
            int xCells = 10;
            int yCells = 10;
            double cellSize = 10.0;
            OccupancyGridMap map = new OccupancyGridMap(cellSize: cellSize, xCells: xCells, yCells: yCells);
            for (int y = 0; y < yCells; y++)
                map.m[3 + y * xCells].OccupancyLogOdds = GridCell.ConvertProbToLogOdds(0.9);
            return map;
        }

        // -----------------------------------------------------------------------
        // Bug-fix tests — map sharing
        // -----------------------------------------------------------------------

        /// <summary>
        /// Core regression test for the map-sharing bug.
        ///
        /// Before the fix, Iterate() updated belief.map in place and then passed
        /// the same reference into the new BeliefeOccupancyGrid.  When multiple
        /// particles shared a map (common after resampling), the next call to
        /// Iterate() applied every update to the same object.
        ///
        /// After the fix, Iterate() clones the map before updating, so the
        /// original input map is never modified.
        /// </summary>
        [TestMethod]
        public void Iterate_DoesNotModifyInputMap()
        {
            // All 5 particles intentionally share the same map (worst case of the bug).
            OccupancyGridMap sharedMap = new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10);
            double[] originalLogOdds = sharedMap.m.Select(c => c.OccupancyLogOdds).ToArray();

            List<BeliefeOccupancyGrid> beliefs = new List<BeliefeOccupancyGrid>();
            for (int i = 0; i < 5; i++)
            {
                beliefs.Add(new BeliefeOccupancyGrid(
                    pose: new Pose(50, 50, i * Math.PI / 5),
                    map: sharedMap,
                    path: null));
            }

            MakeFastSLAM().Iterate(beliefs, MakeControl(),
                MakeMeasurement(new double[] { 40.0 }, new double[] { 0.0 }));

            for (int i = 0; i < sharedMap.m.Length; i++)
            {
                Assert.AreEqual(
                    originalLogOdds[i],
                    sharedMap.m[i].OccupancyLogOdds,
                    1e-10,
                    "Cell " + i + " of the input map was modified. " +
                    "Iterate() must clone the map before calling UpdateMap().");
            }
        }

        /// <summary>
        /// Verifies that modifying one output particle's map does not affect
        /// a second output particle's map, confirming maps are independent objects
        /// after the fix is applied.
        /// </summary>
        [TestMethod]
        public void Iterate_OutputMaps_AreIndependentObjects()
        {
            int n = 10;
            List<BeliefeOccupancyGrid> beliefs = new List<BeliefeOccupancyGrid>();
            for (int i = 0; i < n; i++)
            {
                beliefs.Add(new BeliefeOccupancyGrid(
                    pose: new Pose(50, 50, 0),
                    map: new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10),
                    path: null));
            }

            List<FastSLAMOccupancyGrid.BeliefWeightPair> result =
                MakeFastSLAM().Iterate(beliefs, MakeControl(),
                    MakeMeasurement(new double[] { 40.0 }, new double[] { 0.0 }));

            List<OccupancyGridMap> maps = result.Select(r => (OccupancyGridMap)r.grid.map).ToList();

            // Record cell 0 value for all particles, then mutate particle 0.
            double[] before = maps.Select(m => m.m[0].OccupancyLogOdds).ToArray();
            maps[0].m[0].OccupancyLogOdds = 9999.0;

            for (int i = 1; i < maps.Count; i++)
            {
                Assert.AreEqual(
                    before[i],
                    maps[i].m[0].OccupancyLogOdds,
                    1e-10,
                    "Particle " + i + " map cell 0 changed when particle 0's map was mutated. " +
                    "Maps must be independent objects after Iterate().");
            }
        }

        // -----------------------------------------------------------------------
        // Basic algorithm invariants
        // -----------------------------------------------------------------------

        [TestMethod]
        public void Iterate_OutputParticleCount_EqualsInputCount()
        {
            int n = 50;
            List<BeliefeOccupancyGrid> beliefs = new List<BeliefeOccupancyGrid>();
            for (int i = 0; i < n; i++)
            {
                beliefs.Add(new BeliefeOccupancyGrid(
                    pose: new Pose(50, 50, i * 0.1),
                    map: new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10),
                    path: null));
            }

            List<FastSLAMOccupancyGrid.BeliefWeightPair> result =
                MakeFastSLAM().Iterate(beliefs, MakeControl(),
                    MakeMeasurement(new double[] { 40.0, 40.0 }, new double[] { 0.0, Math.PI / 2 }));

            Assert.AreEqual(n, result.Count,
                "Iterate() must return exactly as many particles as it received.");
        }

        [TestMethod]
        public void Iterate_AllOutputWeights_AreNonNegative()
        {
            int n = 20;
            List<BeliefeOccupancyGrid> beliefs = new List<BeliefeOccupancyGrid>();
            for (int i = 0; i < n; i++)
            {
                beliefs.Add(new BeliefeOccupancyGrid(
                    pose: new Pose(50, 50, i * 0.3),
                    map: new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10),
                    path: null));
            }

            List<FastSLAMOccupancyGrid.BeliefWeightPair> result =
                MakeFastSLAM().Iterate(beliefs, MakeControl(),
                    MakeMeasurement(new double[] { 30.0 }, new double[] { 0.0 }));

            foreach (FastSLAMOccupancyGrid.BeliefWeightPair pair in result)
            {
                Assert.IsTrue(pair.weight >= 0,
                    "Particle weight " + pair.weight + " is negative. " +
                    "Weights are products of probabilities and must be >= 0.");
            }
        }

        [TestMethod]
        public void Iterate_PoseHistory_GrowsByOnePerIteration()
        {
            int n = 5;
            List<BeliefeOccupancyGrid> beliefs = new List<BeliefeOccupancyGrid>();
            for (int i = 0; i < n; i++)
            {
                beliefs.Add(new BeliefeOccupancyGrid(
                    pose: new Pose(50, 50, 0),
                    map: new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10),
                    path: null));
            }

            FastSLAMOccupancyGrid slam = MakeFastSLAM();
            ControlRotation control = MakeControl();
            UltraSonicMeasurement meas = MakeMeasurement(new double[] { 40.0 }, new double[] { 0.0 });

            // The BeliefeOccupancyGrid constructor appends the initial pose, so path starts at 1.
            Assert.AreEqual(1, beliefs[0].path.Count, "Initial path length must be 1.");

            List<FastSLAMOccupancyGrid.BeliefWeightPair> result1 = slam.Iterate(beliefs, control, meas);
            Assert.AreEqual(2, result1[0].grid.path.Count,
                "After iteration 1 the path length must be 2.");

            List<BeliefeOccupancyGrid> beliefs2 = result1.Select(r => r.grid).ToList();
            List<FastSLAMOccupancyGrid.BeliefWeightPair> result2 = slam.Iterate(beliefs2, control, meas);
            Assert.AreEqual(3, result2[0].grid.path.Count,
                "After iteration 2 the path length must be 3.");
        }

        // -----------------------------------------------------------------------
        // Sensor model correctness
        // -----------------------------------------------------------------------

        /// <summary>
        /// A particle whose pose is consistent with the sensor reading must receive
        /// a strictly higher weight than a particle facing the wrong direction.
        /// Setup: obstacle at column 3 (x = 30-40 cm).
        ///   poseMatch    = (5, 5, 0)  — pointing right, ray hits column 3 at ~25 cm.
        ///   poseMismatch = (5, 5, pi) — pointing left, ray exits map, returns 255.
        /// Measurement z = 25 cm should score poseMatch much higher.
        /// </summary>
        [TestMethod]
        public void SensorModel_MatchingPose_GetsHigherWeight_ThanNonMatchingPose()
        {
            OccupancyGridMap map = MakeMapWithObstacleAtColumn3();
            Pose poseMatch    = new Pose(x: 5, y: 5, theta: 0);
            Pose poseMismatch = new Pose(x: 5, y: 5, theta: Math.PI);

            UltraSonicMeasurement measurement = MakeMeasurement(
                new double[] { 25.0 },
                new double[] { 0.0 });

            BeamRangeFinderModel sensor = MakeFastSLAM().measurementModel;

            double wMatch    = sensor.BeamRangeFinder(measurement, poseMatch,    map);
            double wMismatch = sensor.BeamRangeFinder(measurement, poseMismatch, map);

            Assert.IsTrue(wMatch > wMismatch,
                "Particle facing obstacle (w=" + wMatch.ToString("g4") + ") must outweigh " +
                "particle facing away (w=" + wMismatch.ToString("g4") + ").");
        }

        /// <summary>
        /// A max-range reading on an empty map should score higher than a short-range
        /// reading, because the ray-cast also returns max range on an empty map.
        /// </summary>
        [TestMethod]
        public void SensorModel_MaxRangeReading_GetsHigherWeight_OnEmptyMap()
        {
            OccupancyGridMap emptyMap = new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10);
            Pose pose = new Pose(x: 5, y: 5, theta: 0);

            BeamRangeFinderModel sensor = MakeFastSLAM().measurementModel;

            double wMax   = sensor.BeamRangeFinder(MakeMeasurement(new double[] { 255.0 }, new double[] { 0.0 }), pose, emptyMap);
            double wShort = sensor.BeamRangeFinder(MakeMeasurement(new double[] { 10.0  }, new double[] { 0.0 }), pose, emptyMap);

            Assert.IsTrue(wMax > wShort,
                "Max-range reading (w=" + wMax.ToString("g4") + ") should outweigh " +
                "a short reading (w=" + wShort.ToString("g4") + ") when the map predicts no obstacle.");
        }

        // -----------------------------------------------------------------------
        // Log-odds grid cell
        // -----------------------------------------------------------------------

        [TestMethod]
        public void GridCell_ConvertProbToLogOdds_RoundTrip()
        {
            double[] probs = new double[] { 0.1, 0.3, 0.5, 0.7, 0.9 };
            foreach (double p in probs)
            {
                double logOdds = GridCell.ConvertProbToLogOdds(p);
                double back    = GridCell.ConvertLogOddsToProb(logOdds);
                Assert.AreEqual(p, back, 1e-10,
                    "Round-trip failed for p=" + p + ": got " + back);
            }
        }

        [TestMethod]
        public void GridCell_ConvertProbToLogOdds_BoundaryThrows()
        {
            bool threw1 = false;
            try { GridCell.ConvertProbToLogOdds(-0.01); }
            catch (ArgumentOutOfRangeException) { threw1 = true; }
            Assert.IsTrue(threw1, "Expected ArgumentOutOfRangeException for p=-0.01");

            bool threw2 = false;
            try { GridCell.ConvertProbToLogOdds(1.01); }
            catch (ArgumentOutOfRangeException) { threw2 = true; }
            Assert.IsTrue(threw2, "Expected ArgumentOutOfRangeException for p=1.01");
        }

        [TestMethod]
        public void OccupancyGridMap_InitializedToUnknown()
        {
            OccupancyGridMap map = new OccupancyGridMap(cellSize: 10, xCells: 5, yCells: 5);
            double expectedLogOdds = GridCell.ConvertProbToLogOdds(0.5);

            foreach (GridCell cell in map.m)
            {
                Assert.AreEqual(expectedLogOdds, cell.OccupancyLogOdds, 1e-10,
                    "All cells must start at the unknown (50%) prior.");
                Assert.AreEqual(expectedLogOdds, cell.PriorOccupacyPob, 1e-10,
                    "Prior must also be 50%.");
            }
        }
    }
}
