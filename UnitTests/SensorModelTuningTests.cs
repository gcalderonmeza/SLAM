using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Models;

namespace UnitTests
{
    /// <summary>
    /// Tests that demonstrate why sigmaHit must be set to a realistic value for the
    /// sensor model to drive convergence.
    ///
    /// Background
    /// ----------
    /// PHit(z, z*) is proportional to exp( -0.5 * (z - z*)^2 / sigmaHit^2 ).
    /// It rewards particles whose map predicts a range (z*) close to the actual reading (z).
    ///
    /// If sigmaHit is far smaller than the sensor's real noise, any realistic deviation
    /// |z - z*| collapses PHit to ~0.  All particles then score identically through PRand
    /// alone (1/zMax per beam), resampling becomes random, and the filter cannot converge.
    ///
    /// A sigmaHit of 5-20 cm is appropriate for a typical HC-SR04 ultrasonic sensor
    /// (hardware noise ~1-3 cm, plus map discretisation error).  The exact value should
    /// be calibrated with BeamRangeFinderModel.Learn_intrinsic_parameters().
    /// </summary>
    [TestClass]
    public class SensorModelTuningTests
    {
        // Map: 10x10 grid, 10 cm cells.  Column 3 is occupied.
        // Robot at (5, 5, 0) pointing right: ray hits column 3 at ~25 cm (kTStar = 25).
        // Robot at (5, 5, pi) pointing left: ray exits map with no hit (kTStar = 255).
        private const int XCells = 10;
        private const int YCells = 10;
        private const double CellSize = 10.0;

        private static OccupancyGridMap MakeMapWithObstacleAtColumn3()
        {
            OccupancyGridMap map = new OccupancyGridMap(cellSize: CellSize, xCells: XCells, yCells: YCells);
            for (int y = 0; y < YCells; y++)
                map.m[3 + y * XCells].OccupancyLogOdds = GridCell.ConvertProbToLogOdds(0.9);
            return map;
        }

        private static UltraSonicMeasurement MakeMeasurement(double z, double theta)
        {
            return new UltraSonicMeasurement
            {
                z = new double[] { z },
                theta = new double[] { theta },
            };
        }

        private static BeamRangeFinderModel MakeSensor(double sigmaHit)
        {
            return new BeamRangeFinderModel(
                beta: 40, alpha: 10,
                zHit: 0.71, zShort: 0.08, zMax: 0.09, zRand: 0.12,
                sigmaHit: sigmaHit, lambdaShort: 0.5);
        }

        // -----------------------------------------------------------------------
        // Core tests: weight ratio as a function of sigmaHit
        // -----------------------------------------------------------------------

        /// <summary>
        /// With a realistic sigmaHit (5 cm) the matching particle must score at least
        /// 10x higher than the mismatching particle.  This ratio is what drives resampling
        /// to keep good hypotheses and discard bad ones.
        /// </summary>
        [TestMethod]
        public void PHit_AppropiateSigma_MatchingParticleOutscoresMismatching_ByLargeFactor()
        {
            OccupancyGridMap map = MakeMapWithObstacleAtColumn3();
            UltraSonicMeasurement meas = MakeMeasurement(z: 25.0, theta: 0.0);

            Pose poseMatch    = new Pose(x: 5, y: 5, theta: 0);       // kTStar ≈ 25 cm
            Pose poseMismatch = new Pose(x: 5, y: 5, theta: Math.PI); // kTStar = 255 cm

            BeamRangeFinderModel sensor = MakeSensor(sigmaHit: 5.0);

            double wMatch    = sensor.BeamRangeFinder(meas, poseMatch,    map);
            double wMismatch = sensor.BeamRangeFinder(meas, poseMismatch, map);

            double ratio = wMatch / wMismatch;

            Assert.IsTrue(ratio > 10.0,
                "With sigmaHit=5 cm the matching particle should score >10x higher than " +
                "the mismatching one. Actual ratio: " + ratio.ToString("f1"));
        }

        /// <summary>
        /// With an unrealistically tight sigmaHit (0.13 cm), PHit collapses to ~0 for any
        /// realistic deviation.  The weight ratio between matching and mismatching particles
        /// drops to near 1 — resampling becomes random and convergence is impossible.
        /// </summary>
        [TestMethod]
        public void PHit_TightSigma_MatchingAndMismatchingParticlesGetSimilarWeights()
        {
            OccupancyGridMap map = MakeMapWithObstacleAtColumn3();
            UltraSonicMeasurement meas = MakeMeasurement(z: 25.0, theta: 0.0);

            Pose poseMatch    = new Pose(x: 5, y: 5, theta: 0);
            Pose poseMismatch = new Pose(x: 5, y: 5, theta: Math.PI);

            BeamRangeFinderModel sensor = MakeSensor(sigmaHit: 0.13);

            double wMatch    = sensor.BeamRangeFinder(meas, poseMatch,    map);
            double wMismatch = sensor.BeamRangeFinder(meas, poseMismatch, map);

            // Both weights are dominated by PRand (1/255).  Ratio should be close to 1.
            double ratio = wMatch / wMismatch;

            Assert.IsTrue(ratio < 5.0,
                "With sigmaHit=0.13 cm PHit is near 0 for both particles so the ratio " +
                "should be close to 1 (PRand dominates).  Actual ratio: " + ratio.ToString("f1"));
        }

        // -----------------------------------------------------------------------
        // PHit shape: score decreases as measurement deviates from expected range
        // -----------------------------------------------------------------------

        /// <summary>
        /// For a sensor with 5 cm sigmaHit, weights must strictly decrease as the
        /// measurement moves away from the map-predicted distance.
        /// This verifies that PHit is the dominant term and forms a meaningful gradient.
        /// </summary>
        [TestMethod]
        public void PHit_AppropiateSigma_WeightDecreasesAsDeviationIncreases()
        {
            OccupancyGridMap map = MakeMapWithObstacleAtColumn3();
            Pose pose = new Pose(x: 5, y: 5, theta: 0); // kTStar ≈ 25 cm

            BeamRangeFinderModel sensor = MakeSensor(sigmaHit: 5.0);

            // Measurements at 0, 5, 10, 15 cm deviation from the expected 25 cm
            double wAt25  = sensor.BeamRangeFinder(MakeMeasurement(25.0, 0.0), pose, map);
            double wAt30  = sensor.BeamRangeFinder(MakeMeasurement(30.0, 0.0), pose, map);
            double wAt35  = sensor.BeamRangeFinder(MakeMeasurement(35.0, 0.0), pose, map);
            double wAt40  = sensor.BeamRangeFinder(MakeMeasurement(40.0, 0.0), pose, map);

            Assert.IsTrue(wAt25 > wAt30,
                "Weight at z=25 (perfect match) must exceed weight at z=30 (+5 cm off).");
            Assert.IsTrue(wAt30 > wAt35,
                "Weight at z=30 must exceed weight at z=35 as deviation grows.");
            Assert.IsTrue(wAt35 > wAt40,
                "Weight at z=35 must exceed weight at z=40 as deviation grows.");
        }

        /// <summary>
        /// For an unrealistically tight sigmaHit (0.13 cm), the weight is near-uniform
        /// across a wide measurement range — PHit contributes nothing meaningful.
        /// </summary>
        [TestMethod]
        public void PHit_TightSigma_WeightIsNearlyFlatAcrossReasonableRange()
        {
            OccupancyGridMap map = MakeMapWithObstacleAtColumn3();
            Pose pose = new Pose(x: 5, y: 5, theta: 0); // kTStar ≈ 25 cm

            BeamRangeFinderModel sensor = MakeSensor(sigmaHit: 0.13);

            // A real sonar might read anywhere between 20 and 30 cm due to noise.
            // With sigmaHit=0.13, all of these should get essentially the same weight.
            double wAt25 = sensor.BeamRangeFinder(MakeMeasurement(25.0, 0.0), pose, map);
            double wAt27 = sensor.BeamRangeFinder(MakeMeasurement(27.0, 0.0), pose, map);
            double wAt30 = sensor.BeamRangeFinder(MakeMeasurement(30.0, 0.0), pose, map);

            // The ratio between best and worst should be < 2 (PRand flatness dominates)
            double maxW = Math.Max(wAt25, Math.Max(wAt27, wAt30));
            double minW = Math.Min(wAt25, Math.Min(wAt27, wAt30));
            double flatnessRatio = maxW / minW;

            Assert.IsTrue(flatnessRatio < 2.0,
                "With sigmaHit=0.13 cm all weights in a realistic noise range should be " +
                "nearly equal (flatnessRatio < 2). Actual: " + flatnessRatio.ToString("f3"));
        }

        // -----------------------------------------------------------------------
        // PRand floor: weights never reach absolute zero
        // -----------------------------------------------------------------------

        /// <summary>
        /// Even with a completely wrong measurement, PRand provides a non-zero floor.
        /// The particle filter must never assign weight 0 (which would cause an infinite
        /// loop in SampleProbDistribution when total weight = 0).
        /// </summary>
        [TestMethod]
        public void BeamRangeFinder_Weight_IsAlwaysPositive()
        {
            OccupancyGridMap map = new OccupancyGridMap(cellSize: CellSize, xCells: XCells, yCells: YCells);
            Pose pose = new Pose(x: 5, y: 5, theta: 0);

            double[] sigmaValues = new double[] { 0.1, 0.5, 1.0, 5.0, 20.0 };
            double[] zValues     = new double[] { 1.0, 50.0, 100.0, 200.0 };

            foreach (double sigma in sigmaValues)
            {
                BeamRangeFinderModel sensor = MakeSensor(sigmaHit: sigma);
                foreach (double z in zValues)
                {
                    double w = sensor.BeamRangeFinder(MakeMeasurement(z, 0.0), pose, map);
                    Assert.IsTrue(w > 0,
                        "Weight must be > 0 for sigma=" + sigma + " z=" + z +
                        " (PRand must provide a floor). Got: " + w);
                }
            }
        }

        // -----------------------------------------------------------------------
        // Convergence-driving ratio: quantify improvement from sigma fix
        // -----------------------------------------------------------------------

        /// <summary>
        /// Documents the quantitative improvement: the weight ratio between a matching
        /// and mismatching particle must be at least 100x larger with sigmaHit=5 cm
        /// than with sigmaHit=0.13 cm.
        /// </summary>
        [TestMethod]
        public void PHit_AppropiateSigma_WeightRatioMuchHigherThan_TightSigma()
        {
            OccupancyGridMap map = MakeMapWithObstacleAtColumn3();
            UltraSonicMeasurement meas = MakeMeasurement(z: 25.0, theta: 0.0);
            Pose poseMatch    = new Pose(x: 5, y: 5, theta: 0);
            Pose poseMismatch = new Pose(x: 5, y: 5, theta: Math.PI);

            BeamRangeFinderModel sensorTight = MakeSensor(sigmaHit: 0.13);
            double wMatchTight    = sensorTight.BeamRangeFinder(meas, poseMatch,    map);
            double wMismatchTight = sensorTight.BeamRangeFinder(meas, poseMismatch, map);
            double ratioTight = wMatchTight / wMismatchTight;

            BeamRangeFinderModel sensorGood = MakeSensor(sigmaHit: 5.0);
            double wMatchGood    = sensorGood.BeamRangeFinder(meas, poseMatch,    map);
            double wMismatchGood = sensorGood.BeamRangeFinder(meas, poseMismatch, map);
            double ratioGood = wMatchGood / wMismatchGood;

            Assert.IsTrue(ratioGood > 100.0 * ratioTight,
                "The weight ratio with sigmaHit=5 cm should be at least 100x larger than " +
                "with sigmaHit=0.13 cm. ratioGood=" + ratioGood.ToString("f1") +
                ", ratioTight=" + ratioTight.ToString("f3"));
        }
    }
}
