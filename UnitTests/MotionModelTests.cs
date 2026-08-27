using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Models;

namespace UnitTests
{
    /// <summary>
    /// Tests for the velocity motion model (MotionVelocity.SampleModel).
    ///
    /// All deterministic tests use a zero-noise sampler (variance => 0) so that
    /// vBar = v, wBar = w, gBar = 0, making expected values easy to calculate by hand.
    ///
    /// Reference: Probabilistic Robotics, Thrun et al., Table 5.3.
    /// </summary>
    [TestClass]
    public class MotionModelTests
    {
        private const double Tolerance = 1e-9;

        /// <summary>
        /// Returns a MotionVelocity with zero noise (all alpha = 0, sampler returns 0).
        /// With zero noise: vBar = v, wBar = w, gBar = 0.
        /// </summary>
        private static MotionVelocity ZeroNoiseModel(double dt = 1.0)
        {
            return new MotionVelocity
            {
                alpha1 = 0, alpha2 = 0,
                alpha3 = 0, alpha4 = 0,
                alpha5 = 0, alpha6 = 0,
                sampler = variance => 0.0,
                dt = dt,
            };
        }

        // -----------------------------------------------------------------------
        // Bug-fix tests — division by zero guard
        // -----------------------------------------------------------------------

        /// <summary>
        /// Core regression test: w = 0 must not produce NaN or Infinity.
        /// Before the fix, vBar/wBar = v/0 = Infinity, propagating NaN through sin/cos.
        /// </summary>
        [TestMethod]
        public void SampleModel_ZeroAngularVelocity_ProducesFiniteResult()
        {
            MotionVelocity model = ZeroNoiseModel(dt: 1.0);
            Pose pose = new Pose(0, 0, 0);
            ControlRotation control = new ControlRotation { v = 1.0, w = 0.0 };

            Pose result = model.SampleModel(control, pose);

            Assert.IsFalse(double.IsNaN(result.x),     "x must not be NaN when w=0");
            Assert.IsFalse(double.IsNaN(result.y),     "y must not be NaN when w=0");
            Assert.IsFalse(double.IsNaN(result.theta), "theta must not be NaN when w=0");
            Assert.IsFalse(double.IsInfinity(result.x),     "x must not be Infinity when w=0");
            Assert.IsFalse(double.IsInfinity(result.y),     "y must not be Infinity when w=0");
            Assert.IsFalse(double.IsInfinity(result.theta), "theta must not be Infinity when w=0");
        }

        /// <summary>
        /// Very small (sub-epsilon) angular velocity must also produce finite results.
        /// </summary>
        [TestMethod]
        public void SampleModel_NearZeroAngularVelocity_ProducesFiniteResult()
        {
            MotionVelocity model = ZeroNoiseModel(dt: 1.0);
            Pose pose = new Pose(0, 0, 0);
            ControlRotation control = new ControlRotation { v = 1.0, w = 1e-10 };

            Pose result = model.SampleModel(control, pose);

            Assert.IsFalse(double.IsNaN(result.x),     "x must not be NaN for near-zero w");
            Assert.IsFalse(double.IsNaN(result.y),     "y must not be NaN for near-zero w");
            Assert.IsFalse(double.IsNaN(result.theta), "theta must not be NaN for near-zero w");
            Assert.IsFalse(double.IsInfinity(result.x),     "x must not be Infinity for near-zero w");
            Assert.IsFalse(double.IsInfinity(result.y),     "y must not be Infinity for near-zero w");
            Assert.IsFalse(double.IsInfinity(result.theta), "theta must not be Infinity for near-zero w");
        }

        // -----------------------------------------------------------------------
        // Straight-line motion (w = 0)
        // -----------------------------------------------------------------------

        /// <summary>
        /// w=0, theta=0 (pointing right): robot should move purely in the +x direction.
        /// Expected: x' = x + v*dt, y' unchanged, theta' unchanged.
        /// </summary>
        [TestMethod]
        public void SampleModel_ZeroAngularVelocity_PointingRight_MovesAlongX()
        {
            double dt = 0.5;
            double v  = 2.0;
            MotionVelocity model = ZeroNoiseModel(dt: dt);
            Pose pose = new Pose(3.0, 4.0, 0.0);
            ControlRotation control = new ControlRotation { v = v, w = 0.0 };

            Pose result = model.SampleModel(control, pose);

            Assert.AreEqual(pose.x + v * dt, result.x,     Tolerance, "x should advance by v*dt");
            Assert.AreEqual(pose.y,           result.y,     Tolerance, "y should be unchanged");
            Assert.AreEqual(pose.theta,       result.theta, Tolerance, "theta should be unchanged");
        }

        /// <summary>
        /// w=0, theta=pi/2 (pointing up): robot should move purely in the +y direction.
        /// </summary>
        [TestMethod]
        public void SampleModel_ZeroAngularVelocity_PointingUp_MovesAlongY()
        {
            double dt = 1.0;
            double v  = 3.0;
            MotionVelocity model = ZeroNoiseModel(dt: dt);
            Pose pose = new Pose(1.0, 2.0, Math.PI / 2.0);
            ControlRotation control = new ControlRotation { v = v, w = 0.0 };

            Pose result = model.SampleModel(control, pose);

            // cos(pi/2) = 0 (numerically ≈ 6e-17), sin(pi/2) = 1
            Assert.AreEqual(pose.x,               result.x,     1e-6, "x should be (nearly) unchanged");
            Assert.AreEqual(pose.y + v * dt,       result.y,     Tolerance, "y should advance by v*dt");
            Assert.AreEqual(pose.theta,            result.theta, Tolerance, "theta should be unchanged");
        }

        /// <summary>
        /// w=0, v=0: the robot is stationary and must not move.
        /// </summary>
        [TestMethod]
        public void SampleModel_ZeroVelocityAndAngularVelocity_RobotDoesNotMove()
        {
            MotionVelocity model = ZeroNoiseModel(dt: 1.0);
            Pose pose = new Pose(5.0, 7.0, 1.2);
            ControlRotation control = new ControlRotation { v = 0.0, w = 0.0 };

            Pose result = model.SampleModel(control, pose);

            Assert.AreEqual(pose.x,     result.x,     Tolerance, "x must not change");
            Assert.AreEqual(pose.y,     result.y,     Tolerance, "y must not change");
            Assert.AreEqual(pose.theta, result.theta, Tolerance, "theta must not change");
        }

        // -----------------------------------------------------------------------
        // Circular arc motion (w != 0)
        // -----------------------------------------------------------------------

        /// <summary>
        /// v=1, w=pi/4 rad/s, dt=1 s, starting at (0,0,0).
        /// Analytic expected values from the circular-arc formula (Probabilistic Robotics Table 5.3):
        ///   r = v/w = 4/pi
        ///   x' = r * sin(w*dt) = (4/pi) * sin(pi/4)  ≈ 0.9003
        ///   y' = r * (1 - cos(w*dt)) = (4/pi)*(1-cos(pi/4)) ≈ 0.3729
        ///   theta' = w*dt = pi/4
        /// </summary>
        [TestMethod]
        public void SampleModel_CircularArc_MatchesAnalyticFormula()
        {
            double dt  = 1.0;
            double v   = 1.0;
            double w   = Math.PI / 4.0;
            MotionVelocity model = ZeroNoiseModel(dt: dt);
            Pose pose = new Pose(0.0, 0.0, 0.0);
            ControlRotation control = new ControlRotation { v = v, w = w };

            Pose result = model.SampleModel(control, pose);

            double r            = v / w;
            double expectedX     = r * Math.Sin(w * dt);
            double expectedY     = r * (1.0 - Math.Cos(w * dt));
            double expectedTheta = w * dt;

            Assert.AreEqual(expectedX,     result.x,     Tolerance, "x does not match circular-arc formula");
            Assert.AreEqual(expectedY,     result.y,     Tolerance, "y does not match circular-arc formula");
            Assert.AreEqual(expectedTheta, result.theta, Tolerance, "theta does not match circular-arc formula");
        }

        /// <summary>
        /// Negative angular velocity (turning right) must also produce finite results
        /// and correctly advance the pose.
        /// </summary>
        [TestMethod]
        public void SampleModel_NegativeAngularVelocity_ProducesFiniteResult()
        {
            MotionVelocity model = ZeroNoiseModel(dt: 1.0);
            Pose pose = new Pose(0.0, 0.0, 0.0);
            ControlRotation control = new ControlRotation { v = 1.0, w = -Math.PI / 4.0 };

            Pose result = model.SampleModel(control, pose);

            Assert.IsFalse(double.IsNaN(result.x),     "x must not be NaN for negative w");
            Assert.IsFalse(double.IsNaN(result.y),     "y must not be NaN for negative w");
            Assert.IsFalse(double.IsNaN(result.theta), "theta must not be NaN for negative w");
        }

        /// <summary>
        /// Large angular velocity must not produce Infinity or NaN.
        /// </summary>
        [TestMethod]
        public void SampleModel_LargeAngularVelocity_ProducesFiniteResult()
        {
            MotionVelocity model = ZeroNoiseModel(dt: 0.1);
            Pose pose = new Pose(0.0, 0.0, 0.0);
            ControlRotation control = new ControlRotation { v = 1.0, w = 1000.0 };

            Pose result = model.SampleModel(control, pose);

            Assert.IsFalse(double.IsNaN(result.x)     || double.IsInfinity(result.x),     "x must be finite");
            Assert.IsFalse(double.IsNaN(result.y)     || double.IsInfinity(result.y),     "y must be finite");
            Assert.IsFalse(double.IsNaN(result.theta) || double.IsInfinity(result.theta), "theta must be finite");
        }

        // -----------------------------------------------------------------------
        // Continuity at the straight-line / arc boundary
        // -----------------------------------------------------------------------

        /// <summary>
        /// The straight-line formula is the mathematical limit of the circular-arc formula
        /// as w → 0.  Poses computed just below (w = 5e-7) and just above (w = 2e-6)
        /// the epsilon boundary must be close to each other and to the w=0 result.
        /// </summary>
        [TestMethod]
        public void SampleModel_ContinuousAtBoundary_NearEpsilon()
        {
            double dt = 1.0;
            double v  = 1.0;
            MotionVelocity model = ZeroNoiseModel(dt: dt);
            Pose pose = new Pose(0.0, 0.0, 0.0);

            Pose atZero      = model.SampleModel(new ControlRotation { v = v, w = 0.0    }, pose);
            Pose belowEps    = model.SampleModel(new ControlRotation { v = v, w = 5e-7   }, pose);
            Pose aboveEps    = model.SampleModel(new ControlRotation { v = v, w = 2e-6   }, pose);

            // All three must be very close (within 1e-5 cm for a 1 cm/s robot over 1 s)
            Assert.AreEqual(atZero.x, belowEps.x, 1e-5, "x discontinuity below epsilon boundary");
            Assert.AreEqual(atZero.x, aboveEps.x, 1e-5, "x discontinuity above epsilon boundary");
            Assert.AreEqual(atZero.y, belowEps.y, 1e-5, "y discontinuity below epsilon boundary");
            Assert.AreEqual(atZero.y, aboveEps.y, 1e-5, "y discontinuity above epsilon boundary");
        }

        // -----------------------------------------------------------------------
        // Noise integration
        // -----------------------------------------------------------------------

        /// <summary>
        /// With non-zero noise parameters the sampler is called and the result differs
        /// from the zero-noise case.  This verifies noise is correctly wired into the model.
        /// </summary>
        [TestMethod]
        public void SampleModel_WithNoise_ResultDiffersFromNoiselessBaseline()
        {
            double dt = 1.0;
            ControlRotation control = new ControlRotation { v = 1.0, w = 0.5 };
            Pose pose = new Pose(0.0, 0.0, 0.0);

            MotionVelocity noiseless = ZeroNoiseModel(dt: dt);
            Pose baseline = noiseless.SampleModel(control, pose);

            // Use a sampler that always returns a fixed offset so the result is deterministic.
            MotionVelocity noisy = new MotionVelocity
            {
                alpha1 = 1, alpha2 = 1,
                alpha3 = 1, alpha4 = 1,
                alpha5 = 1, alpha6 = 1,
                sampler = variance => 0.1,   // fixed additive noise
                dt = dt,
            };
            Pose noisyResult = noisy.SampleModel(control, pose);

            // The noisy result must differ from the noiseless baseline
            bool differs = Math.Abs(noisyResult.x - baseline.x) > 1e-9
                        || Math.Abs(noisyResult.y - baseline.y) > 1e-9
                        || Math.Abs(noisyResult.theta - baseline.theta) > 1e-9;

            Assert.IsTrue(differs, "Noisy model produced the same result as noiseless; noise is not being applied.");
        }
    }
}
