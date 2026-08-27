using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    /// <summary>
    /// Represents a real beam range finder
    /// </summary>
    public class BeamRangeFinderModel
    {
        private const double maxValue = 255.0;
        private const double minValue = 5.0;

        /// <summary>
        /// Probability of detecting a real obstacle
        /// </summary>
        private double zHit;

        /// <summary>
        /// Probability of measuring a range that is shorter than the correct one
        /// </summary>
        private double zShort;

        /// <summary>
        /// Probability of measuring a maximum range when the range is shorter
        /// </summary>
        public double zMax;

        /// <summary>
        /// Probability of measuring a random value
        /// </summary>
        private double zRand;

        /// <summary>
        /// Likelyhood of sensing unexpected objects
        /// </summary>
        private double lambdaShort;

        /// <summary>
        /// Thickness of obstacles (in cm)
        /// </summary>
        private double alpha;

        /// <summary>
        /// Width of field of view (angle, degrees)
        /// </summary>
        public double beta;

        /// <summary>
        /// Log odds probability of occupied cells
        /// </summary>
        private double locc;

        /// <summary>
        /// Log odds probability of free cells
        /// </summary>
        private double lfree;

        /// <summary>
        /// Standard deviation of the hit distribution
        /// </summary>
        private double sigmaHit;

        /// <summary>
        /// Initializes and instance of the type BeamRangeFinderModel type
        /// </summary>
        /// <param name="beta">Width of the field of view (in degrees)</param>
        /// <param name="alpha">The width of the obstacles (in cm)</param>
        /// <param name="zHit">Probability of detecting a real obstacle</param>
        /// <param name="zShort">Probability of measuring a range that is shorter than the correct one</param>
        /// <param name="zMax">Probability of measuring a maximum range when the range is shorter</param>
        /// <param name="zRand">Probability of measuring a random value</param>
        /// <param name="sigmaHit">Standard deviation of the hit distribution</param>
        /// <param name="lambdaShort">Likelyhood of sensing unexpected objects</param>
        public BeamRangeFinderModel(double beta, double alpha, double zHit, double zShort, double zMax, double zRand, double sigmaHit, double lambdaShort)
        {
            this.beta = beta * Math.PI / 180.0;
            this.alpha = alpha;
            this.zHit = zHit;
            this.zShort = zShort;
            this.zMax = zMax;
            this.zRand = zRand;
            this.sigmaHit = sigmaHit;

            // In log odds this means 50%
            this.lfree = Math.Log(0.2 / (1 - .2));
            this.locc = Math.Log(0.8 / (1 - 0.8));

            this.lambdaShort = lambdaShort;
        }

        /// <summary>
        /// Compute the probability of the measurement (zT) given the pose (xT) according to map (m)
        /// </summary>
        /// <param name="zT">Measurements (i.e. beams in the sensor)</param>
        /// <param name="xT">Pose of the robot (belief)</param>
        /// <param name="m">Current map of the environment</param>
        /// <returns>The probability of the measurement (zT) given the pose (xT) according to map (m)</returns>
        public double BeamRangeFinder(UltraSonicMeasurement zT, Pose xT, MapBase m)
        {
            double q = 1;

            // Iterate for all the beams in the sensor (i.e.: measurements)
            for (int k = 0; k < zT.Beams; k++)
            {
                // Expected distance computed using ray casting
                double kTStar = this.ComputeZStar(xT, m, zT.theta[k]);

                // Probabilities of hit, short, max, random for the measurements
                double pHitFactor = this.PHit(zT.z[k], m, kTStar);
                double pShortFactor = this.PShort(zT.z[k], m, kTStar);
                double pMaxFactor = this.PMax(zT.z[k], m);
                double pRandFactor = this.PRand(zT.z[k], m);

                // Combined probability of the measurement
                double p = this.zHit * pHitFactor + this.zShort * pShortFactor +
                this.zMax * pMaxFactor + this.zRand * pRandFactor;

                q *= p;
            }

            return q;
        }

        /// <summary>
        /// Probability of reading the correct value from the sensor
        /// </summary>
        /// <param name="zT">Measurement value</param>
        /// <param name="m">Current map</param>
        /// <param name="kTStar">Expected (correct) value computed from the current pose</param>
        /// <returns>The probability of reading the correct value from the sensor</returns>
        private double PHit(double zT, MapBase m, double kTStar)
        {
            if (zT >= 0 && zT <= maxValue)
            {
                double sigmaHit2 = this.sigmaHit * this.sigmaHit;
                double eta = this.ComputeEta(mean: kTStar, variance: sigmaHit2);
                double normal = Distributions.Normal(z: zT, mean: kTStar, variance: sigmaHit2);

                return eta * normal;
            }

            return 0;
        }

        private double ComputeEtaIterative(double mean, double variance)
        {
            //// Equals 1 / Integ(Normal(z, z*, sigma ^ 2), z in [0, zMax])
            double cummulative = 0;
            double low = 0;
            double delta = (maxValue / 500.0);
            double previousValue = Distributions.Normal(low, mean, variance);
            double nextValue;
            while (low + delta <= maxValue)
            {
                nextValue = Distributions.Normal(low + delta, mean, variance);
                cummulative += delta * (previousValue + nextValue);
                low += delta;
                previousValue = nextValue;
            }

            if (low + delta > maxValue)
            {
                cummulative += delta * (previousValue + Distributions.Normal(maxValue, mean, variance));
            }

            return cummulative > 0 ? 2.0/cummulative : 0;
        }

        /// <summary>
        /// Compute the normalizing factor for a normal distrbution with the given mean and variance
        /// </summary>
        /// <param name="mean">The mean of the distribution</param>
        /// <param name="variance">The variance of the distribution</param>
        /// <returns>The normalizing factor (area under the curve) of the distrbution</returns>
        private double ComputeEta(double mean, double variance)
        {
            //// Equals 1 / Integ(Normal(z, z*, sigma ^ 2), z in [0, zMax])
            // double integral = Distributions.DefiniteIntegralNormal(mean: mean, variance: variance, lowerLimit: 0.0, upperLimit: this.zMax);
            // return 1.0/integral;
            return this.ComputeEtaIterative(mean, variance);
        }

        /// <summary>
        /// Probability of reading a value that is shorter than the expected value (kt*)
        /// </summary>
        /// <param name="zT">Measurement value</param>
        /// <param name="m">Current map</param>
        /// <param name="kTStar">Expected range value. Computed from the current pose.</param>
        /// <returns>The probability of reading a value that is shorter than the expected value (kt*)</returns>
        private double PShort(double zT, MapBase m, double kTStar)
        {
            if (zT >= 0 && zT < kTStar)
            {
                double eta = 1 - Math.Exp(-lambdaShort * kTStar);
                return lambdaShort * Math.Exp(-lambdaShort * zT) / eta;
            }

            return 0;
        }

        /// <summary>
        /// Probability of reading a max value from the sensor
        /// </summary>
        /// <param name="zT">Measurement value</param>
        /// <param name="m">Current map</param>
        /// <returns>the probability of reading a max value from the sensor</returns>
        private double PMax(double zT, MapBase m)
        {
            return (zT == maxValue) ? 1.0 : 0.0;
        }

        /// <summary>
        /// Probablity of a random value from the sensor
        /// </summary>
        /// <param name="zT">Measurement value</param>
        /// <param name="m">Currnet map</param>
        /// <returns>The probablity of a random value from the sensor</returns>
        private double PRand(double zT, MapBase m)
        {
            return (zT >= 0 && zT < maxValue) ? 1.0 / maxValue : 0.0;
        }

        /// <summary>
        /// Computes the probability (log odds format) of being in xT given the current measurements (zT)
        /// </summary>
        /// <param name="mi">Cell being considered</param>
        /// <param name="xT">Robot pose</param>
        /// <param name="zT">Measurement</param>
        /// <returns></returns>
        public double InverseRangeSensorModel(GridCell mi, Pose xT, UltraSonicMeasurement zT, int indexToClosestBeam = -1, double angleToClosestBeam = double.MaxValue, double distanceToCenterOfMass = 0)
        {
            if (indexToClosestBeam == -1)
            {
                distanceToCenterOfMass = zT.FindRangeAndAngleToClosestBeam(mi, xT, out angleToClosestBeam, out indexToClosestBeam);
            }

            // Decide if occupied, free, or don't know
            // First if the distance to the center of mass is greater than the measured range or the angle to the closest beam is greater than the field of view of the sensor, then don't know
            if (distanceToCenterOfMass > Math.Min(maxValue, zT.z[indexToClosestBeam] + this.alpha / 2.0) || angleToClosestBeam > this.beta / 2.0)
            {
                return mi.PriorOccupacyPob;
            }

            // Second if the measured range is smaller than the max range and measured range makes sense (could be singnaling an obstacle), then say probably occupied
            if (zT.z[indexToClosestBeam] < maxValue && Math.Abs(distanceToCenterOfMass - zT.z[indexToClosestBeam]) < this.alpha / 2.0)
            {
                return this.locc;
            }

            // Third, distance to center of mass smaller than or equal to measured range, say probably free
            if (distanceToCenterOfMass <= zT.z[indexToClosestBeam])
            {
                return this.lfree;
            }

            // Error condition, no way to decide!
            throw new Exception("No value to return");
        }

        /// <summary>
        /// Learns the intrinsic parameters of the sensor (i.e. all the z* parameters, plus the sigmaHit and lambdaShort)
        /// </summary>
        /// <param name="z">Set of measurements to learn from</param>
        /// <param name="x">Set of poses to learn from</param>
        /// <param name="m">The map of the environment</param>
        public void Learn_intrinsic_parameters(UltraSonicMeasurement[] z, Pose[] x, OccupancyGridMap m)
        {
            double threshold = 0.05;

            double prezHit;
            double prezShort;
            double prezMax;
            double prezRand;
            double preSigmaHit;
            double preLambdaShort;

            do
            {
                prezHit = this.zHit;
                prezShort = this.zShort;
                prezMax = this.zMax;
                prezRand = this.zRand;
                preSigmaHit = this.sigmaHit;
                preLambdaShort = this.lambdaShort;

                double[] e_hit = new double[z.Length];
                double[] e_short = new double[z.Length];
                double[] e_max = new double[z.Length];
                double[] e_rand = new double[z.Length];
                double[] zStar = new double[z.Length];

                // Iterate over the measurements
                for (int i=0;i<z.Length;i++)
                {
                    // Error in the algorihtm (book), z* is needed to compute pHit for instance... So it needs to be computed before eta
                    // Compute the expected range using ray casting and the current map.
                    // NOTE: Using only one beam of the sensor
                    zStar[i] = this.ComputeZStar(x[i], m, z[i].theta[1]);

                    // Probabilities of hit, short, max, and random
                    double hit = this.PHit(z[i].z[1], m, zStar[i]);
                    double shrt = this.PShort(z[i].z[1], m, zStar[i]);
                    double mx = this.PMax(z[i].z[1], m);
                    double rnd = PRand(z[i].z[1], m);

                    // Normalization factor
                    double eta_1 = hit + shrt + mx + rnd;

                    // Store normalized values
                    e_hit[i] = hit / eta_1;
                    e_short[i] = shrt / eta_1;
                    e_max[i] = mx / eta_1;
                    e_rand[i] = rnd / eta_1;
                }

                // Compute the parameters based on the stored values for all the iterations
                this.zHit = e_hit.Sum() / z.Length;
                this.zShort = e_short.Sum() / z.Length;
                this.zMax = e_max.Sum() / z.Length;
                this.zRand = e_rand.Sum() / z.Length;

                double[] tempZ = z.Select(e => e.z[1]).ToArray();

                this.sigmaHit = Math.Sqrt(e_hit.Prod(tempZ.Diff(zStar).Square()).Sum()) / e_hit.Sum();
                this.lambdaShort = e_short.Sum() / e_short.Prod(tempZ).Sum();
            }
            while (Math.Abs((prezHit - this.zHit) / this.zHit) > threshold || 
                Math.Abs((prezShort - this.zShort) / this.zShort) > threshold || 
                Math.Abs((prezRand - this.zRand) / this.zRand) > threshold || 
                Math.Abs((prezMax - this.zMax) / this.zMax) > threshold ||
                Math.Abs((preSigmaHit - this.sigmaHit) / this.sigmaHit) > threshold ||
                Math.Abs((preLambdaShort - this.lambdaShort) / this.lambdaShort) > threshold);
        }

        /// <summary>
        /// <para>Compute the expected range using ray casting on an accupancy grid.</para>
        /// <para>Premises: The map starts at 0,0 (left, bottom corner of it).</para>
        /// <para>          The orientation of the robot (theta) is 0 deg when pointing to the right, 90 deg when pointing upwards (North), 180 deg when pointing left (East)</para>
        /// <para>          The range is measured between the centers of gravity of the cells.</para>
        /// </summary>
        /// <param name="xT">Current pose</param>
        /// <param name="m">Current map</param>
        /// <param name="angleRobotCoordinates">The angle of the beam with respect to the robot</param>
        /// <returns>The expected range</returns>
        private double ComputeZStar(Pose xT, MapBase m, double angleRobotCoordinates)
        {
            OccupancyGridMap map = m as OccupancyGridMap;

            if (map == null)
            {
                throw new NullReferenceException("The map is not an OccupancyGridMap");
            }

            // TODO find a better name for this function, z* is not a good name
            double angleRad = (xT.theta + angleRobotCoordinates);
            double angleDeg = angleRad * 180.0 / Math.PI;

            // The components of a vector pointing in the same direction the sensor is pointing to, but with r = 1
            double deltax = Math.Cos(angleRad);
            double deltay = Math.Sin(angleRad);
            double cellSize = map.CellSize;

            double rx2;
            double ry2;
            MoveToNextCell(xTemp: xT, cellSize: cellSize, deltax: deltax, deltay: deltay, rx2: out rx2, ry2: out ry2);

            // Compute in the direction of the sensor, not the robot only
            Pose xTemp = new Pose(x: xT.x, y: xT.y, theta: angleRad);

            xTemp.x += rx2;
            xTemp.y += ry2;

            Tuple<int, int> tuple = map.GetCellFromPose(xTemp);

            ////
            //// Now it is about determining of the ray (rx1, ry1), (rx2, ry2) intersects each of the boxes in the grid.
            //// 
            ////
            // The most naive implementation will move (imaginary move) along a line in the same direction the robot is moving and determine if the ray intersects any occupied cell
            // Assume the robot is not at (in) an occupied cell, nor inside a hollow box. So the first occupied cell found must be the start (external suface) of an obstacle
            double logOddsThreshold = GridCell.ConvertProbToLogOdds(0.5);
            while (tuple.Item1 >= 0 && tuple.Item1 < map.xCells && tuple.Item2 >= 0 && tuple.Item2 < map.yCells)
            {
                if (map.m[tuple.Item1+tuple.Item2 * map.yCells].OccupancyLogOdds > logOddsThreshold)
                {
                    double deltaX = xTemp.x - xT.x;
                    double deltaY = xTemp.y - xT.y;
                    return Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
                }

                MoveToNextCell(xTemp: xTemp, cellSize: cellSize, deltax: deltax, deltay: deltay, rx2: out rx2, ry2: out ry2);

                xTemp.x += rx2;
                xTemp.y += ry2;

                tuple = map.GetCellFromPose(xTemp);
            }

            return 255;
        }

        private static void MoveToNextCell(Pose xTemp, double cellSize, double deltax, double deltay, out double rx2, out double ry2)
        {
            double dx = deltax < 0 ? (cellSize * Math.Floor(xTemp.x / cellSize) - xTemp.x) / deltax : (deltax > 0) ? (cellSize * Math.Floor((xTemp.x + cellSize) / cellSize) - xTemp.x) / deltax : 0;
            double dy = deltay < 0 ? (cellSize * Math.Floor(xTemp.y / cellSize) - xTemp.y) / deltay : (deltay > 0) ? (cellSize * Math.Floor((xTemp.y + cellSize) / cellSize) - xTemp.y) / deltay : 0;

            // Factor measures how many deltax/y can the ray travel in the direction of the sensor (in world coordinates) to cross the cell boundaries
            double factor;

            if (deltax != 0 && deltay != 0)
            {
                factor = Math.Min(dx, dy);
            }
            else if (deltax == 0) 
            {
                factor = dy;
            }
            else {
                factor = dx;
            }

            rx2 = deltax * factor * (deltax < 0 ? 1.00001 : 1.0);
            ry2 = deltay * factor * (deltay < 0 ? 1.00001 : 1.0);
        }

        /// <summary>
        /// Flags whether 
        /// </summary>
        /// <param name="mi">Cell being inspected</param>
        /// <param name="xT">Current robot pose</param>
        /// <param name="zT">Current measurement</param>
        /// <param name="indexToClosestBeam">Index of the sensor's beam closest (anglar distance) to the cell mi</param>
        /// <param name="angleToClosestBeam">Angle to the cell mi from the closest sensor beam</param>
        /// <param name="distanceToCenterOfMass">Distance from the robot to the center of mass of the cell mi</param>
        /// <returns>true if the center of mass of the cell mi is in the perceptual field of the sensor's beam</returns>
        public bool InPerceptualField(GridCell mi, Pose xT, UltraSonicMeasurement zT, out int indexToClosestBeam, out double angleToClosestBeam, out double distanceToCenterOfMass)
        {
            distanceToCenterOfMass = zT.FindRangeAndAngleToClosestBeam(mi, xT, out angleToClosestBeam, out indexToClosestBeam);

            // Is it in range and in the field of view?
            return distanceToCenterOfMass < maxValue && angleToClosestBeam < this.beta / 2.0;
        }
    }

    internal static class VectorExtentions
    {
        public static double[] Prod(this IEnumerable<double> x, double[] y)
        {
            return x.Zip(y, (a, b) => a * b).ToArray();
        }

        public static double[] Pow(this IEnumerable<double> x, double e)
        {
            return x.Select(xi => Math.Pow(xi, e)).ToArray();
        }

        public static double[] Square(this IEnumerable<double> x)
        {
            return x.Select(xi => Math.Pow(xi, 2.0)).ToArray();
        }

        public static double[] Diff(this IEnumerable<double> x, double[] y)
        {
            return x.Zip(y, (a, b) => a - b).ToArray();
        }
    }
}
