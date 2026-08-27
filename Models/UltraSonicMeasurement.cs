using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public class UltraSonicMeasurement
    {
        private int beams;

        public int Beams
        {
            get
            {
                if (theta == null)
                {
                    return 0;
                }
                else
                {
                    this.Beams = theta.Length;
                    return beams;
                };
            }

            private set
            {
                this.beams = value;
            }
        }

        // Value of the sensor (beam)
        public double[] z;

        // Angle of the sensor (beam)
        public double[] theta;

        public double FindRangeAndAngleToClosestBeam(GridCell mi, Pose xT, out double angleToClosestBeam, out int indexToClosest)
        {
            double deltaX = mi.xi - xT.x;
            double deltaY = mi.yi - xT.y;

            // Compute range from the robot's position to the center of mass of mi
            double r = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);

            // Compute angle to the center of mass in robot coordinates.
            // Atan2 returns a value in [-pi, pi]
            double phi = Math.Atan2(deltaY, deltaX) - xT.theta;

            // Figure out if any of the sensors can actually see the center of mass of mi from the current robot pose (xT)
            // Find the sensor that is closer to the center of mass (in the line of sight sense)
            angleToClosestBeam = double.MaxValue;
            indexToClosest = -1;

            // Normalization needed for the comparisson to make sense
            // Angle will be in [-pi, pi]
            double normalizedPhi = NormalizeAngle(phi);
            for (int k = 0;k < this.z.Length; k++)
            {
                double theta = this.theta[k];
                // Perhaps we need to normalize the difference too
                double diff = Math.Abs(NormalizeAngle(normalizedPhi - theta));
                if (angleToClosestBeam > diff)
                {
                    angleToClosestBeam = diff;
                    indexToClosest = k;
                }
            }

            return r;
        }

        /// <summary>
        /// Normalizes the angles to the range [-pi, pi]
        /// </summary>
        /// <param name="angleRad">Input angle in radians</param>
        /// <returns>Angle in radians normalized to the [pi, pi] range</returns>
        private static double NormalizeAngle(double angleRad)
        {
            double twoPi = Math.PI * 2;
            double sign = Math.Sign(angleRad);
            angleRad = Math.Abs(angleRad);
            if (angleRad > twoPi)
            {
                double ratioInt = Math.Truncate(angleRad / twoPi);
                angleRad -= ratioInt * twoPi;
            }

            if (sign < 0)
            {
                if (angleRad > Math.PI)
                {
                    return twoPi - angleRad;
                }
                else
                {
                    return -angleRad;
                }
            }
            else
            {
                if (angleRad > Math.PI)
                {
                    return angleRad - twoPi;
                }
                else
                {
                    return angleRad;
                }
            }
        }
    }
}
