using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public class MotionVelocity
    {
        // Translational noise
        public double alpha1;
        public double alpha2;

        // Angular noise
        public double alpha3;
        public double alpha4;

        // Other noise
        public double alpha5;
        public double alpha6;

        public Func<double, double> sampler;

        public double dt;

        public Pose SampleModel(ControlRotation uT, Pose xT_1)
        {
            double v2 = uT.v * uT.v;
            double w2 = uT.w * uT.w;
            double vBar = uT.v + sampler(this.alpha1 * v2 + this.alpha2 * w2);
            double wBar = uT.w + sampler(this.alpha3 * v2 + this.alpha4 * w2);
            double gBar = sampler(this.alpha5 * v2 + this.alpha6 * w2);

            double xPrime, yPrime, thetaPrime;

            if (Math.Abs(wBar) < 1e-6)
            {
                // Straight-line limit of the circular-arc formula as w → 0.
                // Avoids division by zero when the robot moves in (nearly) a straight line.
                xPrime     = xT_1.x + vBar * dt * Math.Cos(xT_1.theta);
                yPrime     = xT_1.y + vBar * dt * Math.Sin(xT_1.theta);
                thetaPrime = xT_1.theta + gBar * dt;
            }
            else
            {
                xPrime     = xT_1.x - vBar / wBar * (Math.Sin(xT_1.theta) - Math.Sin(xT_1.theta + wBar * dt));
                yPrime     = xT_1.y + vBar / wBar * (Math.Cos(xT_1.theta) - Math.Cos(xT_1.theta + wBar * dt));
                thetaPrime = xT_1.theta + (wBar + gBar) * dt;
            }

            // Return x_t (the sampled belief)
            return new Pose(xPrime, yPrime, thetaPrime);
        }
    }
}
