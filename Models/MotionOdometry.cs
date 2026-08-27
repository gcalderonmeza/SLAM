using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public class MotionOdometry
    {
        // Translational noise
        public double alpha1;
        public double alpha2;

        // Angular noise
        public double alpha3;
        public double alpha4;

        public Func<double, double> sampler;

        public Pose SampleModel(ControlOdometry uT, Pose xT_1)
        {
            double deltaRot1 = Math.Atan2(uT.xBarT.y - uT.xBarT_1.y, uT.xBarT.x - uT.xBarT_1.x) - uT.xBarT_1.theta;
            double deltaTrans = Math.Sqrt(Math.Pow(uT.xBarT_1.x - uT.xBarT.x, 2) + Math.Pow(uT.xBarT_1.y - uT.xBarT.y, 2));
            double deltaRot2 = uT.xBarT.theta - uT.xBarT_1.theta - deltaRot1;

            double deltaBarRot1 = deltaRot1 - sampler(this.alpha1 * Math.Pow(deltaRot1, 2) + this.alpha2 * Math.Pow(deltaTrans, 2));
            double deltaBarTrans = deltaTrans - sampler(this.alpha3 * Math.Pow(deltaTrans, 2) + this.alpha4 * Math.Pow(deltaRot1, 2) + this.alpha4 * Math.Pow(deltaRot2, 2));
            double deltaBarRot2 = deltaRot2 - sampler(this.alpha1 * Math.Pow(deltaRot2, 2) + this.alpha2 * Math.Pow(deltaTrans, 2));

            double xPrime = xT_1.x + deltaBarTrans * Math.Cos(xT_1.theta + deltaBarRot1);
            double yPrime = xT_1.y + deltaBarTrans * Math.Sin(xT_1.theta + deltaBarRot1);
            double thetaPrime = xT_1.theta + deltaBarRot1 + deltaBarRot2;

            return new Pose(xPrime, yPrime, thetaPrime);
        }
    }
}
