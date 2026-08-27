using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public class Pose
    {
        public double x;
        public double y;
        public double theta;

        public Pose(double x, double y, double theta)
        {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        public override string ToString()
        {
            return string.Format("x:{0:f2} cm, y:{1:f2} cm, theta:{2:f2} deg", this.x, this.y, theta * 180 / Math.PI);
        }
    }
}
