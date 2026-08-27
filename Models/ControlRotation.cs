using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public class ControlRotation
    {
        public double v;
        public double w;

        public override string ToString()
        {
            return string.Format("v:{0:f1} cm/s, w:{1:f4} deg", v, w * 180 / Math.PI);
        }

        public string ToString(bool extended = true)
        {
            return string.Format("v:{0} cm/s, w:{1} deg", v, w * 180 / Math.PI);
        }
    }
}
