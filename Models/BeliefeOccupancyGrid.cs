using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Models;

namespace Models
{
    public class BeliefeOccupancyGrid
    {
        public Pose pose {get; private set; }
        public MapBase map {get; private set; }
        public List<Pose> path {get; private set; }

        public BeliefeOccupancyGrid(Pose pose, MapBase map, List<Pose> path)
        {
            this.pose = pose;
            this.map = map;

            this.path = new List<Pose>();
            if (path != null)
            {
                this.path.AddRange(path);
            }
            
            this.path.Add(pose);
        }

        public override string ToString()
        {
            StringBuilder str = new StringBuilder();

            str.Append("Pose: ");
            str.AppendLine(this.pose.ToString());

            str.Append("Map: ");
            str.AppendLine(this.map.ToString());

            return str.ToString();
        }
    }
}
