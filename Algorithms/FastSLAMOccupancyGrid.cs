using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Models;

namespace Algorithms
{
    public class FastSLAMOccupancyGrid
    {
        public Models.MotionVelocity motionVelocityModel;
        public BeamRangeFinderModel measurementModel;

        public List<BeliefWeightPair> Iterate(List<BeliefeOccupancyGrid> ChiT_1, ControlRotation uT, UltraSonicMeasurement zT)
        {
            List<BeliefWeightPair> chiTBar = new List<BeliefWeightPair>();
                 
            foreach(var belief in ChiT_1)
            {
                // Sample motion model
                var xT = this.motionVelocityModel.SampleModel(uT, belief.pose);

                // Sample the measurement model against the CURRENT (pre-update) map
                double wT = this.measurementModel.BeamRangeFinder(zT, xT, belief.map);

                // Clone the map before updating so each particle has an independent copy.
                // Without cloning, particles drawn multiple times during resampling share the
                // same OccupancyGridMap object: subsequent iterations update it repeatedly and
                // all copies see the same mutations, destroying particle diversity.
                OccupancyGridMap map = (OccupancyGridMap)((OccupancyGridMap)belief.map).Clone();
                map.UpdateMap(xT, zT, sensor: this.measurementModel);

                BeliefWeightPair beliefWeightPair = new BeliefWeightPair
                {
                    grid = new BeliefeOccupancyGrid(pose: xT, map: map, path: belief.path),
                    weight = wT,
                };

                chiTBar.Add(beliefWeightPair);
            }

            return SampleProbDistribution(totalSamples: ChiT_1.Count, chiTBar: chiTBar);
        }

        public class BeliefWeightPair
        {
            public BeliefeOccupancyGrid grid;
            public double weight;

            public override string ToString()
            {
                if (weight < 0.001)
                {
                    return string.Format("Belief:{0}, w:{1:g5}", grid, weight);
                }
                else
                {
                    return string.Format("Belief:{0}, w:{1:f3}", grid, weight);
                }
            }
        }

        private List<BeliefWeightPair> SampleProbDistribution(int totalSamples, List<BeliefWeightPair> chiTBar)
        {
            List<BeliefWeightPair> chiT = new List<BeliefWeightPair>();

            if (chiTBar.Count == 0)
            {
                return chiT;
            }

            // Compute the limits
            double curSum = 0;
            double[] limits = chiTBar.Select(e => curSum += e.weight).ToArray();

            // Obtain the total of the weights
            double total = limits[limits.Length - 1];

            // draw i with probability proportional to wt(i) from chi bar t
            Random rnd = new Random();
            while (totalSamples > chiT.Count)
            {
                double rand = rnd.NextDouble() * total;

                if (rand == 0.0)
                {
                    continue;
                }
                
                // Search for limit with that value or for info about corresponding position
                int i = Array.BinarySearch(limits, rand);

                if (i < 0)
                {   // Not found, but got information about immediate neighbor
                    // It cannot happen that |i| > limits.length - 1 
                    i = -i;
                }

                // Add the particle
                chiT.Add(chiTBar[i - 1]);
            }

            return chiT;
        }

        public void Initialize()
        {
            this.measurementModel = new BeamRangeFinderModel(beta: 30, alpha: 10, zHit: 0.5, zShort: 0.5, zMax: 0.5, zRand: 0.5, sigmaHit: 5.0, lambdaShort: 0.5);
        }

        public override string ToString()
        {
            StringBuilder str = new StringBuilder();

            str.AppendLine(this.measurementModel.ToString());
            str.AppendLine(this.motionVelocityModel.ToString());

            return str.ToString();
        }
    }
}
