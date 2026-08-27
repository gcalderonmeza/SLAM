using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Models;
using Algorithms;
using System.IO;

namespace UnitTests
{
    public class FastSLAMModel
    {
        public int numBeliefs;
        public int xCells;
        public int yCells;
        public double cellSize;

        public List<BeliefeOccupancyGrid> beliefs;
        public List<FastSLAMOccupancyGrid.BeliefWeightPair> weightedBliefs;
        public FastSLAMOccupancyGrid fastSLAM;

        public FastSLAMModel(int numBeliefs, int xCells, int yCells, double cellSize)
        {
            this.InitializeGridParams(numBeliefs, xCells, yCells, cellSize);

            this.PopulateBeliefsRandomly();
            this.PopulateOccupancyGrid();
        }

        public void InitializeGridParams(int numBeliefs, int xCells, int yCells, double cellSize)
        {
            this.numBeliefs = numBeliefs;
            this.xCells = xCells;
            this.yCells = yCells;
            this.cellSize = cellSize;
        }

        public void ResetModel()
        {
            this.PopulateBeliefsRandomly();
            this.PopulateOccupancyGrid();
        }

        public void PopulateBeliefsRandomly()
        {
            OccupancyGridMap map;
            Pose pose;

            double maxXPos = this.xCells * cellSize;
            double maxYPos = this.yCells * cellSize;

            this.beliefs = new List<BeliefeOccupancyGrid>();

            Random rnd = new Random();
            for (int i = 0; i < this.numBeliefs; i++)
            {
                map = new OccupancyGridMap(cellSize: this.cellSize, xCells: this.xCells, yCells: this.yCells);
                pose = new Pose(x: rnd.NextDouble() * maxXPos, y: rnd.NextDouble() * maxYPos, theta: rnd.NextDouble() * 360 * Math.PI / 180.0);

                // TODO: Thi is sharing the same map for all the believes
                BeliefeOccupancyGrid belief = new BeliefeOccupancyGrid(pose: pose, map: map, path: null);

                beliefs.Add(belief);
            }
        }

        public void PopulateOccupancyGrid()
        {
            this.fastSLAM = new FastSLAMOccupancyGrid
            {
                // Using this model for the motion, it could not be the best (0 error in 5 and 6)
                motionVelocityModel = new MotionVelocity
                {
                    alpha1 = 0.005,
                    alpha2 = 0.005,
                    alpha3 = 0.08 * Math.PI / 180,
                    alpha4 = 0.08 * Math.PI / 180,
                    alpha5 = 0.0000,
                    alpha6 = 0.0000,
                    sampler = Distributions.SampleNormal,
                    dt = 0.5,
                },

                // sigmaHit is the std-dev (cm) of the Gaussian that scores how well a sensor
                // reading matches the map-predicted distance. A value of 0.13 cm is far too tight
                // for a real ultrasonic sensor (HC-SR04 noise is ~1-3 cm): PHit becomes
                // effectively 0 for any realistic deviation, PRand dominates, all particles get
                // equal weights, and the filter cannot converge.  Use Learn_intrinsic_parameters()
                // to calibrate this from real sensor data; 5 cm is a safe starting default.
                measurementModel = new BeamRangeFinderModel(beta: 40, alpha: 10, zHit: 0.71, zShort: 0.08, zMax: 0.09, zRand: 0.12, sigmaHit: 5.0, lambdaShort: 0.5),
            };
        }

        public void Iterate(ControlRotation control, UltraSonicMeasurement measurement)
        {
            this.weightedBliefs = fastSLAM.Iterate(this.beliefs, control, measurement);

            // this.weightedBliefs.Sort(
            this.beliefs = this.weightedBliefs.Select(e => e.grid).ToList();
        }

        public override string ToString()
        {
            StringBuilder str = new StringBuilder();

            str.Append("Cell size: ");
            str.Append(cellSize);
            str.AppendLine();
            str.Append("xCells: ");
            str.Append(xCells);
            str.AppendLine();
            str.Append("yCells: ");
            str.Append(yCells);
            str.AppendLine();
            str.Append("Num beliefs: ");
            str.Append(numBeliefs);
            str.AppendLine();

            foreach(var belief in this.beliefs)
            {
                str.AppendLine(belief.ToString());
            }

            if (this.weightedBliefs == null)
            {
                str.AppendLine("No weighted beliefs");
            }
            else
            {
                foreach(var belief in this.weightedBliefs)
                {
                    str.AppendLine(belief.ToString());
                }
            }

            str.AppendLine(this.fastSLAM.ToString());

            return str.ToString();
        }
    }
}
