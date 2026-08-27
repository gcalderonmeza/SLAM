using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public class OccupancyGridMap : MapBase, ICloneable
    {
        public GridCell[] m;
        public int xCells;
        public int yCells;

        public double CellSize
        {
            get
            {
                if (this.m == null || this.xCells == 0 || this.yCells == 0)
                {
                    return -1;
                }

                return this.m[0].cellSize;
            }
        }

        private OccupancyGridMap()
        {
        }

        public OccupancyGridMap(double cellSize, int xCells, int yCells)
        {
            int cells = xCells * yCells;
            this.m = new GridCell[cells];

            this.xCells = xCells;
            this.yCells = yCells;

            for (int x = 0; x < xCells; x++)
            {
                for (int y = 0; y < yCells; y++)
                {
                    int i = x + y * xCells;
                    this.m[i] = new GridCell
                    {
                        xi = cellSize * x + cellSize / 2.0,
                        yi = cellSize * y + cellSize / 2.0,

                        // Since it is unknown if the cell is occupied or not, initialize to p - 0.5
                        PriorOccupacyPob = GridCell.ConvertProbToLogOdds(0.5),
                        OccupancyLogOdds = GridCell.ConvertProbToLogOdds(0.5),
                        cellSize = cellSize,
                    };
                }
            }
        }

        /// <summary>
        /// <para>Updates the OccupancyGridMap.</para>
        /// <para>This method assumes that the occupancy values are expressed in "log odds"</para>
        /// </summary>
        /// <param name="xT"></param>
        /// <param name="zT"></param>
        /// <param name="sensor"></param>
        /// <returns></returns>
        public OccupancyGridMap UpdateMap(Pose xT, UltraSonicMeasurement zT, BeamRangeFinderModel sensor)
        {
            //OccupancyGridMap lt = (OccupancyGridMap)this.Clone();

            //for (var i = 0; i < this.m.Length;i++ )
            foreach(var mi in this.m)
            {
                //GridCell mi = this.m[i];

                int indexToClosestBeam;
                double angleToClosestBeam;
                double distanceToCenterOfMass;
                if (sensor.InPerceptualField(mi, xT, zT, out indexToClosestBeam, out angleToClosestBeam, out distanceToCenterOfMass))
                {
                    double inverseSensorModel = sensor.InverseRangeSensorModel(mi, xT, zT, indexToClosestBeam, angleToClosestBeam, distanceToCenterOfMass);

                    // This is correct only if the occupancy probs are expressed in log odds
                    //lt.m[i].OccupancyLogOdds = mi.OccupancyLogOdds + inverseSensorModel - mi.PriorOccupacyPob;
                    mi.OccupancyLogOdds += inverseSensorModel - mi.PriorOccupacyPob;
                }
            }

            //return lt;
            return this;
        }

        public Tuple<int, int> GetCellFromPose(Pose xT)
        {
            int x = (int)Math.Floor(xT.x / this.CellSize);
            int y = (int)Math.Floor(xT.y / this.CellSize);

            return new Tuple<int, int>(x, y);
        }

        /**
        private static bool InPerceptualField(GridCell mi, Pose xT, BeamRangeFinderModel sensor, UltraSonicMeasurement zT)
        {
            int indexToClosestBeam;
            double min;
            double r = zT.FindRangeAndAngleToClosestBeam(mi, xT, out min, out indexToClosestBeam);

            // Is it in range and in the field of view?
            return r < sensor.zMax && min < sensor.beta / 2.0;
        } **/

        public object Clone()
        {
            OccupancyGridMap temp = new OccupancyGridMap(this.CellSize, this.xCells, this.yCells);
            temp.m = new GridCell[this.m.Length];
            for(int i=0; i < temp.m.Length; i++)
            {
                temp.m[i] = (GridCell)this.m[i].Clone();
            }

            return temp;
        }
    }
}
