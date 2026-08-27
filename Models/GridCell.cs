using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    /// <summary>
    /// A cel in a grid (accupancy map)
    /// </summary>
    public class GridCell : ICloneable
    {
        /// <summary>
        /// X coordinate of the center of gravity
        /// </summary>
        public double xi;

        /// <summary>
        /// Y coordinate of the center of gravity
        /// </summary>
        public double yi;

        /// <summary>
        /// Cell size in distance units (usually cm)
        /// </summary>
        public double cellSize;

        /// <summary>
        /// Gets or sets the prior occupancy probability as a log odds value
        /// </summary>
        public double PriorOccupacyPob { get; set; }

        /// <summary>
        /// <para>Gets or sets the occupancy interpreting the value as a log odds value.</para>
        /// </summary>
        public double OccupancyLogOdds { get; set; }

        /// <summary>
        /// Convert fro probability to log odds
        /// </summary>
        /// <param name="prob">The probability value</param>
        /// <returns>The log odds value corresponding to give probability</returns>
        public static double ConvertProbToLogOdds(double prob)
        {
            if (prob < 0 || prob > 1)
            {
                throw new ArgumentOutOfRangeException("Probability must be in the range [0..1]");
            }

            return Math.Log(prob / (1 - prob));
        }

        /// <summary>
        /// Convert from log odds to probability
        /// </summary>
        /// <param name="logOdds">Log odds value</param>
        /// <returns>The probability value conrrespoding to the given log odds</returns>
        public static double ConvertLogOddsToProb(double logOdds)
        {
            double exp = Math.Exp(logOdds);
            return exp / (1 + exp);
        }

        public override string ToString()
        {
            return string.Format("x:{0:f2} cm, y:{1:f2} cm, locc:{2:f2}, prioLocc:{3:f2}", this.xi, this.yi, this.OccupancyLogOdds, this.PriorOccupacyPob);
        }

        public object Clone()
        {
            GridCell temp = new GridCell
            {
                xi = this.xi,
                yi = this.yi,
                cellSize = this.cellSize,
                OccupancyLogOdds = this.OccupancyLogOdds,
                PriorOccupacyPob = this.PriorOccupacyPob
            };

            return temp;
        }
    }
}
