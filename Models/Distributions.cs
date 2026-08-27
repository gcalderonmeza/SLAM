using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Models
{
    public static class Distributions
    {
        const double triangularFactor = 2.4494897427831780981972840747059 / 2.0; // Math.Sqrt(6) / 2.0
        private static readonly Random rnd = new Random();

        /// <summary>
        /// Generates zero mean, uniformly distributed random numbers in the range [-min max]
        /// </summary>
        /// <param name="min">The minimum value generable</param>
        /// <param name="max">The maximum value generable</param>
        /// <returns>One sample of zero mean, uniformly distributed random number in the range [-min max]</returns>
        public static double Rand(double min, double max)
        {
            var range = max - min;

            return (rnd.NextDouble() * range) + min;
        }

        public static double SampleNormal(double variance)
        {
            double b = Math.Sqrt(variance);
            double sum = 0;
            for (int i = 0; i < 12; i++)
            {
                sum += Rand(-b, b);
            }

            return 0.5 * sum;
        }

        public static double SampleTriangular(double variance)
        {
            double b = Math.Sqrt(variance);
            return triangularFactor * (Rand(-b, b) + Rand(-b, b));
        }

        public static double Normal(double z, double mean, double variance)
        {
            // TOOD: check for correct integral variance / (-(z - mean) * Math.Sqrt(2.0 * Math.PI * variance)) * Math.Exp(-0.5 * Math.Pow(z - mean, 2.0) / variance)
            return Math.Exp(-0.5 * Math.Pow(z - mean, 2.0) / variance) / Math.Sqrt(2.0 * Math.PI * variance); 
        }

        public static double IntegralNormal(double z, double mean, double variance)
        {
            // Disregarding integration constant
            double delta = z - mean;
            double result = -2.0 / Math.Sqrt(Math.PI) * Math.Exp(-(delta * delta) / (2.0 * variance));
            return result;
        }

        public static double DefiniteIntegralNormal(double mean, double variance, double lowerLimit, double upperLimit)
        {
            return IntegralNormal(z: upperLimit, mean: mean, variance: variance) - IntegralNormal(z: lowerLimit, mean: mean, variance: variance);
        }
    }
}
