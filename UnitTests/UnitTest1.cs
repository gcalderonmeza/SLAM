using System;
using System.Drawing;
using System.IO;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using Models;
using Algorithms;

namespace UnitTests
{
    [TestClass]
    public class UnitTest1
    {
        [TestMethod]
        public void SampleNormalDistribution_Test()
        {
            using (StreamWriter outfile = new StreamWriter(@"C:\temp\TestingRobotics\SampleNormalDistribution_Test.txt"))
            {
                double variance = 3.5;
                outfile.WriteLine("Variance:" + variance);
                outfile.WriteLine("y");

                for (int i = 0; i < 500; i++)
                {
                    outfile.WriteLine(Distributions.SampleNormal(variance));
                }
            }
        }

        [TestMethod]
        public void SampleTriangularDistribution_Test()
        {
            using (StreamWriter outfile = new StreamWriter(@"C:\temp\TestingRobotics\SampleTriangularDistribution_Test.txt"))
            {
                double variance = 3.5;
                outfile.WriteLine("Variance:" + variance);
                outfile.WriteLine("y");

                for (int i = 0; i < 500; i++)
                {
                    outfile.WriteLine(Distributions.SampleTriangular(variance));
                }
            }
        }

        [TestMethod]
        public void SampleMotionModelVelocity_Test()
        {
            MotionVelocity model = new MotionVelocity
            {
                alpha1 = 0.005,
                alpha2 = 0.005,
                alpha3 = 0.08 * Math.PI / 180,
                alpha4 = 0.08 * Math.PI / 180,
                alpha5 = 0.0000,
                alpha6 = 0.0000,
                sampler = Distributions.SampleNormal,
            };

            model.dt = 9.0;

            Pose pose = new Pose(0, 0, 0);
            ControlRotation control = new ControlRotation
            {
                v = 1.0,
                w = 5.0 * Math.PI / 180.0,
            };

            List<Pose> poses = new List<Pose>();
            for (int i = 0; i < 500; i++)
            {
                poses.Add(model.SampleModel(control, pose));
            }

            if (!Directory.Exists(@"C:\temp\TestingRobotics"))
            {
                Directory.CreateDirectory(@"C:\temp\TestingRobotics");
            }

            using (StreamWriter outfile = new StreamWriter(@"C:\temp\TestingRobotics\SampleMotionModelVelocity_Test.txt"))
            {
                outfile.WriteLine("x, y, theta");
                outfile.WriteLine(string.Format("{0},{1},{2}", pose.x, pose.y, pose.theta));

                for(int i=0; i< 500; i++)
                {
                    outfile.WriteLine(string.Format("{0},{1},{2}", poses[i].x, poses[i].y, poses[i].theta));
                }
            }
        }

        [TestMethod]
        public void SampleMotionModelOdometry_Test()
        {
            MotionOdometry model = new MotionOdometry
            {
                alpha1 = 0.005,
                alpha2 = 0.005,
                alpha3 = 0.08 * Math.PI / 180,
                alpha4 = 0.08 * Math.PI / 180,
                sampler = Distributions.SampleNormal,
            };

            Pose pose = new Pose(0, 0, 0);
            ControlOdometry control = new ControlOdometry
            {
                xBarT = new Pose(0, 0, 0),
                xBarT_1 = new Pose(8.1, 3.36, 45 * Math.PI / 180),
            };

            List<Pose> poses = new List<Pose>();
            for (int i = 0; i < 500; i++)
            {
                poses.Add(model.SampleModel(control, pose));
            }

            if (!Directory.Exists(@"C:\temp\TestingRobotics"))
            {
                Directory.CreateDirectory(@"C:\temp\TestingRobotics");
            }

            using (StreamWriter outfile = new StreamWriter(@"C:\temp\TestingRobotics\SampleMotionModelOdometry_Test.txt"))
            {
                outfile.WriteLine("x, y, theta");
                outfile.WriteLine(string.Format("{0},{1},{2}", pose.x, pose.y, pose.theta));

                for (int i = 0; i < 500; i++)
                {
                    outfile.WriteLine(string.Format("{0},{1},{2}", poses[i].x, poses[i].y, poses[i].theta));
                }
            }
        }

        private void FastSLAM_Algorithm()
        {
            FastSLAMModel model = new FastSLAMModel(numBeliefs: 1000, xCells: 100, yCells: 100, cellSize: 1);
            ControlRotation control = new ControlRotation
                {
                    v = 1,
                    w = 0.0001
                };
            UltraSonicMeasurement measurement = new UltraSonicMeasurement
                {
                    theta = new double[] { Math.PI / 2, -Math.PI /2 },
                    z = new double[] { 49, 29 }
                };

            RobotWorldForm form = new RobotWorldForm(control: control, measurement: measurement);

            form.model = model;
            form.Update();

            Random rnd = new Random();

            int iterations = 100;
            do
            {
                measurement.z[0] = measurement.z[0] + rnd.Next(2);
                measurement.z[1] = measurement.z[1] + rnd.Next(2);
                model.Iterate(control: control, measurement: measurement);
                form.Update();
            }
            while (--iterations > 0);

            ////using (StreamWriter outfile = new StreamWriter(@"C:\temp\TestingRobotics\SampleNormalDistribution_Test.txt"))
            ////{
            ////    double variance = 3.5;
            ////    outfile.WriteLine("Variance:" + variance);
            ////    outfile.WriteLine("y");

            ////    for (int i = 0; i < 500; i++)
            ////    {
            ////        outfile.WriteLine(Distributions.SampleNormal(variance));
            ////    }
            ////}
        }

        [TestMethod]
        public void FastSLAM_Test()
        {
            ControlRotation control = new ControlRotation
            {
                v = 1,
                w = 0.0001
            };
            UltraSonicMeasurement measurement = new UltraSonicMeasurement
            {
                theta = new double[] { Math.PI / 2, -Math.PI / 2 },
                z = new double[] { 49, 29 }
            };

            RobotWorldForm form = new RobotWorldForm(control: control, measurement: measurement);
            form.model = new FastSLAMModel(numBeliefs: 1000, xCells: 100, yCells: 100, cellSize: 1);

            form.SetDesktopBounds(10, 10, 520, 520);
            form.ShowDialog();

            // this.FastSLAM_Algorithm();
        }

        [TestMethod]
        public void Learn_Parameters_Test()
        {
            BeamRangeFinderModel beamRangeFinder = new BeamRangeFinderModel(beta: 0.1, alpha: 0.1, zHit: 22.0, zShort: 5.0, zMax: 255.0, zRand: 0.1, sigmaHit: 2.0, lambdaShort: 0.5);
            OccupancyGridMap map = new OccupancyGridMap(cellSize: 10, xCells: 10, yCells: 10);
            double occupiedLogOddsThreshold = GridCell.ConvertProbToLogOdds(0.8);
            for (int i = 0; i < 8; i++)
            {
                map.m[6 + i * 10].OccupancyLogOdds = occupiedLogOddsThreshold;
            }

            double[] angles = new double[] { -90 * Math.PI / 180, 0, 90 * Math.PI / 180 };
            string[] lines = System.IO.File.ReadAllLines(@"C:\temp\Robotics\UnitTests\sample_sonar_1.csv");
            string line;

            List<Pose> poses = new List<Pose>();
            List<UltraSonicMeasurement> measurements = new List<UltraSonicMeasurement>();
            for (int i = 0; i < lines.Length; i++)
            {
                line = lines[i];
                if (line != null)
                {
                    string[] columns = line.Split(',');

                    poses.Add(new Pose(x: double.Parse(columns[0]), y: double.Parse(columns[1]), theta: double.Parse(columns[2])));

                    // Assume three sensors at -90, 0, 90
                    // TODO: how to simulate actual measurements while the robot moves
                    measurements.Add(new UltraSonicMeasurement
                        {
                            theta = angles,

                            // TODO: randomize this or use the input file
                            // z = new double[] { int.Parse(columns[3]), int.Parse(columns[4]), int.Parse(columns[5]) },
                            z = new double[] { 255, int.Parse(columns[4]), 255 },
                        });
                }
            }

            beamRangeFinder.Learn_intrinsic_parameters(measurements.ToArray(), poses.ToArray(), map);

            ////using (StreamWriter outfile = new StreamWriter(@"C:\temp\TestingRobotics\SampleNormalDistribution_Test.txt"))
            ////{
            ////    double variance = 3.5;
            ////    outfile.WriteLine("Variance:" + variance);
            ////    outfile.WriteLine("y");

            ////    for (int i = 0; i < 500; i++)
            ////    {
            ////        outfile.WriteLine(Distributions.SampleNormal(variance));
            ////    }
            ////}
        }
    }
}
