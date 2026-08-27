using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Models;
using Algorithms;
using System.IO;

namespace UnitTests
{
    public partial class RobotWorldForm : Form
    {
        public FastSLAMModel model;

        private int indexSelected;

        private bool newIteration;

        // Control action variables
        private StreamReader inputData;
        private Stream fakeInputControlActions;

        private ControlRotation defaultControlAction;

        // Measurements variables
        private StreamReader measurementInputData;
        private Stream fakeMeasurementInputData;

        private UltraSonicMeasurement defaultMeasurement;

        public RobotWorldForm(ControlRotation control, UltraSonicMeasurement measurement)
        {
            InitializeComponent();

            this.particlesListBox.ClearSelected();
            this.indexSelected = this.particlesListBox.SelectedIndex;
            this.newIteration = true;

            this.fakeInputControlActions = new MemoryStream();
            this.inputData = new StreamReader(this.fakeInputControlActions);
            this.defaultControlAction = control;

            this.fakeMeasurementInputData = new MemoryStream();
            this.measurementInputData = new StreamReader(this.fakeMeasurementInputData);
            this.defaultMeasurement = measurement;
        }

        /*
        private static byte[] GetBytes(string str)
        {
            byte[] bytes = new byte[str.Length * sizeof(char)];
            System.Buffer.BlockCopy(str.ToCharArray(), 0, bytes, 0, bytes.Length);
            return bytes;
        }

        private static string GetString(byte[] bytes)
        {
            char[] chars = new char[bytes.Length / sizeof(char)];
            System.Buffer.BlockCopy(bytes, 0, chars, 0, bytes.Length);
            return new string(chars);
        }
         * */

        private ControlRotation PopNextControlAction()
        {
            ControlRotation control = new ControlRotation();

            if (this.read_from_file_checkBox.Checked && (this.inputData != null))
            {
                string line = this.inputData.ReadLine();

                string[] sections = line.Split(new[] { ':', ' ' });
                double v = double.Parse(sections[1]);
                double w = double.Parse(sections[4]);

                control.v = v;
                control.w = w;
            }
            else
            {
                if (!string.IsNullOrWhiteSpace(this.w_textBox.Text))
                {
                    control.w = double.Parse(this.w_textBox.Text);
                }

                if (!string.IsNullOrWhiteSpace(this.v_textBox.Text))
                {
                    control.v = double.Parse(this.v_textBox.Text);
                }
            }

            return control;
        }

        private UltraSonicMeasurement PopNextMeasurement()
        {
            UltraSonicMeasurement measurement = new UltraSonicMeasurement();
            if (this.read_measurement_from_file_checkBox.Checked && (this.measurementInputData != null))
            {
                string line = this.measurementInputData.ReadLine();

                string[] sections = line.Split(new[] { ':', ' ' });
                double v = double.Parse(sections[1]);
                double w = double.Parse(sections[4]);

                measurement.theta = null;
                measurement.z = null;
            }
            else
            {
                // Assume sensors at pi/2, -pi/2 rads in robot coordinates (clockwise is negative, 0 is the heading of the robot)
                // TODO: how to simulate actual measurements while the robot moves
                Random rnd = new Random();
                measurement.theta = new double[] { Math.PI / 2, -Math.PI / 2.0 };
                measurement.z = new double[2];
                if (!string.IsNullOrWhiteSpace(this.sensor1_textBox.Text))
                {
                    measurement.z[0] = double.Parse(this.sensor1_textBox.Text) + rnd.Next(2);
                }

                if (!string.IsNullOrWhiteSpace(this.sensor2_textBox.Text))
                {
                    measurement.z[1] = double.Parse(this.sensor2_textBox.Text) + rnd.Next(2);
                }
            }

            return measurement;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            var original = this.iterate_once_button.Text;
            this.iterate_once_button.Text = "Computing";
            this.iterate_once_button.Enabled = false;

            ControlRotation control = this.PopNextControlAction();
            UltraSonicMeasurement measurement = this.PopNextMeasurement();
            this.SaveState(control, measurement);
            this.model.Iterate(control, measurement);
            this.newIteration = true;

            this.Invalidate();
            this.Update();

            this.iterate_once_button.Enabled = true;
            this.iterate_once_button.Text = original;
        }

        private void SaveState(ControlRotation control, UltraSonicMeasurement measurement)
        {
            if (this.save_state_checkBox.Checked)
            {
                string state_filename_templ = "state_{0}.csv";
                string state_filename;
                string[] files = Directory.GetFiles(".", string.Format(state_filename_templ, "*"), SearchOption.TopDirectoryOnly);

                if (files == null || files.Length == 0)
                {
                    state_filename = string.Format(state_filename_templ, "0");
                }
                else
                {
                    int index = 1;
                    state_filename = string.Format(state_filename_templ, index);
                }

                using (StreamWriter outfile = new StreamWriter(state_filename))
                {
                    outfile.WriteLine("ActionControl: " + control.ToString());
                    outfile.WriteLine("Measurement: " + measurement.ToString());
                    outfile.WriteLine();

                    outfile.WriteLine("Model: ");
                    outfile.WriteLine(this.model.ToString());
                }
            }
        }

        private void DrawPath(System.Drawing.Graphics graphicsObj, int factorX, int factorY, BeliefeOccupancyGrid belief)
        {
            Pen myPenBlack = new Pen(System.Drawing.Color.Black);

            Point point = this.ConvertToPoint(x: belief.path[0].x, y: belief.path[0].y, factorX: factorX, factorY: factorY);

            for(int i=1;i < belief.path.Count; i++)
            {
                Point next = this.ConvertToPoint(x: belief.path[i].x, y: belief.path[i].y, factorX: factorX, factorY: factorY);
                graphicsObj.DrawLine(myPenBlack, point.X, point.Y, next.X, next.Y);
                point = next;
            }
        }

        private void DrawMap(System.Drawing.Graphics graphicsObj, int factorX, int factorY)
        {
            if (this.indexSelected != -1)
            {
                // SolidBrush myGrayBrush = new SolidBrush(System.Drawing.Color.Gray);
                SolidBrush myWhiteBrush = new SolidBrush(System.Drawing.Color.White);
                SolidBrush myBlackBrush = new SolidBrush(System.Drawing.Color.Black);

                foreach (var belief in model.beliefs)
                {
                    int centerX = (int)(belief.pose.x * factorX);
                    int centerY = (int)(500 - belief.pose.y * factorY);
                    double cosFactor = Math.Cos(belief.pose.theta);
                    double sinFactor = Math.Sin(belief.pose.theta);

                    if (belief == this.model.beliefs[this.indexSelected] && this.drawMap_checkBox.Checked)
                    {
                        OccupancyGridMap myMap = belief.map as OccupancyGridMap;
                        foreach (var gridCell in myMap.m)
                        {
                            Brush brush;
                            if (gridCell.OccupancyLogOdds == 0)
                            {
                                continue;
                            }
                            else if (gridCell.OccupancyLogOdds < 0)
                            {
                                brush = myWhiteBrush;
                            }
                            else
                            {
                                brush = myBlackBrush;
                            }

                            graphicsObj.FillRectangle(brush, (int)((gridCell.xi - gridCell.cellSize / 2) * factorX), (int)(500 - (gridCell.yi + gridCell.cellSize / 2) * factorY), (int)(gridCell.cellSize * factorX), (int)(gridCell.cellSize * factorY));
                        }

                        this.DrawPath(graphicsObj, factorX, factorY, belief);
                    }
                }
            }
        }

        private Point ConvertToPoint(double x, double y, int factorX, int factorY)
        {
            int n_x = (int)(x * factorX);
            int n_y = (int)(500 - y * factorY);

            return new Point(x: n_x, y: n_y);
        }

        private void RobotWorldForm_Paint(object sender, PaintEventArgs e)
        {
            var bm = new Bitmap(502, 502);
            System.Drawing.Graphics graphicsObj = Graphics.FromImage(bm);

            Pen myPenBlack = new Pen(System.Drawing.Color.Black);
            Pen myPenGreen = new Pen(System.Drawing.Color.Green, 1);
            Pen myGrayPen = new Pen(System.Drawing.Color.Gray, 1);
            Pen myWhitePen = new Pen(System.Drawing.Color.White, 1);
            Pen myRedPen = new Pen(System.Drawing.Color.Red);

            Rectangle myrect = new Rectangle(1, 1, 500, 500);
            graphicsObj.DrawRectangle(myPenBlack, myrect);

            int factorX = 5;
            int factorY = 5;

            if (this.newIteration)
            {
                this.particlesListBox.ClearSelected();
                this.particlesListBox.Items.Clear();
                this.indexSelected = -1;
            }

            if (this.drawMap_checkBox.Checked)
            {
                this.DrawMap(graphicsObj, factorX, factorY);
            }

            for (int i = 1; i < 10; i++)
            {
                graphicsObj.DrawLine(myGrayPen, i * 10 * factorX, 2, i * 10 * factorX, 499);
                graphicsObj.DrawLine(myGrayPen, 2, i * 10 * factorY, 499, i * 10 * factorY);
            }

            foreach (var belief in model.beliefs)
            {
                int centerX = (int)(belief.pose.x * factorX);
                int centerY = (int)(500 - belief.pose.y * factorY);
                double cosFactor = Math.Cos(belief.pose.theta);
                double sinFactor = Math.Sin(belief.pose.theta);

                if (this.indexSelected != -1 && belief == this.model.beliefs[this.indexSelected])
                {
                    graphicsObj.DrawEllipse(myPenGreen, centerX - 4, centerY - 4, 6, 6);
                    graphicsObj.DrawLine(myPenGreen, centerX, centerY, (int)(centerX + 10 * cosFactor), (int)(centerY - 10 * sinFactor));
                }
                else
                {
                    graphicsObj.DrawEllipse(myRedPen, centerX - 2, centerY - 2, 3, 3);
                    graphicsObj.DrawLine(myRedPen, centerX, centerY, (int)(centerX + 5 * cosFactor), (int)(centerY - 5 * sinFactor));
                }

                if (this.newIteration)
                {
                    this.particlesListBox.Items.Add(belief.pose.ToString());
                }
            }

            this.pictureBox1.Image = bm;

            this.numParticlesBox.Text = string.Format("{0}", this.model.beliefs.Count);

            if (string.IsNullOrWhiteSpace(this.v_textBox.Text))
            {
                this.v_textBox.Text = string.Format("{0:f1}", this.defaultControlAction.v);
            }

            if (string.IsNullOrWhiteSpace(this.w_textBox.Text))
            {
                this.w_textBox.Text = string.Format("{0:f4}", this.defaultControlAction.w);
            }

            if (string.IsNullOrWhiteSpace(this.sensor1_textBox.Text))
            {
                this.sensor1_textBox.Text = string.Format("{0:f1}", this.defaultMeasurement.z[0]);
            }

            if (string.IsNullOrWhiteSpace(this.sensor2_textBox.Text))
            {
                this.sensor2_textBox.Text = string.Format("{0:f1}", this.defaultMeasurement.z[1]);
            }

            this.cellsX_textBox.Text = string.Format("{0}", this.model.xCells);
            this.cellsY_textBox.Text = string.Format("{0}", this.model.yCells);
            this.cellSize_textBox.Text = string.Format("{0:f1}", this.model.cellSize);
            this.numParticlesBox.Text = string.Format("{0}", this.model.numBeliefs);
        }

        private void particlesListBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (this.particlesListBox.SelectedIndex != -1)
            {
                this.newIteration = false;
                this.indexSelected = this.particlesListBox.SelectedIndex;
                this.Invalidate();
                // this.pictureBox1.Invalidate();
                this.Update();
            }
        }

        private void drawMap_checkBox_CheckedChanged(object sender, EventArgs e)
        {
            this.Invalidate();
            this.Update();
        }

        private void cellsX_textBox_TextChanged(object sender, EventArgs e)
        {
            this.apply_button.Enabled = true;
            this.reset_values_button.Enabled = true;
        }

        private void cellsY_textBox_TextChanged(object sender, EventArgs e)
        {
            this.apply_button.Enabled = true;
            this.reset_values_button.Enabled = true;
        }

        private void cellSize_textBox_TextChanged(object sender, EventArgs e)
        {
            this.apply_button.Enabled = true;
            this.reset_values_button.Enabled = true;
        }

        private void numParticlesBox_TextChanged(object sender, EventArgs e)
        {
            this.apply_button.Enabled = true;
            this.reset_values_button.Enabled = true;
        }

        private void apply_button_Click(object sender, EventArgs e)
        {
            if (!string.IsNullOrWhiteSpace(this.cellsX_textBox.Text))
            {
                // TODO: If there is a change here we should restart
                this.model.xCells = int.Parse(this.cellsX_textBox.Text);
            }

            if (!string.IsNullOrWhiteSpace(this.cellsY_textBox.Text))
            {
                // TODO: If there is a change here we should restart
                this.model.yCells = int.Parse(this.cellsY_textBox.Text);
            }

            if (!string.IsNullOrWhiteSpace(this.cellSize_textBox.Text))
            {
                // TODO: If there is a change here we should restart
                this.model.cellSize = double.Parse(this.cellSize_textBox.Text);
            }

            if (!string.IsNullOrWhiteSpace(this.numParticlesBox.Text))
            {
                // TODO: If there is a change here we should restart
                this.model.numBeliefs = int.Parse(this.numParticlesBox.Text);
            }

            this.model.ResetModel();

            this.apply_button.Enabled = false;
            this.reset_values_button.Enabled = false;

            this.Invalidate();
            // this.pictureBox1.Invalidate();
            this.Update();
        }

        private void reset_values_button_Click(object sender, EventArgs e)
        {
            this.cellsX_textBox.Text = this.model.xCells.ToString();
            this.cellsY_textBox.Text = this.model.yCells.ToString();
            this.cellSize_textBox.Text = this.model.cellSize.ToString();
            this.numParticlesBox.Text = this.model.numBeliefs.ToString();

            this.apply_button.Enabled = false;
            this.reset_values_button.Enabled = false;

            this.Invalidate();
            // this.pictureBox1.Invalidate();
            this.Update();
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            var result = this.openFileDialog1.ShowDialog();
            if (result == System.Windows.Forms.DialogResult.OK)
            {
                this.filename_textBox.Text = this.openFileDialog1.FileName;

                this.Invalidate();
                this.Update();
            }
        }

        private void read_from_file_checkBox_CheckedChanged(object sender, EventArgs e)
        {
            this.filename_textBox.Enabled = this.read_from_file_checkBox.Checked;
            this.browse_button.Enabled = this.read_from_file_checkBox.Checked;

            this.v_textBox.Enabled = !this.filename_textBox.Enabled;
            this.w_textBox.Enabled = !this.filename_textBox.Enabled;

            this.Invalidate();
            this.Update();
        }

        private void filename_textBox_TextChanged(object sender, EventArgs e)
        {
            if (!string.IsNullOrWhiteSpace(this.filename_textBox.Text))
            {
                try
                {
                    if (this.inputData != null)
                    {
                        this.inputData.Close();
                    }

                    this.inputData = new StreamReader(this.filename_textBox.Text);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(text: "Could not read the file." + ex, caption: "Error reading the file", buttons: MessageBoxButtons.OK, icon: MessageBoxIcon.Error);
                }
            }

            this.Invalidate();
            this.Update();
        }

        private void read_measurement_from_file_checkBox_CheckedChanged(object sender, EventArgs e)
        {
            this.measurements_filename_textBox.Enabled = this.read_measurement_from_file_checkBox.Checked;
            this.browse_measurement_button.Enabled = this.read_measurement_from_file_checkBox.Checked;

            this.sensor1_textBox.Enabled = !this.measurements_filename_textBox.Enabled;
            this.sensor2_textBox.Enabled = !this.measurements_filename_textBox.Enabled;

            this.Invalidate();
            this.Update();
        }

        private void browse_measurement_button_Click(object sender, EventArgs e)
        {
            var result = this.openFileDialog1.ShowDialog();
            if (result == System.Windows.Forms.DialogResult.OK)
            {
                this.measurements_filename_textBox.Text = this.openFileDialog1.FileName;

                this.Invalidate();
                this.Update();
            }
        }

        private void measurements_filename_textBox_TextChanged(object sender, EventArgs e)
        {
            if (!string.IsNullOrWhiteSpace(this.measurements_filename_textBox.Text))
            {
                try
                {
                    if (this.measurementInputData != null)
                    {
                        this.measurementInputData.Close();
                    }

                    this.measurementInputData = new StreamReader(this.measurements_filename_textBox.Text);
                }
                catch (Exception ex)
                {
                    MessageBox.Show(text: "Could not read the file. " + ex, caption: "Error reading the file", buttons: MessageBoxButtons.OK, icon: MessageBoxIcon.Error);
                }
            }

            this.Invalidate();
            this.Update();
        }

        private void nIterationsTextBox_TextChanged(object sender, EventArgs e)
        {
            int iterations;
            if (!string.IsNullOrWhiteSpace(this.nIterationsTextBox.Text))
            {
                if (int.TryParse(this.nIterationsTextBox.Text, out iterations))
                {
                    this.iterate_n_button.Enabled = true;
                }
                else
                {
                    MessageBox.Show(text: "Invalid iterations value", caption: "Invalid iterations value", buttons: MessageBoxButtons.OK, icon: MessageBoxIcon.Error);
                    this.nIterationsTextBox.Text = null;
                }
            }

            this.Invalidate();
            this.Update();
        }

        private void iterate_n_button_Click(object sender, EventArgs e)
        {
            var original = this.iterate_n_button.Text;
            this.iterate_n_button.Text = "Computing";
            this.iterate_n_button.Enabled = false;
            this.iterate_once_button.Enabled = false;

            int iterations;
            if (!string.IsNullOrWhiteSpace(this.nIterationsTextBox.Text))
            {
                if (int.TryParse(this.nIterationsTextBox.Text, out iterations))
                {
                    ControlRotation control;
                    UltraSonicMeasurement measurement;
                    for(int i=0;i < iterations; i++)
                    {
                        control = this.PopNextControlAction();
                        measurement = this.PopNextMeasurement();

                        this.model.Iterate(control, measurement);
                        this.newIteration = true;

                        this.Invalidate();
                        this.Update();
                    }
                }
                else
                {
                    MessageBox.Show(text: "Invalid iterations value", caption: "Invalid iterations value", buttons: MessageBoxButtons.OK, icon: MessageBoxIcon.Error);
                }
            }

            this.iterate_once_button.Enabled = true;
            this.iterate_n_button.Enabled = true;
            this.iterate_n_button.Text = original;
        }
    }
}
