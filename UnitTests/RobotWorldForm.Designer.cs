namespace UnitTests
{
    partial class RobotWorldForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (this.inputData != null))
            {
                this.inputData.Close();
                this.inputData.Dispose();
            }

            if (disposing && (this.measurementInputData != null))
            {
                this.measurementInputData.Close();
                this.measurementInputData.Dispose();
            }

            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.iterate_once_button = new System.Windows.Forms.Button();
            this.iterate_n_button = new System.Windows.Forms.Button();
            this.numParticlesBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.particlesListBox = new System.Windows.Forms.ListBox();
            this.label2 = new System.Windows.Forms.Label();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.drawMap_checkBox = new System.Windows.Forms.CheckBox();
            this.nIterationsTextBox = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.v_textBox = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.w_textBox = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.cellsX_textBox = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.cellsY_textBox = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.cellSize_textBox = new System.Windows.Forms.TextBox();
            this.apply_button = new System.Windows.Forms.Button();
            this.reset_values_button = new System.Windows.Forms.Button();
            this.read_from_file_checkBox = new System.Windows.Forms.CheckBox();
            this.filename_textBox = new System.Windows.Forms.TextBox();
            this.browse_button = new System.Windows.Forms.Button();
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.sensor1_textBox = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.sensor2_textBox = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.browse_measurement_button = new System.Windows.Forms.Button();
            this.measurements_filename_textBox = new System.Windows.Forms.TextBox();
            this.read_measurement_from_file_checkBox = new System.Windows.Forms.CheckBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.save_state_checkBox = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.SuspendLayout();
            // 
            // iterate_once_button
            // 
            this.iterate_once_button.Location = new System.Drawing.Point(18, 30);
            this.iterate_once_button.Name = "iterate_once_button";
            this.iterate_once_button.Size = new System.Drawing.Size(91, 23);
            this.iterate_once_button.TabIndex = 1;
            this.iterate_once_button.Text = "Iterate Once";
            this.iterate_once_button.UseVisualStyleBackColor = true;
            this.iterate_once_button.Click += new System.EventHandler(this.button1_Click);
            // 
            // iterate_n_button
            // 
            this.iterate_n_button.Enabled = false;
            this.iterate_n_button.Location = new System.Drawing.Point(18, 59);
            this.iterate_n_button.Name = "iterate_n_button";
            this.iterate_n_button.Size = new System.Drawing.Size(91, 23);
            this.iterate_n_button.TabIndex = 2;
            this.iterate_n_button.Text = "Iterate n times";
            this.iterate_n_button.UseVisualStyleBackColor = true;
            this.iterate_n_button.Click += new System.EventHandler(this.iterate_n_button_Click);
            // 
            // numParticlesBox
            // 
            this.numParticlesBox.Location = new System.Drawing.Point(120, 76);
            this.numParticlesBox.MaxLength = 10;
            this.numParticlesBox.Name = "numParticlesBox";
            this.numParticlesBox.Size = new System.Drawing.Size(108, 20);
            this.numParticlesBox.TabIndex = 3;
            this.numParticlesBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.numParticlesBox.TextChanged += new System.EventHandler(this.numParticlesBox_TextChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(13, 80);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(104, 13);
            this.label1.TabIndex = 4;
            this.label1.Text = "Number of particles: ";
            // 
            // particlesListBox
            // 
            this.particlesListBox.FormattingEnabled = true;
            this.particlesListBox.Location = new System.Drawing.Point(13, 49);
            this.particlesListBox.Name = "particlesListBox";
            this.particlesListBox.Size = new System.Drawing.Size(221, 264);
            this.particlesListBox.TabIndex = 5;
            this.particlesListBox.SelectedIndexChanged += new System.EventHandler(this.particlesListBox_SelectedIndexChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(10, 22);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(47, 13);
            this.label2.TabIndex = 6;
            this.label2.Text = "Particles";
            // 
            // pictureBox1
            // 
            this.pictureBox1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.pictureBox1.Location = new System.Drawing.Point(12, 9);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(505, 505);
            this.pictureBox1.TabIndex = 7;
            this.pictureBox1.TabStop = false;
            // 
            // drawMap_checkBox
            // 
            this.drawMap_checkBox.AutoSize = true;
            this.drawMap_checkBox.Location = new System.Drawing.Point(91, 19);
            this.drawMap_checkBox.Name = "drawMap_checkBox";
            this.drawMap_checkBox.Size = new System.Drawing.Size(146, 17);
            this.drawMap_checkBox.TabIndex = 8;
            this.drawMap_checkBox.Text = "Draw map when selected";
            this.drawMap_checkBox.UseVisualStyleBackColor = true;
            this.drawMap_checkBox.CheckedChanged += new System.EventHandler(this.drawMap_checkBox_CheckedChanged);
            // 
            // nIterationsTextBox
            // 
            this.nIterationsTextBox.Location = new System.Drawing.Point(120, 61);
            this.nIterationsTextBox.MaxLength = 10;
            this.nIterationsTextBox.Name = "nIterationsTextBox";
            this.nIterationsTextBox.Size = new System.Drawing.Size(100, 20);
            this.nIterationsTextBox.TabIndex = 9;
            this.nIterationsTextBox.TextChanged += new System.EventHandler(this.nIterationsTextBox_TextChanged);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(18, 16);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(109, 13);
            this.label3.TabIndex = 10;
            this.label3.Text = "Manual control action";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(18, 35);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(17, 13);
            this.label4.TabIndex = 11;
            this.label4.Text = "V:";
            // 
            // v_textBox
            // 
            this.v_textBox.Location = new System.Drawing.Point(69, 34);
            this.v_textBox.MaxLength = 10;
            this.v_textBox.Name = "v_textBox";
            this.v_textBox.Size = new System.Drawing.Size(100, 20);
            this.v_textBox.TabIndex = 12;
            this.v_textBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(18, 61);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(44, 13);
            this.label5.TabIndex = 13;
            this.label5.Text = "Omega:";
            // 
            // w_textBox
            // 
            this.w_textBox.Location = new System.Drawing.Point(69, 61);
            this.w_textBox.MaxLength = 10;
            this.w_textBox.Name = "w_textBox";
            this.w_textBox.Size = new System.Drawing.Size(100, 20);
            this.w_textBox.TabIndex = 14;
            this.w_textBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(176, 36);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(43, 13);
            this.label6.TabIndex = 15;
            this.label6.Text = "cm/sec";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(176, 67);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(44, 13);
            this.label7.TabIndex = 16;
            this.label7.Text = "rad/sec";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(12, 20);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(103, 13);
            this.label8.TabIndex = 17;
            this.label8.Text = "Grid size (Hor x Ver):";
            // 
            // cellsX_textBox
            // 
            this.cellsX_textBox.Location = new System.Drawing.Point(121, 17);
            this.cellsX_textBox.MaxLength = 5;
            this.cellsX_textBox.Name = "cellsX_textBox";
            this.cellsX_textBox.Size = new System.Drawing.Size(44, 20);
            this.cellsX_textBox.TabIndex = 18;
            this.cellsX_textBox.TextChanged += new System.EventHandler(this.cellsX_textBox_TextChanged);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(168, 24);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(12, 13);
            this.label9.TabIndex = 19;
            this.label9.Text = "x";
            // 
            // cellsY_textBox
            // 
            this.cellsY_textBox.Location = new System.Drawing.Point(183, 17);
            this.cellsY_textBox.MaxLength = 5;
            this.cellsY_textBox.Name = "cellsY_textBox";
            this.cellsY_textBox.Size = new System.Drawing.Size(44, 20);
            this.cellsY_textBox.TabIndex = 20;
            this.cellsY_textBox.TextChanged += new System.EventHandler(this.cellsY_textBox_TextChanged);
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(13, 48);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(68, 13);
            this.label10.TabIndex = 21;
            this.label10.Text = "Cell size (cm)";
            // 
            // cellSize_textBox
            // 
            this.cellSize_textBox.Location = new System.Drawing.Point(121, 45);
            this.cellSize_textBox.MaxLength = 10;
            this.cellSize_textBox.Name = "cellSize_textBox";
            this.cellSize_textBox.Size = new System.Drawing.Size(106, 20);
            this.cellSize_textBox.TabIndex = 22;
            this.cellSize_textBox.TextChanged += new System.EventHandler(this.cellSize_textBox_TextChanged);
            // 
            // apply_button
            // 
            this.apply_button.Enabled = false;
            this.apply_button.Location = new System.Drawing.Point(75, 103);
            this.apply_button.Name = "apply_button";
            this.apply_button.Size = new System.Drawing.Size(62, 23);
            this.apply_button.TabIndex = 23;
            this.apply_button.Text = "Apply";
            this.apply_button.UseVisualStyleBackColor = true;
            this.apply_button.Click += new System.EventHandler(this.apply_button_Click);
            // 
            // reset_values_button
            // 
            this.reset_values_button.Enabled = false;
            this.reset_values_button.Location = new System.Drawing.Point(153, 102);
            this.reset_values_button.Name = "reset_values_button";
            this.reset_values_button.Size = new System.Drawing.Size(75, 23);
            this.reset_values_button.TabIndex = 24;
            this.reset_values_button.Text = "Reset";
            this.reset_values_button.UseVisualStyleBackColor = true;
            this.reset_values_button.Click += new System.EventHandler(this.reset_values_button_Click);
            // 
            // read_from_file_checkBox
            // 
            this.read_from_file_checkBox.AutoSize = true;
            this.read_from_file_checkBox.Location = new System.Drawing.Point(18, 93);
            this.read_from_file_checkBox.Name = "read_from_file_checkBox";
            this.read_from_file_checkBox.Size = new System.Drawing.Size(135, 17);
            this.read_from_file_checkBox.TabIndex = 25;
            this.read_from_file_checkBox.Text = "Control actions from file";
            this.read_from_file_checkBox.UseVisualStyleBackColor = true;
            this.read_from_file_checkBox.CheckedChanged += new System.EventHandler(this.read_from_file_checkBox_CheckedChanged);
            // 
            // filename_textBox
            // 
            this.filename_textBox.Enabled = false;
            this.filename_textBox.Location = new System.Drawing.Point(18, 117);
            this.filename_textBox.Name = "filename_textBox";
            this.filename_textBox.Size = new System.Drawing.Size(177, 20);
            this.filename_textBox.TabIndex = 26;
            this.filename_textBox.TextChanged += new System.EventHandler(this.filename_textBox_TextChanged);
            // 
            // browse_button
            // 
            this.browse_button.Location = new System.Drawing.Point(201, 117);
            this.browse_button.Name = "browse_button";
            this.browse_button.Size = new System.Drawing.Size(56, 23);
            this.browse_button.TabIndex = 27;
            this.browse_button.Text = "Browse";
            this.browse_button.UseVisualStyleBackColor = true;
            this.browse_button.Click += new System.EventHandler(this.button1_Click_1);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(24, 28);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(113, 13);
            this.label11.TabIndex = 28;
            this.label11.Text = "Manual measurements";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(24, 52);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(106, 13);
            this.label12.TabIndex = 29;
            this.label12.Text = "Sensor 1 (cm @Pi/2)";
            // 
            // sensor1_textBox
            // 
            this.sensor1_textBox.Location = new System.Drawing.Point(149, 52);
            this.sensor1_textBox.MaxLength = 10;
            this.sensor1_textBox.Name = "sensor1_textBox";
            this.sensor1_textBox.Size = new System.Drawing.Size(76, 20);
            this.sensor1_textBox.TabIndex = 30;
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(24, 80);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(109, 13);
            this.label13.TabIndex = 31;
            this.label13.Text = "Sensor 2 (cm @-Pi/2)";
            // 
            // sensor2_textBox
            // 
            this.sensor2_textBox.Location = new System.Drawing.Point(149, 80);
            this.sensor2_textBox.MaxLength = 10;
            this.sensor2_textBox.Name = "sensor2_textBox";
            this.sensor2_textBox.Size = new System.Drawing.Size(77, 20);
            this.sensor2_textBox.TabIndex = 32;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.v_textBox);
            this.groupBox1.Controls.Add(this.label5);
            this.groupBox1.Controls.Add(this.w_textBox);
            this.groupBox1.Controls.Add(this.label6);
            this.groupBox1.Controls.Add(this.browse_button);
            this.groupBox1.Controls.Add(this.label7);
            this.groupBox1.Controls.Add(this.filename_textBox);
            this.groupBox1.Controls.Add(this.read_from_file_checkBox);
            this.groupBox1.Location = new System.Drawing.Point(803, 22);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(269, 172);
            this.groupBox1.TabIndex = 33;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Control action";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.browse_measurement_button);
            this.groupBox2.Controls.Add(this.measurements_filename_textBox);
            this.groupBox2.Controls.Add(this.read_measurement_from_file_checkBox);
            this.groupBox2.Controls.Add(this.sensor1_textBox);
            this.groupBox2.Controls.Add(this.label12);
            this.groupBox2.Controls.Add(this.sensor2_textBox);
            this.groupBox2.Controls.Add(this.label13);
            this.groupBox2.Controls.Add(this.label11);
            this.groupBox2.Location = new System.Drawing.Point(803, 211);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(269, 172);
            this.groupBox2.TabIndex = 34;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Measurements";
            // 
            // browse_measurement_button
            // 
            this.browse_measurement_button.Location = new System.Drawing.Point(201, 134);
            this.browse_measurement_button.Name = "browse_measurement_button";
            this.browse_measurement_button.Size = new System.Drawing.Size(56, 23);
            this.browse_measurement_button.TabIndex = 35;
            this.browse_measurement_button.Text = "Browse";
            this.browse_measurement_button.UseVisualStyleBackColor = true;
            this.browse_measurement_button.Click += new System.EventHandler(this.browse_measurement_button_Click);
            // 
            // measurements_filename_textBox
            // 
            this.measurements_filename_textBox.Enabled = false;
            this.measurements_filename_textBox.Location = new System.Drawing.Point(18, 137);
            this.measurements_filename_textBox.Name = "measurements_filename_textBox";
            this.measurements_filename_textBox.Size = new System.Drawing.Size(177, 20);
            this.measurements_filename_textBox.TabIndex = 34;
            this.measurements_filename_textBox.TextChanged += new System.EventHandler(this.measurements_filename_textBox_TextChanged);
            // 
            // read_measurement_from_file_checkBox
            // 
            this.read_measurement_from_file_checkBox.AutoSize = true;
            this.read_measurement_from_file_checkBox.Location = new System.Drawing.Point(18, 114);
            this.read_measurement_from_file_checkBox.Name = "read_measurement_from_file_checkBox";
            this.read_measurement_from_file_checkBox.Size = new System.Drawing.Size(134, 17);
            this.read_measurement_from_file_checkBox.TabIndex = 33;
            this.read_measurement_from_file_checkBox.Text = "Measurements from file";
            this.read_measurement_from_file_checkBox.UseVisualStyleBackColor = true;
            this.read_measurement_from_file_checkBox.CheckedChanged += new System.EventHandler(this.read_measurement_from_file_checkBox_CheckedChanged);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.iterate_once_button);
            this.groupBox3.Controls.Add(this.iterate_n_button);
            this.groupBox3.Controls.Add(this.nIterationsTextBox);
            this.groupBox3.Location = new System.Drawing.Point(803, 396);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(269, 118);
            this.groupBox3.TabIndex = 35;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Iteration control";
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.save_state_checkBox);
            this.groupBox4.Controls.Add(this.label2);
            this.groupBox4.Controls.Add(this.particlesListBox);
            this.groupBox4.Controls.Add(this.drawMap_checkBox);
            this.groupBox4.Location = new System.Drawing.Point(539, 167);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(249, 345);
            this.groupBox4.TabIndex = 36;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Current state";
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.label8);
            this.groupBox5.Controls.Add(this.numParticlesBox);
            this.groupBox5.Controls.Add(this.label1);
            this.groupBox5.Controls.Add(this.cellsX_textBox);
            this.groupBox5.Controls.Add(this.label9);
            this.groupBox5.Controls.Add(this.reset_values_button);
            this.groupBox5.Controls.Add(this.cellsY_textBox);
            this.groupBox5.Controls.Add(this.apply_button);
            this.groupBox5.Controls.Add(this.label10);
            this.groupBox5.Controls.Add(this.cellSize_textBox);
            this.groupBox5.Location = new System.Drawing.Point(539, 22);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(249, 139);
            this.groupBox5.TabIndex = 37;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "World options";
            // 
            // save_state_checkBox
            // 
            this.save_state_checkBox.AutoSize = true;
            this.save_state_checkBox.Location = new System.Drawing.Point(16, 322);
            this.save_state_checkBox.Name = "save_state_checkBox";
            this.save_state_checkBox.Size = new System.Drawing.Size(114, 17);
            this.save_state_checkBox.TabIndex = 9;
            this.save_state_checkBox.Text = "Save state to a file";
            this.save_state_checkBox.UseVisualStyleBackColor = true;
            // 
            // RobotWorldForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1084, 524);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.pictureBox1);
            this.MaximumSize = new System.Drawing.Size(1100, 563);
            this.MinimumSize = new System.Drawing.Size(1100, 563);
            this.Name = "RobotWorldForm";
            this.Text = "RobotWorldForm";
            this.WindowState = System.Windows.Forms.FormWindowState.Maximized;
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.RobotWorldForm_Paint);
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button iterate_once_button;
        private System.Windows.Forms.Button iterate_n_button;
        private System.Windows.Forms.TextBox numParticlesBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ListBox particlesListBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.CheckBox drawMap_checkBox;
        private System.Windows.Forms.TextBox nIterationsTextBox;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox v_textBox;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox w_textBox;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox cellsX_textBox;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox cellsY_textBox;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox cellSize_textBox;
        private System.Windows.Forms.Button apply_button;
        private System.Windows.Forms.Button reset_values_button;
        private System.Windows.Forms.CheckBox read_from_file_checkBox;
        private System.Windows.Forms.TextBox filename_textBox;
        private System.Windows.Forms.Button browse_button;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox sensor1_textBox;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox sensor2_textBox;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox measurements_filename_textBox;
        private System.Windows.Forms.CheckBox read_measurement_from_file_checkBox;
        private System.Windows.Forms.Button browse_measurement_button;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.CheckBox save_state_checkBox;
    }
}