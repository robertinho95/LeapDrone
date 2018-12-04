﻿namespace LeapDrone
{
    partial class Form1
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
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea1 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend1 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series1 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Title title1 = new System.Windows.Forms.DataVisualization.Charting.Title();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea2 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend2 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series2 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Title title2 = new System.Windows.Forms.DataVisualization.Charting.Title();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea3 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend3 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series3 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Title title3 = new System.Windows.Forms.DataVisualization.Charting.Title();
            System.Windows.Forms.DataVisualization.Charting.ChartArea chartArea4 = new System.Windows.Forms.DataVisualization.Charting.ChartArea();
            System.Windows.Forms.DataVisualization.Charting.Legend legend4 = new System.Windows.Forms.DataVisualization.Charting.Legend();
            System.Windows.Forms.DataVisualization.Charting.Series series4 = new System.Windows.Forms.DataVisualization.Charting.Series();
            System.Windows.Forms.DataVisualization.Charting.Title title4 = new System.Windows.Forms.DataVisualization.Charting.Title();
            this.displayID = new System.Windows.Forms.Label();
            this.displayTimestamp = new System.Windows.Forms.Label();
            this.displayFPS = new System.Windows.Forms.Label();
            this.displayHandCount = new System.Windows.Forms.Label();
            this.displayImages = new System.Windows.Forms.PictureBox();
            this.RollLabel = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.heightTXT = new System.Windows.Forms.Label();
            this.yawTXT = new System.Windows.Forms.Label();
            this.pitchTXT = new System.Windows.Forms.Label();
            this.rollTXT = new System.Windows.Forms.Label();
            this.heightJOY = new System.Windows.Forms.Label();
            this.yawJOY = new System.Windows.Forms.Label();
            this.pitchJOY = new System.Windows.Forms.Label();
            this.rollJOY = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.maxValJOY = new System.Windows.Forms.Label();
            this.minValJOY = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.heightProg = new System.Windows.Forms.ProgressBar();
            this.yawProg = new System.Windows.Forms.ProgressBar();
            this.rollProg = new System.Windows.Forms.ProgressBar();
            this.pitchProg = new System.Windows.Forms.ProgressBar();
            this.rollChart = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.pitchChart = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.heightChart = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.yawChart = new System.Windows.Forms.DataVisualization.Charting.Chart();
            this.frameCheckBox = new System.Windows.Forms.CheckBox();
            this.splineRadioButton = new System.Windows.Forms.RadioButton();
            this.chartGroupBox = new System.Windows.Forms.GroupBox();
            this.linesRadioButton = new System.Windows.Forms.RadioButton();
            this.loggerButton = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.displayImages)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.rollChart)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pitchChart)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.heightChart)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.yawChart)).BeginInit();
            this.chartGroupBox.SuspendLayout();
            this.SuspendLayout();
            // 
            // displayID
            // 
            this.displayID.AutoSize = true;
            this.displayID.Location = new System.Drawing.Point(728, 25);
            this.displayID.Name = "displayID";
            this.displayID.Size = new System.Drawing.Size(35, 13);
            this.displayID.TabIndex = 0;
            this.displayID.Text = "label1";
            // 
            // displayTimestamp
            // 
            this.displayTimestamp.AutoSize = true;
            this.displayTimestamp.Location = new System.Drawing.Point(728, 50);
            this.displayTimestamp.Name = "displayTimestamp";
            this.displayTimestamp.Size = new System.Drawing.Size(35, 13);
            this.displayTimestamp.TabIndex = 1;
            this.displayTimestamp.Text = "label2";
            // 
            // displayFPS
            // 
            this.displayFPS.AutoSize = true;
            this.displayFPS.Location = new System.Drawing.Point(728, 75);
            this.displayFPS.Name = "displayFPS";
            this.displayFPS.Size = new System.Drawing.Size(35, 13);
            this.displayFPS.TabIndex = 2;
            this.displayFPS.Text = "label3";
            // 
            // displayHandCount
            // 
            this.displayHandCount.AutoSize = true;
            this.displayHandCount.Location = new System.Drawing.Point(728, 100);
            this.displayHandCount.Name = "displayHandCount";
            this.displayHandCount.Size = new System.Drawing.Size(35, 13);
            this.displayHandCount.TabIndex = 3;
            this.displayHandCount.Text = "label4";
            // 
            // displayImages
            // 
            this.displayImages.Location = new System.Drawing.Point(15, 25);
            this.displayImages.Name = "displayImages";
            this.displayImages.Size = new System.Drawing.Size(640, 480);
            this.displayImages.SizeMode = System.Windows.Forms.PictureBoxSizeMode.CenterImage;
            this.displayImages.TabIndex = 4;
            this.displayImages.TabStop = false;
            // 
            // RollLabel
            // 
            this.RollLabel.AutoSize = true;
            this.RollLabel.Location = new System.Drawing.Point(662, 25);
            this.RollLabel.Name = "RollLabel";
            this.RollLabel.Size = new System.Drawing.Size(47, 13);
            this.RollLabel.TabIndex = 6;
            this.RollLabel.Text = "FrameID";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(662, 50);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(60, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "TimeStamp";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(662, 75);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(27, 13);
            this.label3.TabIndex = 8;
            this.label3.Text = "FPS";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(662, 150);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(31, 13);
            this.label4.TabIndex = 11;
            this.label4.Text = "Pitch";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(662, 125);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(25, 13);
            this.label5.TabIndex = 10;
            this.label5.Text = "Roll";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(662, 100);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(45, 13);
            this.label6.TabIndex = 9;
            this.label6.Text = "Hand n.";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(662, 200);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(38, 13);
            this.label1.TabIndex = 13;
            this.label1.Text = "Height";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(662, 175);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(28, 13);
            this.label7.TabIndex = 12;
            this.label7.Text = "Yaw";
            // 
            // heightTXT
            // 
            this.heightTXT.AutoSize = true;
            this.heightTXT.Location = new System.Drawing.Point(728, 200);
            this.heightTXT.Name = "heightTXT";
            this.heightTXT.Size = new System.Drawing.Size(35, 13);
            this.heightTXT.TabIndex = 17;
            this.heightTXT.Text = "label8";
            // 
            // yawTXT
            // 
            this.yawTXT.AutoSize = true;
            this.yawTXT.Location = new System.Drawing.Point(728, 175);
            this.yawTXT.Name = "yawTXT";
            this.yawTXT.Size = new System.Drawing.Size(35, 13);
            this.yawTXT.TabIndex = 16;
            this.yawTXT.Text = "label7";
            // 
            // pitchTXT
            // 
            this.pitchTXT.AutoSize = true;
            this.pitchTXT.Location = new System.Drawing.Point(728, 150);
            this.pitchTXT.Name = "pitchTXT";
            this.pitchTXT.Size = new System.Drawing.Size(35, 13);
            this.pitchTXT.TabIndex = 15;
            this.pitchTXT.Text = "label6";
            // 
            // rollTXT
            // 
            this.rollTXT.AutoSize = true;
            this.rollTXT.Location = new System.Drawing.Point(728, 125);
            this.rollTXT.Name = "rollTXT";
            this.rollTXT.Size = new System.Drawing.Size(35, 13);
            this.rollTXT.TabIndex = 14;
            this.rollTXT.Text = "label5";
            // 
            // heightJOY
            // 
            this.heightJOY.AutoSize = true;
            this.heightJOY.Location = new System.Drawing.Point(728, 300);
            this.heightJOY.Name = "heightJOY";
            this.heightJOY.Size = new System.Drawing.Size(35, 13);
            this.heightJOY.TabIndex = 25;
            this.heightJOY.Text = "label8";
            // 
            // yawJOY
            // 
            this.yawJOY.AutoSize = true;
            this.yawJOY.Location = new System.Drawing.Point(728, 275);
            this.yawJOY.Name = "yawJOY";
            this.yawJOY.Size = new System.Drawing.Size(35, 13);
            this.yawJOY.TabIndex = 24;
            this.yawJOY.Text = "label7";
            // 
            // pitchJOY
            // 
            this.pitchJOY.AutoSize = true;
            this.pitchJOY.Location = new System.Drawing.Point(728, 250);
            this.pitchJOY.Name = "pitchJOY";
            this.pitchJOY.Size = new System.Drawing.Size(35, 13);
            this.pitchJOY.TabIndex = 23;
            this.pitchJOY.Text = "label6";
            // 
            // rollJOY
            // 
            this.rollJOY.AutoSize = true;
            this.rollJOY.Location = new System.Drawing.Point(728, 225);
            this.rollJOY.Name = "rollJOY";
            this.rollJOY.Size = new System.Drawing.Size(35, 13);
            this.rollJOY.TabIndex = 22;
            this.rollJOY.Text = "label5";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(662, 300);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(54, 13);
            this.label12.TabIndex = 21;
            this.label12.Text = "HeightJoy";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(662, 275);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(44, 13);
            this.label13.TabIndex = 20;
            this.label13.Text = "YawJoy";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(662, 250);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(47, 13);
            this.label14.TabIndex = 19;
            this.label14.Text = "PitchJoy";
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(662, 225);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(41, 13);
            this.label15.TabIndex = 18;
            this.label15.Text = "RollJoy";
            // 
            // maxValJOY
            // 
            this.maxValJOY.AutoSize = true;
            this.maxValJOY.Location = new System.Drawing.Point(728, 350);
            this.maxValJOY.Name = "maxValJOY";
            this.maxValJOY.Size = new System.Drawing.Size(35, 13);
            this.maxValJOY.TabIndex = 29;
            this.maxValJOY.Text = "label8";
            // 
            // minValJOY
            // 
            this.minValJOY.AutoSize = true;
            this.minValJOY.Location = new System.Drawing.Point(728, 325);
            this.minValJOY.Name = "minValJOY";
            this.minValJOY.Size = new System.Drawing.Size(35, 13);
            this.minValJOY.TabIndex = 28;
            this.minValJOY.Text = "label7";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(662, 350);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(57, 13);
            this.label10.TabIndex = 27;
            this.label10.Text = "maxValJoy";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(662, 325);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(54, 13);
            this.label11.TabIndex = 26;
            this.label11.Text = "minValJoy";
            // 
            // heightProg
            // 
            this.heightProg.Location = new System.Drawing.Point(769, 301);
            this.heightProg.Name = "heightProg";
            this.heightProg.Size = new System.Drawing.Size(109, 12);
            this.heightProg.TabIndex = 30;
            // 
            // yawProg
            // 
            this.yawProg.Location = new System.Drawing.Point(770, 275);
            this.yawProg.Name = "yawProg";
            this.yawProg.Size = new System.Drawing.Size(109, 12);
            this.yawProg.TabIndex = 31;
            // 
            // rollProg
            // 
            this.rollProg.Location = new System.Drawing.Point(770, 226);
            this.rollProg.Name = "rollProg";
            this.rollProg.Size = new System.Drawing.Size(109, 12);
            this.rollProg.TabIndex = 33;
            // 
            // pitchProg
            // 
            this.pitchProg.Location = new System.Drawing.Point(769, 251);
            this.pitchProg.Name = "pitchProg";
            this.pitchProg.Size = new System.Drawing.Size(109, 12);
            this.pitchProg.TabIndex = 32;
            // 
            // rollChart
            // 
            chartArea1.Name = "ChartArea1";
            this.rollChart.ChartAreas.Add(chartArea1);
            legend1.Enabled = false;
            legend1.Name = "Legend1";
            this.rollChart.Legends.Add(legend1);
            this.rollChart.Location = new System.Drawing.Point(15, 511);
            this.rollChart.Name = "rollChart";
            series1.BorderWidth = 3;
            series1.ChartArea = "ChartArea1";
            series1.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series1.IsVisibleInLegend = false;
            series1.Legend = "Legend1";
            series1.Name = "rollSeries";
            series1.XValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt64;
            series1.YValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt32;
            this.rollChart.Series.Add(series1);
            this.rollChart.Size = new System.Drawing.Size(863, 181);
            this.rollChart.TabIndex = 34;
            this.rollChart.Text = "Roll";
            title1.Name = "Title1";
            title1.Text = "Roll";
            this.rollChart.Titles.Add(title1);
            // 
            // pitchChart
            // 
            chartArea2.Name = "ChartArea1";
            this.pitchChart.ChartAreas.Add(chartArea2);
            legend2.Enabled = false;
            legend2.Name = "Legend1";
            this.pitchChart.Legends.Add(legend2);
            this.pitchChart.Location = new System.Drawing.Point(16, 698);
            this.pitchChart.Name = "pitchChart";
            series2.BorderWidth = 3;
            series2.ChartArea = "ChartArea1";
            series2.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series2.Color = System.Drawing.Color.Red;
            series2.IsVisibleInLegend = false;
            series2.Legend = "Legend1";
            series2.Name = "pitchSeries";
            series2.XValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt64;
            series2.YValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt32;
            this.pitchChart.Series.Add(series2);
            this.pitchChart.Size = new System.Drawing.Size(863, 181);
            this.pitchChart.TabIndex = 35;
            this.pitchChart.Text = "Pitch";
            title2.Name = "Title1";
            title2.Text = "Pitch";
            this.pitchChart.Titles.Add(title2);
            // 
            // heightChart
            // 
            chartArea3.Name = "ChartArea1";
            this.heightChart.ChartAreas.Add(chartArea3);
            legend3.Enabled = false;
            legend3.Name = "Legend1";
            this.heightChart.Legends.Add(legend3);
            this.heightChart.Location = new System.Drawing.Point(16, 1072);
            this.heightChart.Name = "heightChart";
            series3.BorderWidth = 3;
            series3.ChartArea = "ChartArea1";
            series3.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series3.Color = System.Drawing.Color.Fuchsia;
            series3.IsVisibleInLegend = false;
            series3.Legend = "Legend1";
            series3.Name = "heightSeries";
            series3.XValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt64;
            series3.YValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt32;
            this.heightChart.Series.Add(series3);
            this.heightChart.Size = new System.Drawing.Size(863, 181);
            this.heightChart.TabIndex = 37;
            this.heightChart.Text = "Height";
            title3.Name = "Title1";
            title3.Text = "Height";
            this.heightChart.Titles.Add(title3);
            // 
            // yawChart
            // 
            chartArea4.Name = "ChartArea1";
            this.yawChart.ChartAreas.Add(chartArea4);
            legend4.Enabled = false;
            legend4.Name = "Legend1";
            this.yawChart.Legends.Add(legend4);
            this.yawChart.Location = new System.Drawing.Point(15, 885);
            this.yawChart.Name = "yawChart";
            series4.BorderWidth = 3;
            series4.ChartArea = "ChartArea1";
            series4.ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Spline;
            series4.Color = System.Drawing.Color.Lime;
            series4.IsVisibleInLegend = false;
            series4.Legend = "Legend1";
            series4.Name = "yawSeries";
            series4.XValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt64;
            series4.YValueType = System.Windows.Forms.DataVisualization.Charting.ChartValueType.UInt32;
            this.yawChart.Series.Add(series4);
            this.yawChart.Size = new System.Drawing.Size(863, 181);
            this.yawChart.TabIndex = 36;
            this.yawChart.Text = "Yaw";
            title4.Name = "Title1";
            title4.Text = "Yaw";
            this.yawChart.Titles.Add(title4);
            // 
            // frameCheckBox
            // 
            this.frameCheckBox.AutoSize = true;
            this.frameCheckBox.Location = new System.Drawing.Point(665, 488);
            this.frameCheckBox.Name = "frameCheckBox";
            this.frameCheckBox.Size = new System.Drawing.Size(83, 17);
            this.frameCheckBox.TabIndex = 38;
            this.frameCheckBox.Text = "View frames";
            this.frameCheckBox.UseVisualStyleBackColor = true;
            // 
            // splineRadioButton
            // 
            this.splineRadioButton.AutoSize = true;
            this.splineRadioButton.Checked = true;
            this.splineRadioButton.Location = new System.Drawing.Point(6, 19);
            this.splineRadioButton.Name = "splineRadioButton";
            this.splineRadioButton.Size = new System.Drawing.Size(54, 17);
            this.splineRadioButton.TabIndex = 39;
            this.splineRadioButton.TabStop = true;
            this.splineRadioButton.Text = "Spline";
            this.splineRadioButton.UseVisualStyleBackColor = true;
            this.splineRadioButton.CheckedChanged += new System.EventHandler(this.chartRadioButton_CheckedChanged);
            // 
            // chartGroupBox
            // 
            this.chartGroupBox.Controls.Add(this.linesRadioButton);
            this.chartGroupBox.Controls.Add(this.splineRadioButton);
            this.chartGroupBox.Location = new System.Drawing.Point(665, 375);
            this.chartGroupBox.Name = "chartGroupBox";
            this.chartGroupBox.Size = new System.Drawing.Size(83, 66);
            this.chartGroupBox.TabIndex = 40;
            this.chartGroupBox.TabStop = false;
            this.chartGroupBox.Text = "Chart type";
            // 
            // linesRadioButton
            // 
            this.linesRadioButton.AutoSize = true;
            this.linesRadioButton.Location = new System.Drawing.Point(6, 42);
            this.linesRadioButton.Name = "linesRadioButton";
            this.linesRadioButton.Size = new System.Drawing.Size(50, 17);
            this.linesRadioButton.TabIndex = 40;
            this.linesRadioButton.Text = "Lines";
            this.linesRadioButton.UseVisualStyleBackColor = true;
            // 
            // loggerButton
            // 
            this.loggerButton.Location = new System.Drawing.Point(665, 447);
            this.loggerButton.Name = "loggerButton";
            this.loggerButton.Size = new System.Drawing.Size(75, 23);
            this.loggerButton.TabIndex = 41;
            this.loggerButton.Text = "Start logger";
            this.loggerButton.UseVisualStyleBackColor = true;
            this.loggerButton.Click += new System.EventHandler(this.loggerButton_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(891, 1267);
            this.Controls.Add(this.loggerButton);
            this.Controls.Add(this.chartGroupBox);
            this.Controls.Add(this.frameCheckBox);
            this.Controls.Add(this.heightChart);
            this.Controls.Add(this.yawChart);
            this.Controls.Add(this.pitchChart);
            this.Controls.Add(this.rollChart);
            this.Controls.Add(this.rollProg);
            this.Controls.Add(this.pitchProg);
            this.Controls.Add(this.yawProg);
            this.Controls.Add(this.heightProg);
            this.Controls.Add(this.maxValJOY);
            this.Controls.Add(this.minValJOY);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.heightJOY);
            this.Controls.Add(this.yawJOY);
            this.Controls.Add(this.pitchJOY);
            this.Controls.Add(this.rollJOY);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.heightTXT);
            this.Controls.Add(this.yawTXT);
            this.Controls.Add(this.pitchTXT);
            this.Controls.Add(this.rollTXT);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.RollLabel);
            this.Controls.Add(this.displayImages);
            this.Controls.Add(this.displayHandCount);
            this.Controls.Add(this.displayFPS);
            this.Controls.Add(this.displayTimestamp);
            this.Controls.Add(this.displayID);
            this.Name = "Form1";
            this.Text = "Frame Data";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.onClose);
            ((System.ComponentModel.ISupportInitialize)(this.displayImages)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.rollChart)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pitchChart)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.heightChart)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.yawChart)).EndInit();
            this.chartGroupBox.ResumeLayout(false);
            this.chartGroupBox.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label displayID;
        private System.Windows.Forms.Label displayTimestamp;
        private System.Windows.Forms.Label displayFPS;
        private System.Windows.Forms.Label displayHandCount;
        private System.Windows.Forms.PictureBox displayImages;
        private System.Windows.Forms.Label RollLabel;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label heightTXT;
        private System.Windows.Forms.Label yawTXT;
        private System.Windows.Forms.Label pitchTXT;
        private System.Windows.Forms.Label rollTXT;
        private System.Windows.Forms.Label heightJOY;
        private System.Windows.Forms.Label yawJOY;
        private System.Windows.Forms.Label pitchJOY;
        private System.Windows.Forms.Label rollJOY;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label maxValJOY;
        private System.Windows.Forms.Label minValJOY;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.ProgressBar heightProg;
        private System.Windows.Forms.ProgressBar yawProg;
        private System.Windows.Forms.ProgressBar rollProg;
        private System.Windows.Forms.ProgressBar pitchProg;
        private System.Windows.Forms.DataVisualization.Charting.Chart rollChart;
        private System.Windows.Forms.DataVisualization.Charting.Chart pitchChart;
        private System.Windows.Forms.DataVisualization.Charting.Chart heightChart;
        private System.Windows.Forms.DataVisualization.Charting.Chart yawChart;
        private System.Windows.Forms.CheckBox frameCheckBox;
        private System.Windows.Forms.RadioButton splineRadioButton;
        private System.Windows.Forms.GroupBox chartGroupBox;
        private System.Windows.Forms.RadioButton linesRadioButton;
        private System.Windows.Forms.Button loggerButton;
    }
}
